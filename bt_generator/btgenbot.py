# PyQt5 import
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QTextEdit, QPushButton
from PyQt5.QtCore import Qt

from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel
import torch, accelerate
from pathlib import Path
from paramiko import SSHClient
from scp import SCPClient
import yaml
import re
from pathlib import Path

class InputWindow(QWidget):
    # Define the GUI layout
    def __init__(self):
        super().__init__()
        self.initUI()

    # Initialize the GUI
    def initUI(self):
        self.setWindowTitle('BTGenBot')
        self.setGeometry(100, 100, 750, 450)

        layout = QVBoxLayout()

        self.example_task_label = QLabel('One-shot example task:')
        self.example_task_label.setStyleSheet("font-size: 16px; color: #333;")
        layout.addWidget(self.example_task_label)
        
        self.example_task_input = QTextEdit()
        self.example_task_input.setFixedHeight(80)  # Increasing input cell height
        self.example_task_input.setStyleSheet("font-size: 14px; padding: 5px;")
        self.example_task_input.setPlaceholderText("Please provide a description of the task for your one-shot example.")
        layout.addWidget(self.example_task_input)

        self.example_bt_label = QLabel('One-shot example output:')
        self.example_bt_label.setStyleSheet("font-size: 16px; color: #333;")
        layout.addWidget(self.example_bt_label)
        
        self.example_bt_input = QTextEdit()
        self.example_bt_input.setFixedHeight(80)  # Increasing input cell height
        self.example_bt_input.setStyleSheet("font-size: 14px; padding: 5px;")
        self.example_bt_input.setPlaceholderText("Please provide a behavior tree in XML format corresponding to the previous task.")
        layout.addWidget(self.example_bt_input)

        self.task_label = QLabel('Input:')
        self.task_label.setStyleSheet("font-size: 16px; color: #333;")
        layout.addWidget(self.task_label)
        
        self.task_input = QTextEdit()
        self.task_input.setFixedHeight(80)  # Increasing input cell height
        self.task_input.setStyleSheet("font-size: 14px; padding: 5px;")
        self.task_input.setPlaceholderText("Please provide a description of the new task.")
        layout.addWidget(self.task_input)
        
        self.error_label = QLabel()
        self.error_label.setStyleSheet("color: red; font-size: 14px;")
        layout.addWidget(self.error_label)
        
        self.success_label = QLabel()
        self.success_label.setStyleSheet("color: green; font-size: 14px;")
        layout.addWidget(self.success_label)

        self.submit_button = QPushButton('Submit')
        self.submit_button.setStyleSheet("font-size: 16px; padding: 10px; background-color: #007BFF; color: #FFF; border: none;")
        self.submit_button.clicked.connect(self.submit)
        layout.addWidget(self.submit_button)

        self.setLayout(layout)

    # Submit the input to the model
    def submit(self):
        example_task = self.example_task_input.toPlainText().strip()
        example_output = self.example_bt_input.toPlainText().strip()
        task = self.task_input.toPlainText().strip()
        
        # Check if all required fields are filled
        if not example_task or not example_output or not task:
            self.error_label.setText('Complete all the required fields.')
        else:
            self.generate(example_task, example_output, task)

    # Generate the behavior tree with the model and send it to the remote location
    def generate(self, example_task, example_output, task):
        # Define the path to the YAML configuration file
        config_file_path = Path.home() / 'BTGenBot/bt_generator/config/params.yaml'

        # Load parameters from the YAML configuration file
        with open(config_file_path, 'r') as file:
            params = yaml.safe_load(file)
        
        # Setup model and adapter IDs
        model_id = 'meta-llama/Llama-2-7b-chat-hf'
        adapter_id = 'AIRLab-POLIMI/llama-2-7b-chat-hf-btgenbot-adapter'

        # Load tokenizer
        tokenizer = AutoTokenizer.from_pretrained(
            pretrained_model_name_or_path = model_id,
            token=params.get("hf_token"),
        )

        # Load base model
        base_model = AutoModelForCausalLM.from_pretrained(
            pretrained_model_name_or_path = model_id,
            torch_dtype = torch.float16,
            device_map = "auto",
            trust_remote_code = True,
            token=params.get("hf_token"),
        )
        
        # Get the context from the YAML configuration file
        context = params.get("context")
        
        # Define the input prompt
        eval_prompt = "<s>[INST]" + context + example_task + "[/INST]</s>" + example_output + "[INST]" + task + "[/INST]"
        model_input = tokenizer(eval_prompt, return_tensors="pt").to("cuda")
        
        # Load the fine-tuned model
        finetuned_model = PeftModel.from_pretrained(base_model, adapter_id)
        finetuned_model = finetuned_model.merge_and_unload()
        
        # Generate behavior tree
        finetuned_model.eval()
        with torch.no_grad():
            result = tokenizer.decode(finetuned_model.generate(**model_input, max_new_tokens=1000)[0], skip_special_tokens=True)

        # Get the SSH parameters from the YAML configuration file
        host = params.get("host")
        username = params.get("username")
        password = params.get("password")
        local_dir = params.get("local_dir")
        remote_dir = params.get("remote_dir")
        
        # Define regular expression pattern to extract the behavior tree
        pattern = r'<root .*?</root>'
        matches = re.findall(pattern, result, re.DOTALL)

        # Define the path to the behavior tree XML file
        root_dir = Path.home() / "BTGenBot/bt_client/bt_xml"
        tree_xml = params.get("tree_name")

        # Check if the behavior tree was found
        if matches:
            self.success_label.setText('BT found, sending to remote location...')
            final_tree = matches[-1]
            
            # Write the behavior tree to an XML file
            with open(root_dir / tree_xml, "w") as xml_file:
                xml_file.write(final_tree)
            
            # Open SSH connection
            ssh = SSHClient()
            ssh.load_system_host_keys()
            ssh.connect(host, username=username, password=password)
            
            scp = SCPClient(ssh.get_transport())
            # Send the file to the remote location
            scp.put(files=local_dir + tree_xml, remote_path=remote_dir)
            self.success_label.setText('Success. Closing ssh connection.')
            scp.close()
            ssh.close()
        else:
            self.error_label.setText('BT not found. Possible generation error with the model.')
        
        
# Run the application
def main():
    app = QApplication(sys.argv)
    window = InputWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()