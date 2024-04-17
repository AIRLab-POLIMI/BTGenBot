# PyQt5 import
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QTextEdit, QPushButton, QRadioButton, QButtonGroup, QMessageBox

from transformers import AutoModelForCausalLM, AutoTokenizer
from peft import PeftModel
import torch, accelerate
from pathlib import Path
from paramiko import SSHClient
from scp import SCPClient
import yaml
import re
from pathlib import Path

# Define the main window, where the user can input the task and the one-shot example
class MainWindow(QWidget):
    # Define the GUI layout and initialize it
    def __init__(self, mode):
        super().__init__()
        self.mode = mode
        self.setWindowTitle('BTGenBot')
        self.setGeometry(200, 200, 600, 300)
        
        # Load the examples for the automatic mode
        examples_path = Path.home() / 'BTGenBot/bt_generator/config/example_retrieving.yaml'

        # Load parameters from the YAML configuration file
        with open(examples_path, 'r') as file:
            self.examples = yaml.safe_load(file)
            
        self.desc = self.examples.get("navigation_desc")
        self.tree = self.examples.get("navigation")
        
        self.layout = QVBoxLayout()
        
        # Standard mode
        if(self.mode == "standard"):
            self.example_task_label = QLabel('One-shot example task:')
            self.example_task_label.setStyleSheet("font-size: 16px; color: #333;")
            self.layout.addWidget(self.example_task_label)
            
            self.example_task_input = QTextEdit()
            self.example_task_input.setFixedHeight(80)  # Increasing input cell height
            self.example_task_input.setStyleSheet("font-size: 14px; padding: 5px;")
            self.example_task_input.setPlaceholderText("Please provide a description of the task for your one-shot example.")
            self.layout.addWidget(self.example_task_input)

            self.example_bt_label = QLabel('One-shot example output:')
            self.example_bt_label.setStyleSheet("font-size: 16px; color: #333;")
            self.layout.addWidget(self.example_bt_label)
            
            self.example_bt_input = QTextEdit()
            self.example_bt_input.setFixedHeight(80)  # Increasing input cell height
            self.example_bt_input.setStyleSheet("font-size: 14px; padding: 5px;")
            self.example_bt_input.setPlaceholderText("Please provide a behavior tree in XML format corresponding to the previous task.")
            self.layout.addWidget(self.example_bt_input)

            self.task_label = QLabel('Task:')
            self.task_label.setStyleSheet("font-size: 16px; color: #333;")
            self.layout.addWidget(self.task_label)
            
            self.task_input = QTextEdit()
            self.task_input.setFixedHeight(80)  # Increasing input cell height
            self.task_input.setStyleSheet("font-size: 14px; padding: 5px;")
            self.task_input.setPlaceholderText("Please provide a description of the new task.")
            self.layout.addWidget(self.task_input)
            
            self.submit_button = QPushButton('Submit')
            self.submit_button.setStyleSheet(
            "QPushButton {"
            "font-size: 18px;"
            "padding: 10px 20px;"
            "background-color: #007BFF;"
            "color: #FFF;"
            "border: 2px solid black;"
            "border-radius: 10px;"
            "}"
            "QPushButton:hover {"
            "background-color: #0056b3;"
            "}"
            )
            self.submit_button.clicked.connect(self.submit)
            self.layout.addWidget(self.submit_button)
            
        # Automatic retrieval mode
        elif self.mode == "automatic":
            # Task selection for the automatic example retrieval from the YAML file
            self.task_radio_group = QButtonGroup()
            tasks = [
                "Navigation",
                "Navigation with priority",
                "Navigation with fallback",
                "Navigation with arm activity",
                "Exploration",
                "Manipulator exploration",
                "Active vision and picking",
                "Material processing",
                "Multi-station assembly"
            ]
            for i, task in enumerate(tasks):
                radio_button = QRadioButton(task)
                self.task_radio_group.addButton(radio_button, i)
                self.layout.addWidget(radio_button)
                # Set "Navigation" as default
                if task == "Navigation":
                    radio_button.setChecked(True)  

            self.task_radio_group.buttonClicked.connect(self.task_selection_changed)

            
            self.task_label = QLabel('Task:')
            self.task_label.setStyleSheet("font-size: 16px; color: #333;")
            self.layout.addWidget(self.task_label)
            
            self.task_input = QTextEdit()
            self.task_input.setFixedHeight(80)  # Increasing input cell height
            self.task_input.setStyleSheet("font-size: 14px; padding: 5px;")
            self.task_input.setPlaceholderText("Please provide a description of the new task.")
            self.layout.addWidget(self.task_input)
            
            self.submit_button = QPushButton('Submit')
            self.submit_button.setStyleSheet(
            "QPushButton {"
            "font-size: 18px;"
            "padding: 10px 20px;"
            "background-color: #007BFF;"
            "color: #FFF;"
            "border: 2px solid black;"
            "border-radius: 10px;"
            "}"
            "QPushButton:hover {"
            "background-color: #0056b3;"
            "}"
            )
            self.submit_button.clicked.connect(self.submit)
            self.layout.addWidget(self.submit_button)
        
        self.setLayout(self.layout)
    
    # Submit the task and the one-shot example to the model
    def submit(self):
        if(self.mode == "standard"):
            example_task = self.example_task_input.toPlainText().strip()
            example_output = self.example_bt_input.toPlainText().strip()
            task = self.task_input.toPlainText().strip()
        elif(self.mode == "automatic"):
            example_task = self.desc
            example_output = self.tree
            task = self.task_input.toPlainText().strip()
        
        # Check if all required fields are filled
        if not example_task or not example_output or not task:
            QMessageBox.critical(self, "Error", "Complete all the required fields.")
        else:
            self.generate(example_task, example_output, task)
    
    # Task selection for the automatic example retrieval
    def task_selection_changed(self, button):
        index = self.task_radio_group.checkedId()
        
        # Navigation
        if index == 0:
            self.desc = self.examples.get("navigation_desc")
            self.tree = self.examples.get("navigation")
        # Navigation with priority
        elif index == 1:
            self.desc = self.examples.get("navigation_priority_desc")
            self.tree = self.examples.get("navigation_priority")
        # Navigation with fallback
        elif index == 2:
            self.desc = self.examples.get("navigation_fallback_desc")
            self.tree = self.examples.get("navigation_fallback")
        # Navigation with arm activity
        elif index == 3:
            self.desc = self.examples.get("navigation_arm_activity_desc")
            self.tree = self.examples.get("navigation_arm_activity")
        # Exploration
        elif index == 4:
            self.desc = self.examples.get("exploration_desc")
            self.tree = self.examples.get("exploration")
        # Manipulator exploration
        elif index == 5:
            self.desc = self.examples.get("manipulator_exploration_desc")
            self.tree = self.examples.get("manipulator_exploration")
        # Active vision and picking
        elif index == 6:
            self.desc = self.examples.get("active_vision_picking_desc")
            self.tree = self.examples.get("active_vision_picking")
        # Material processing
        elif index == 7:
            self.desc = self.examples.get("material_processing_desc")
            self.tree = self.examples.get("material_processing")
        # Multi-station assembly
        elif index == 8:
            self.desc = self.examples.get("multi_station_assembly_desc")
            self.tree = self.examples.get("multi_station_assembly")
            
            
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
            QMessageBox.information(self, "Success", "BT generated! Closing ssh connection.")
            scp.close()
            ssh.close()
        else:
            QMessageBox.critical(self, "Error", "BT not found. Error during generation, retry.")

# Define the mode selection window, initial selection between basic and automatic mode
class ModeSelectionWindow(QWidget):
    # Define the GUI layout and initialize it
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Mode Selection')
        self.setGeometry(100, 100, 400, 200)
        layout = QVBoxLayout()
        layout.setSpacing(20)

        self.standard_mode_button = QPushButton('Standard Mode')
        self.standard_mode_button.setStyleSheet(
            "QPushButton {"
            "font-size: 18px;"
            "padding: 10px 20px;"
            "background-color: #007BFF;"
            "color: #FFF;"
            "border: 2px solid black;"
            "border-radius: 10px;"
            "}"
            "QPushButton:hover {"
            "background-color: #0056b3;"
            "}"
        )
        self.standard_mode_button.clicked.connect(self.show_standard_mode)
        layout.addWidget(self.standard_mode_button)

        self.automatic_mode_button = QPushButton('Automatic Retrieval Mode')
        self.automatic_mode_button.setStyleSheet(
            "QPushButton {"
            "font-size: 18px;"
            "padding: 10px 20px;"
            "background-color: #007BFF;"
            "color: #FFF;"
            "border: 2px solid black;"
            "border-radius: 10px;"
            "}"
            "QPushButton:hover {"
            "background-color: #0056b3;"
            "}"
        )
        self.automatic_mode_button.clicked.connect(self.show_automatic_mode)
        layout.addWidget(self.automatic_mode_button)

        self.setLayout(layout)
        
    # Show the basic mode window 
    def show_standard_mode(self):
        self.standard_mode_window = MainWindow(mode="standard")
        self.standard_mode_window.show()
        self.hide()

    # Show the automatic mode window
    def show_automatic_mode(self):
        self.automatic_mode_window = MainWindow(mode="automatic")
        self.automatic_mode_window.show()
        self.hide()

# Main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    input_window = ModeSelectionWindow()
    input_window.show()
    sys.exit(app.exec_())