def main():
    from transformers import AutoModelForCausalLM, AutoTokenizer
    from peft import PeftModel
    import torch, accelerate
    from pathlib import Path
    from paramiko import SSHClient
    from scp import SCPClient
    import yaml
    import re
    
    # Define the path to the YAML configuration file
    config_file_path = 'bt_generator/config/params.yaml'

    # Load parameters from the YAML configuration file
    with open(config_file_path, 'r') as file:
        params = yaml.safe_load(file)
        
    model_id = params.get("model_id")
    adapter_id = params.get("adapter_id")

    # Load tokenizer
    tokenizer = AutoTokenizer.from_pretrained(
        pretrained_model_name_or_path = model_id,
    )

    # Load base model
    base_model = AutoModelForCausalLM.from_pretrained(
        pretrained_model_name_or_path = model_id,
        torch_dtype = torch.float16,
        device_map = "auto",
        trust_remote_code = True,
    )
        
    context = params.get("context")
    task = params.get("task")
    example_task = params.get("example_task")
    example_output = params.get("example_output")
    
    eval_prompt = "<s>[INST]" + context + example_task + "[/INST]</s>" + example_output + "[INST]" + task + "[/INST]"
    model_input = tokenizer(eval_prompt, return_tensors="pt").to("cuda")
    
    # Load fine-tuned model
    finetuned_model = PeftModel.from_pretrained(base_model, adapter_id)
    finetuned_model = finetuned_model.merge_and_unload()
    
    # Generate behaviour tree
    finetuned_model.eval()
    with torch.no_grad():
        result = tokenizer.decode(finetuned_model.generate(**model_input, max_new_tokens=1000)[0], skip_special_tokens=True)
        

    host = params.get("host")
    username = params.get("username")
    password = params.get("password")
    local_dir = params.get("local_dir")
    remote_dir = params.get("remote_dir")
    
    # Define regular expression pattern to extract the behavior tree
    pattern = r'<root .*?</root>'
    matches = re.findall(pattern, result, re.DOTALL)

    root_dir = Path(__file__).parent / "../bt_client/bt_xml"
    tree_xml = params.get("tree_name")

    if matches:
        print("BT found, sending to remote location...")
        final_tree = matches[-1]
        
        with open(root_dir / tree_xml, "w") as xml_file:
            xml_file.write(final_tree)
        
        # Open SSH connection
        ssh = SSHClient()
        ssh.load_system_host_keys()
        ssh.connect(host, username=username)
        
        scp = SCPClient(ssh.get_transport())
        # Send the file to the remote location
        scp.put(files=local_dir + tree_xml, remote_path=remote_dir)
        print("Success. Closing ssh connection.")
        scp.close()
        ssh.close()
        
    else:
        print("BT not found.")
        
if __name__ == "__main__":
    main()
