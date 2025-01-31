# BTGenBot

This work presents a novel approach to generating
behavior trees for robots using lightweight large language models
(LLMs) with a maximum of 7 billion parameters. The study
demonstrates that it is possible to achieve satisfying results with
compact LLMs when fine-tuned on a specific dataset. The key
contributions of this research include the creation of a finetuning
dataset based on existing behavior trees using GPT-3.5
and a comprehensive comparison of multiple LLMs (namely
llama2, llama-chat, and code-llama) across nine distinct tasks.
To be thorough, we evaluated the generated behavior trees
using static syntactical analysis, a validation system, a simulated
environment, and a real robot. Furthermore, this work opens
the possibility of deploying such solutions directly on the robot,
enhancing its practical applicability. Findings from this study
demonstrate the potential of LLMs with a limited number of
parameters in generating effective and efficient robot behaviors.

Release for the paper **BTGenBot: Behavior Tree Generation for Robotic
Tasks with Lightweight LLMs**, published in **2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)**.

Preprint available on arxiv: https://arxiv.org/abs/2403.12761  
Paper: https://ieeexplore.ieee.org/document/10802304  

[`Dataset`](https://huggingface.co/datasets/AIRLab-POLIMI/btgenbot), [`llama-2-7b-chat`](https://huggingface.co/AIRLab-POLIMI/llama-2-7b-chat-hf-btgenbot-adapter) and [`codellama-7b-instruct`](https://huggingface.co/AIRLab-POLIMI/codellama-7b-instruct-hf-btgenbot-adapter) LoRA adapters available on HuggingFace.

**Authors**: [Riccardo Andrea Izzo](mailto:riccardo.izzo@mail.polimi.it), [Gianluca Bardaro](mailto:gianluca.bardaro@polimi.it) and [Matteo Matteucci](mailto:matteo.matteucci@polimi.it)  
**Location**: [**AIRLab** (The Artificial Intelligence and Robotics Lab of Politecnico di Milano)](https://airlab.deib.polimi.it/)

## Overview
- `bt_client`: client designed to interpret and execute LLM-generated behavior trees directly on robot
- `bt_generator`: demo notebook to load our fine-tuned models for generating behavior trees
- `bt_validator`: validator that assess the overall correctness of the LLM-generated trees
- `dataset`: our instruction-following dataset used to fine-tune the models
- `lora_adapters`: LoRA adapters for the base models, used in the notebook to load the fine-tuned version
- `prompt`: outcomes of prompts run on both LlamaChat and CodeLlama models, both in zero-shot and one-shot scenarios

## Setup
### bt_generator

Create a conda environment (or equivalent virtualenv):
```
conda create -n btgenbot python==3.10
```

Install dependencies:
```
pip install -r requirements.txt
```

### bt_client/bt_validator

Create a colcon workspace and clone this repository in your ROS2 workspace  
Build:
```
colcon build
```

Required ROS2 dependencies:
- `BehaviorTree.CPP`: available [here](https://github.com/BehaviorTree/BehaviorTree.CPP)
- `BehaviorTree.ROS2`: available [here](https://github.com/BehaviorTree/BehaviorTree.ROS2)
- `igus_rebel_commander`: available [here](https://github.com/AIRLab-POLIMI/ros2-igus-rebel), required only by `bt_client` for the task involving arucos and arm activity
- `aruco_interfaces`: available [here](https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation/tree/main/aruco_interfaces), required only by `bt_client` for the task involving arucos and arm activity

Tested on a Linux computer with Ubuntu 22.04 and ROS2 Humble

## bt_client
### Client usage
- Select the task and the corresponding behavior tree in XML format from the ones available in `/bt_xml`
- Modify `config/tree.yaml` configuration file with the file name in the `tree_name` field:
```
tree_name: "main_tree.xml"
```

To add a new behavior tree, follow these steps:  
1. Create an XML file representing the behavior tree  
2. Place the XML file in the `/bt_xml` folder
3. Specify the name of the XML file in the `config/tree.yaml` configuration file

- Build and source the package
```
colcon build
source install/setup.bash
```
- Launch the client and execute the selected behavior tree
```
ros2 launch bt_client bt.launch.py
```

### Node functionalities
Keep in mind that the system offers a range of pre-defined node functionalities. For instance, the "MoveTo" action facilitates the sending of a navigation goal to the Nav2 server, utilizing the goal specified within the behavior tree XML.

Moreover, locations for testing purposes are outlined in the location.yaml configuration file. These locations are pre-defined and serve as references for navigation tasks.

It is possible to add further actions with
```
factory.registerNodeType<ACTION_NAME>("ACTION_NAME");
```

### Full pipeline usage (LLM -> Robot)
- Launch the pipeline
```
ros2 launch bt_client monitor.launch.py
```

This command initiates the pipeline. Once the behavior tree specified in the `config/tree.yaml` configuration file becomes available, the client will automatically execute it.  
This behavior tree is intended to be the one generated by the LLM, for example with `inference.ipynb` or `btgenbot.py`.

## bt_generator
### inference.ipynb usage
Explore a demonstration notebook showcasing the generation of behavior trees utilizing llamachat and codellama, featuring both zero-shot and one-shot prompts.

### btgenbot.py usage
Client application with GUI that generates a behavior tree given a new task description. After generating the behavior tree, the application saves it to a file and initiates its transmission to the remote location of a robot for immediate execution.

Two modes are available:
- `Standard Mode`: this mode requires a comprehensive one-shot example that includes a description and its corresponding behavior tree, in addition to the new task description. Recommended for new or specialized tasks.
- `Automatic Retrieval Mode`: in this mode, only the new task description is required. Users can select the task domain to automatically infer a one-shot example from a list of predefined ones in a YAML file. Ideal for straightforward tasks similar to those demonstrated in the examples.

Steps:
- Update the `config/params.yaml` configuration file with SSH connection parameters and your HuggingFace access token. Define the `local_dir` parameter to specify the local directory where XML behavior trees will be saved, and set `remote_dir` to indicate the corresponding remote location on the robot where the trees should be stored.
- Run the script
```
python3 btgenbot.py
```

## bt_validator
### Usage
- Build and source the package
```
colcon build
source install/setup.bash
```
- Launch the validator from the root directory
```
./build/bt_validator/main
```

## Citation
If you use this work in your research, please consider citing our paper:

```
@INPROCEEDINGS{10802304,
  author={Izzo, Riccardo Andrea and Bardaro, Gianluca and Matteucci, Matteo},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={BTGenBot: Behavior Tree Generation for Robotic Tasks with Lightweight LLMs}, 
  year={2024},
  volume={},
  number={},
  pages={9684-9690},
  keywords={Accuracy;Service robots;Large language models;Semantics;Natural languages;XML;Syntactics;Robots;Intelligent robots;Logistics},
  doi={10.1109/IROS58592.2024.10802304}}
```
