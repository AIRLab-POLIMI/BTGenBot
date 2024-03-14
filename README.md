# BTGenBot

**Abstract** This paper presents a novel approach to generating
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
Tasks with Lightweight LLMs**, currently in submission at **IEEE/RSJ International Conference on Intelligent Robots and Systems**.

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

Tested on a Linux computer with Ubuntu 22.04 and ROS2 Humble