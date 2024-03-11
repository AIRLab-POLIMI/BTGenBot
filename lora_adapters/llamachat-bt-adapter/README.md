---
library_name: peft
base_model: meta-llama/Llama-2-7b-chat-hf
---

# Model Card for Model ID

<!-- Provide a quick summary of what the model is/does. -->



## Model Details

### Model Description

<!-- Provide a longer summary of what this model is. -->



- **Developed by:** Riccardo Andrea Izzo
- **Model type:** Transformer-based language model
- **Language(s) (NLP):** English
- **Finetuned from model [optional]:** Llama-2-7b-chat-hf

### Model Sources [optional]

<!-- Provide the basic links for the model. -->

- **Repository:** meta-llama/Llama-2-7b-chat-hf

## Uses

Behavior trees generation for robotic tasks

## Environmental Impact

<!-- Total emissions (in grams of CO2eq) and additional considerations, such as electricity usage, go here. Edit the suggested text below accordingly -->

Carbon emissions can be estimated using the [Machine Learning Impact calculator](https://mlco2.github.io/impact#compute) presented in [Lacoste et al. (2019)](https://arxiv.org/abs/1910.09700).

- **Hardware Type:** 2x NVIDIA Quadro RTX 6000
- **Hours used:** 36h


## Training procedure


The following `bitsandbytes` quantization config was used during training:
- quant_method: bitsandbytes
- load_in_8bit: True
- load_in_4bit: False
- llm_int8_threshold: 6.0
- llm_int8_skip_modules: None
- llm_int8_enable_fp32_cpu_offload: False
- llm_int8_has_fp16_weight: False
- bnb_4bit_quant_type: fp4
- bnb_4bit_use_double_quant: False
- bnb_4bit_compute_dtype: float32

### Framework versions


- PEFT 0.6.0
