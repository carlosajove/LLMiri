# pip install accelerate
from huggingface_hub import login 
login()

from transformers import AutoTokenizer, AutoModelForCausalLM
import torch


access_token = "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"
tokenizer = AutoTokenizer.from_pretrained("google/gemma-2-9b", token=access_token)
model = AutoModelForCausalLM.from_pretrained(
    "google/gemma-2-9b",
    device_map="auto",
    torch_dtype=torch.bfloat16
)

input_text = "Write me a poem about Machine Learning."
input_ids = tokenizer(input_text, return_tensors="pt").to("cuda")

outputs = model.generate(**input_ids, max_new_tokens=1000)
print(tokenizer.decode(outputs[0]))




