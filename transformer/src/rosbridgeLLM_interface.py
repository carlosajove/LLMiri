import websocket
import json
import time
import threading


import torch 
from transformers import AutoModelForCausalLM, AutoTokenizer, pipeline 


def on_open(ws):
    advertise_service = {
        "op": "advertise_service",
        "type": "transformer/CallLLM",
        "service": "transformer/LLM_interface"
    }
    ws.send(json.dumps(advertise_service))
    
def on_error(ws, error):
    print("ERROR: LLM interface service", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection LLM interface service closed with code: ", close_status_code, " and message: ", close_msg)


def on_message(ws, message):
    data = json.loads(message)
    if data['op'] == 'call_service':
        service_name = data['service']
        args = data['args']
        
        
        callLLM()
        
        response = {
            "op": "service_response",
            "service": service_name,
            "values": {
                "output": 'a', "success": True
            },
            "result": True
        }
        ws.send(json.dumps(response))

        
def callLLM():
    torch.random.manual_seed(0) 
    model = AutoModelForCausalLM.from_pretrained( 
        "microsoft/Phi-3-mini-128k-instruct",  
        device_map="cuda",  
        torch_dtype="auto",  
        trust_remote_code=True, 
        #attn_implementation="flash_attention_2" Need a newer GPU
    ) 

    tokenizer = AutoTokenizer.from_pretrained("microsoft/Phi-3-mini-128k-instruct") 

    messages = [ 
        {"role": "system", "content": "You are a helpful AI assistant."}, 
        {"role": "user", "content": "Can you provide ways to eat combinations of bananas and dragonfruits?"}, 
        {"role": "assistant", "content": "Sure! Here are some ways to eat bananas and dragonfruits together: 1. Banana and dragonfruit smoothie: Blend bananas and dragonfruits together with some milk and honey. 2. Banana and dragonfruit salad: Mix sliced bananas and dragonfruits together with some lemon juice and honey."}, 
        {"role": "user", "content": "What about solving an 2x + 3 = 7 equation?"}, 
    ] 

    pipe = pipeline( 
        "text-generation", 
        model=model, 
        tokenizer=tokenizer, 
    ) 

    generation_args = { 
        "max_new_tokens": 500, 
        "return_full_text": False, 
        "temperature": 0.01, 
        "do_sample": True,  #True
    } 

    output = pipe(messages, **generation_args) 
    result = json.dumps({'messages': messages, "ouput": output[0]['generated_text']})
    return result
    
    
if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://localhost:9090/",
                                on_open=on_open,
                                on_error=on_error,
                                on_message=on_message,
                                on_close=on_close)
    
    try:
        ws.run_forever()
    except Exception as e:
        print("Exception occurred: ", e)