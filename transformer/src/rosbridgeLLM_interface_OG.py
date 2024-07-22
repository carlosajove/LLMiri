import websocket
import json


from callLLM import PhiMini


def on_open(ws):
    print("Advertising service:")

    advertise_service = {
        "op": "advertise_service",
        "type": "transformer/CallLLM",
        "service": "transformer/phi-mini/callStiffness"
    }
    ws.send(json.dumps(advertise_service))
    print("Advertising topic:")

    advertise_topic =   {
        "op": "advertise",
        "topic": "/transformer/phi-mini/outputStiffness",
        "type": "std_msgs/String"
    }
    ws.send(json.dumps(advertise_topic))
    
    
def publish(ws, msg):
    message = {
        "op": "publish",
        "topic": "/transformer/phi-mini/outputStiffness",
        "msg": {"data": msg}
    }
    ws.send(json.dumps(message))

def on_message(ws, message):
    data = json.loads(message)
    if data['op'] == 'call_service':
        service_name = data['service']
        args = data['args']
        input = args['input']
        
        print("on message")
        result = callPhiMini(input)
        
        response = {
            "op": "service_response",
            "service": service_name,
            "values": {
                "output": result, "success": True
            },
            "result": True
        }

        #return the service 
        ws.send(json.dumps(response))  #not working, so instead publishing
        
        # Publish to ROS topic
        publish(ws, result)

def on_error(ws, error):
    print("ERROR: LLM interface service", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection LLM interface service closed with code: ", close_status_code, " and message: ", close_msg)



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