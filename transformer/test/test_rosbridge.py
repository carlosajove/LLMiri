import websocket
import json
import time
import threading

def on_open(ws):
    print("Connection opened")
    
    # Advertise the topic
    advertise_message = {
        "op": "advertise",
        "topic": "/transformer/LLM_interface/output",
        "type": "std_msgs/String"
    }
    print("Advertising topic:", advertise_message)
    ws.send(json.dumps(advertise_message))

    # Function to publish messages
    def run(*args):
        for i in range(100):
            message = {
                "op": "publish",
                "topic": "/transformer/LLM_interface/output",
                "msg": {"data": f"Hello ROS {i}"}
            }
            print("Sending message:", message)
            ws.send(json.dumps(message))
            time.sleep(1)
        ws.close()

    # Start a new thread to run the message publishing function
    threading.Thread(target=run).start()

def on_error(ws, error):
    print("Error: ", error)

def on_close(ws, close_status_code, close_msg):
    print("Connection closed with code: ", close_status_code, " and message: ", close_msg)

if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://localhost:9090/",
                                on_open=on_open,
                                on_error=on_error,
                                on_close=on_close)
    print('heyhey')
    try:
        ws.run_forever()
    except Exception as e:
        print("Exception occurred: ", e)