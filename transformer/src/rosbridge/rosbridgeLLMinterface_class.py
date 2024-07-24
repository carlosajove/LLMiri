import websocket
import json
from typing import Callable
import threading


class rosbridgeLLMinterface():
    def __init__(self, function: Callable, type: str, service_name: str, topic_name: str):
        """Class that creates custom service/topic (of type CallLLM) pair for bridging LLM with ros

        Args:
            function (Callable): point to LLM call function #TODO: point to LLM class
            type (str): service type: example transformer/CallLLM
            service_name (str): service name must be $model/$service, for example "phi_mini/callStiffness"
            topic_name (string): topic name bust be $model/$topic, for example "phi_mini/outputStiffness"
        """
        self._function = function
        self._service_name = service_name
        self._topic_name = topic_name
        self._type = type
        
        websocket.enableTrace(True)
        self._ws = websocket.WebSocketApp("ws://localhost:9090/",
                                            on_open=self.on_open,
                                            on_error=self.on_error,
                                            on_message=self.on_message,
                                            on_close=self.on_close)
        #self.lock = threading.Lock()
        
    def run_forever(self):
            """Starts the WebSocket connection and runs it indefinitely."""
            try:
                self._ws.run_forever()
            except websocket.WebSocketConnectionClosedException as e:
                print(f"WebSocket connection closed: {e}")
            except websocket.WebSocketException as e:
                print(f"WebSocket exception occurred: {e}")
            except Exception as e:
                print(f"Exception occurred: {e}")
            
        
    def on_open(self, ws):
        #with self.lock:
            print("Advertising service:")

            advertise_service = {
                "op": "advertise_service",
                "type": self._type,
                "service": "transformer/"+self._service_name
            }
            ws.send(json.dumps(advertise_service))
            print("Advertising topic:")

            advertise_topic =   {
                "op": "advertise",
                "topic": "/transformer/"+self._topic_name,
                "type": "std_msgs/String"
            }
            ws.send(json.dumps(advertise_topic))
        
        
    def publish(self, ws, msg):
        #with self.lock:       
            message = {
                "op": "publish",
                "topic": "/transformer/" + self._topic_name,
                "msg": {"data": msg}
            }
            ws.send(json.dumps(message))

    def on_message(self, ws, message):
        #with self.lock:
            print("[INFO] ON_MESSAGE rosbridge")
            data = json.loads(message)
            if data['op'] == 'call_service':
                service_name = data['service']
                args = data['args']
                input = args['input']
                
                print("on message")
                result = self._function(input)
                
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
                self.publish(ws, result)

    def on_error(self, ws, error):
        print("ERROR: LLM interface service", error)

    def on_close(self, ws, close_status_code, close_msg):
        print("Connection LLM interface service closed with code: ", close_status_code, " and message: ", close_msg)
