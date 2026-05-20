import paho.mqtt.client as mqtt
import time

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("#")

def on_message(client, userdata, msg):
    time_MQTT= int(time.time()*1000)
    print(f"Time_MQTT:{time_MQTT} | Topic: {msg.topic} | Payload: {msg.payload.decode()}")

client = mqtt.Client(transport="websockets")
client.tls_set(cert_reqs=0)
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 8333, 60)
client.loop_forever()
