import argparse
import mqtt_common_opt
import paho.mqtt.client as mqtt
import time


def on_connect(client, userdata, flags, reason_code, properties):
    print("Connected with result code", reason_code)
    client.subscribe("#")

def on_message(client, userdata, msg):
    time_MQTT= int(time.time()*1000)
    print(f"Time_MQTT:{time_MQTT} | Topic: {msg.topic} | Payload: {msg.payload.decode()}")

parser = argparse.ArgumentParser(
    description="WSS経由でMQTTブローカーからtopicのリストを得る"
)
parser = mqtt_common_opt.add_common_opts(parser)
args = parser.parse_args()

client = mqtt.Client(
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    transport="websockets")
mqtt_common_opt.configure_tls(client)

client.on_connect = on_connect
client.on_message = on_message

client.connect(args.host, args.port, 60)
client.loop_forever()
