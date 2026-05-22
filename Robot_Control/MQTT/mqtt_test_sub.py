import argparse
import mqtt_common_opt
import paho.mqtt.client as mqtt
import time
import json


def on_connect(client, userdata, flags, rc):
    print("✅ Connected with result code", rc)
    client.subscribe("test/timestamp")  # 可以改为 "#" 订阅所有Topic


def on_message(client, userdata, msg):
    recv_time = int(time.time() * 1000)
    try:
        data = json.loads(msg.payload.decode())
        send_time = data.get("timestamp", 0)
        latency = recv_time - send_time
        print(f"\n📩 Topic: {msg.topic}")
        print(f"Send: {send_time} | Recv: {recv_time} | Latency: {latency} ms")
    except Exception as e:
        print("Decode error:", e)
        print("Payload:", msg.payload)


parser = argparse.ArgumentParser(
    description="WSS経由でMQTTブローカーからタイムスタンプを受信するスクリプト"
)
parser = mqtt_common_opt.add_common_opts(parser)
args = parser.parse_args()

client = mqtt.Client(transport="websockets")
mqtt_common_opt.configure_tls(client)
client.on_connect = on_connect
client.on_message = on_message

print(f"🔄 Connecting to {args.host}:{args.port}...")
client.connect(args.host, args.port, 60)
client.loop_forever()
