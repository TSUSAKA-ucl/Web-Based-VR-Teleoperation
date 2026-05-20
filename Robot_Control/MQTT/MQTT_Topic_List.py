import argparse
import os
import ssl
import paho.mqtt.client as mqtt
import time

parser = argparse.ArgumentParser(
    description="WSS経由でMQTTブローカーにタイムスタンプを送信するスクリプト"
)
parser.add_argument(
    "-H",
    "--host",
    type=str,
    default="192.168.207.175",
    help="MQTTブローカーのホスト名 (IPまたはFQDN) [デフォルト: 192.168.207.175]",
)
parser.add_argument(
    "-p",
    "--port",
    type=int,
    default=8333,
    help="MQTTブローカーのポート番号 [デフォルト: 8333]",
)
args = parser.parse_args()
home_dir = os.path.expanduser("~")
ca_certs_path = os.path.join(home_dir, ".local/share/mkcert/rootCA.pem")


def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe("#")

def on_message(client, userdata, msg):
    time_MQTT= int(time.time()*1000)
    print(f"Time_MQTT:{time_MQTT} | Topic: {msg.topic} | Payload: {msg.payload.decode()}")

client = mqtt.Client(transport="websockets")
# client.tls_set(cert_reqs=0)
client.tls_set(
    ca_certs=ca_certs_path,
    certfile=None,  # クライアント証明書を使わない場合は None
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED,  # サーバー証明書の検証を必須にする
    tls_version=ssl.PROTOCOL_TLSv1_2,  # または ssl.PROTOCOL_TLS
)
client.on_connect = on_connect
client.on_message = on_message

client.connect(args.host, args.port, 60)
client.loop_forever()
