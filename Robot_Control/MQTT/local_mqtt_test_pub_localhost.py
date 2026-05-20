import ssl
import paho.mqtt.client as mqtt
import time
import json

client = mqtt.Client(transport="websockets")
# client.tls_set(cert_reqs=0)
ca_certs_path = "/home/tsusaka/.local/share/mkcert/rootCA.pem"
client.tls_set(
    ca_certs=ca_certs_path,
    certfile=None, # クライアント証明書を使わない場合は None
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED, # サーバー証明書の検証を必須にする
    tls_version=ssl.PROTOCOL_TLSv1_2 # または ssl.PROTOCOL_TLS
)
client.tls_insecure_set(False)

# client.connect("localhost", 8333, 60)
client.connect("uclab-deskmeet-1.ucl.nuee.nagoya-u.ac.jp", 8333, 60)

topic = "test/timestamp"

while True:
    timestamp = int(time.time() * 1000)
    payload = json.dumps({"timestamp": timestamp})
    client.publish(topic, payload)
    print(f"📤 Sent: {payload}")
    time.sleep(1)  # 每秒发一次，可改成更短
