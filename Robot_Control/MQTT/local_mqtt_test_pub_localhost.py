import argparse
import json
import os
import ssl
import time
import paho.mqtt.client as mqtt

# 1. コマンドライン引数の設定
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

# 2. $HOME ディレクトリを動的に取得してCA証明書のパスを作成
home_dir = os.path.expanduser("~")
ca_certs_path = os.path.join(home_dir, ".local/share/mkcert/rootCA.pem")

# MQTTクライアントの初期化
client = mqtt.Client(transport="websockets")

# TLSの設定
# client.tls_set(cert_reqs=0)
client.tls_set(
    ca_certs=ca_certs_path,
    certfile=None,  # クライアント証明書を使わない場合は None
    keyfile=None,
    cert_reqs=ssl.CERT_REQUIRED,  # サーバー証明書の検証を必須にする
    tls_version=ssl.PROTOCOL_TLSv1_2,  # または ssl.PROTOCOL_TLS
)
client.tls_insecure_set(False)

# 引数から取得したホストとポートで接続
print(f"🔄 Connecting to {args.host}:{args.port}...")
client.connect(args.host, args.port, 60)

# メインループの開始（バックグラウンドでのネットワーク処理を維持するため推奨）
client.loop_start()

topic = "test/timestamp"

try:
    while True:
        timestamp = int(time.time() * 1000)
        payload = json.dumps({"timestamp": timestamp})
        client.publish(topic, payload)
        print(f"📤 Sent: {payload}")
        time.sleep(1)  # 毎秒送信
except KeyboardInterrupt:
    print("\n🛑 終了要求を受け取りました。切断します。")
finally:
    client.loop_stop()
    client.disconnect()
