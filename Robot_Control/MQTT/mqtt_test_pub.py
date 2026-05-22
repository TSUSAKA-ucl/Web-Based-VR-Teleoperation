import argparse
import mqtt_common_opt
import json
import time
import paho.mqtt.client as mqtt


parser = argparse.ArgumentParser(
    description="WSS経由でMQTTブローカーにタイムスタンプを送信するスクリプト"
)
parser = mqtt_common_opt.add_common_opts(parser)
args = parser.parse_args()

# MQTTクライアントの初期化
client = mqtt.Client(transport="websockets")

# TLSの設定
mqtt_common_opt.configure_tls(client)

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
