# 共通の定数定義
import os
#
# 以下のものは将来的にこちらに移動するが当面は個別ファイルで定義する
# CA_CERTS_PATH = os.path.join(os.path.dirname(__file__), "certs")
# MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
# MQTT_DEVICE_TOPIC = os.getenv("MQTT_DEVICE_TOPIC", "dev")
# MQTT_LOCAL_SERVER = os.getenv("MQTT_BROKER", "localhost")
# MQTT_LOCAL_PORT = 8333
# MQTT_UCLAB_SERVER = "sora2.uclab.jp"
# MQTT_UCLAB_PORT = 1883
# ROBOT_TYPE = os.getenv("ROBOT_TYPE","piper_right")
# ROBOT_UUID = os.getenv("ROBOT_UUID","MA100101000019005100402")

TIME_OFFSET_PATH = os.getenv("TIME_OFFSET_PATH", f"/run/user/{os.getuid()}/time_offset.txt")
# rootの場合/run/user/0は無いがコンテナ内なので気にしないこととする
# composeで起動する場合 TIME_OFFSET_PATH=/mnt/share/time_offset.txt のように指定する
if not os.path.exists(os.path.dirname(TIME_OFFSET_PATH)):
    os.makedirs(os.path.dirname(TIME_OFFSET_PATH), exist_ok=True)

