import argparse
import os
import time

from dotenv import load_dotenv

from mqtt_client import MQTT_Client, MQTTConfig
from robot_state_buffer import RobotStateBuffer

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))

# ---------------------------------------------------------------------------
# サーバープリセット
#   local / uclab の違いはここで吸収する。クラス内には持ち込まない。
# ---------------------------------------------------------------------------
SERVER_PRESETS = {
    "local": {
        "host":      os.getenv("MQTT_LOCAL_HOST", "192.168.123.51"),
        "port":      int(os.getenv("MQTT_LOCAL_PORT", "9001")),
        "transport": "websockets",
    },
    "uclab": {
        "host":      os.getenv("MQTT_UCLAB_HOST", "sora2.uclab.jp"),
        "port":      int(os.getenv("MQTT_UCLAB_PORT", "1883")),
        "transport": "tcp",
    },
}


def build_config(mode: str) -> MQTTConfig:
    preset = SERVER_PRESETS[mode]
    return MQTTConfig(
        host=preset["host"],
        port=preset["port"],
        transport=preset["transport"],
        robot_uuid=os.getenv("ROBOT_UUID", "2A5PE-YUSHU008"),
        robot_type=os.getenv("ROBOT_TYPE", "Unitree-G1-Dex3"),
        manage_topic=os.getenv("MQTT_MANAGE_TOPIC", "mgr"),
        device_topic=os.getenv("MQTT_DEVICE_TOPIC", "dev"),
        ctrl_topic=os.getenv("MQTT_CTRL_TOPIC", "control"),
        robot_state_topic=os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot"),
    )


def main():
    parser = argparse.ArgumentParser(description="Run MQTT Client in different modes.")
    parser.add_argument(
        '--mode',
        type=str,
        default='local',
        choices=list(SERVER_PRESETS.keys()),
        help="Choose the running mode (default: local)",
    )
    args = parser.parse_args()

    print(f"--- Starting in {args.mode.upper()} Mode ---")

    config = build_config(args.mode)
    buffer = RobotStateBuffer()
    buffer.create()

    client = MQTT_Client(config, buffer)
    client.connect()

    try:
        while True:
            client.publish_robot_state()
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        client.robot_unregister()
        client.disconnect()
        buffer.close()
        print("Client Closed Successfully.")


if __name__ == '__main__':
    main()
