import os
import json
import ssl
import time
from datetime import datetime, timezone
import threading
import numpy as np
from paho.mqtt import client as mqtt
from . import mqtt_common_opt
from .edge_state import EdgeState

# CA_CERTS_PATH = os.path.join(os.path.dirname(__file__), "certs")
# 自分のhomeの .local/share/mkcert/にあるrootCA.pemを使わずに
# 環境変数からパスを取得し無ければこのスクリプトのディレクトリにあるrootCA.pem
# を使用する
# CA_CERTS_PATH = os.path.expanduser("~/.local/share/mkcert/rootCA.pem")
CA_CERTS_PATH = os.getenv(
    "CA_CERTS_PATH", os.path.join(os.path.dirname(__file__), "rootCA.pem")
)
print("CA_CERTS_PATH:", CA_CERTS_PATH)
# For register
# import os
# from dotenv import load_dotenv
# load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
# MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
# MQTT_DEVICE_TOPIC = os.getenv("MQTT_DEVICE_TOPIC", "dev")


MQTT_LOCAL_SERVER = os.getenv("MQTT_BROKER", "localhost")
MQTT_LOCAL_PORT = 8333
MQTT_UCLAB_SERVER = "sora2.uclab.jp"
MQTT_UCLAB_PORT = 1883

# mqtt clientのconnectは connect(args.host, args.port, 60)とする
# argsはコンストラクター引数で渡す
# argsの初期値はMQTT_LOCAL_SERVERとMQTT_LOCAL_PORTで作る
default_args = {
    "host": MQTT_LOCAL_SERVER,
    "port": MQTT_LOCAL_PORT,
    "tls": False,
    "robot_type": os.getenv("ROBOT_TYPE", "piper_right"),
    "robot_uuid": os.getenv("ROBOT_UUID", "MA100101000019005100402"),
}
# ROBOT_TYPE = os.getenv("ROBOT_TYPE","piper_right")
# ROBOT_UUID = os.getenv("ROBOT_UUID","MA100101000019005100402")
# ROBOT_TYPE = os.getenv("ROBOT_TYPE","piper_left")
# ROBOT_UUID = os.getenv("ROBOT_UUID","MA100101000019005100010")


# 現在 control/right/joint, control/right/toolのように3rd levelの名前が
# ベタ書きされているがthird_level_listのように一般化して管理するほうが良い
# PiPERは1DOF toolで self.pose[15]に値をいれるだけだが、Unitreeハンド等は3+2+2軸あり
# この方法で行くなら control/right/thumb,control/right/index,control/right/middle
# のようにすべきだろう

# paho.mqttのmulti threadに対応するため self.lockを定義する
# self.poseが本クラスの外側からのアクセス用変数なのでlockする
# 8:15などは名前をつけたスライスにする
VIRTUTAL_JOINT_SLICE = slice(8, 15)
VIRTUTAL_TOOL_INDEX = 15


class MQTT_Client:
    MQTT_DEVICE_TOPIC_HDR = "dev"
    MGR_REGISTER_TOPIC = "mgr/register"
    MGR_UNREGISTER_TOPIC = "mgr/unregister"

    def __init__(self, arm, mode="local", args=default_args):
        self.args = args
        self.joint_topic = arm + 'joint/'
        self.tool_topic = arm + 'tool/'
        self.mode = mode

        self.ROBOT_TYPE = self.args.get("robot_type", "piper_right")
        self.ROBOT_UUID = self.args.get("robot_uuid", "MA1001010000190050100402")
        self.DEV_ACQUIRE_TOPIC = f"{self.MQTT_DEVICE_TOPIC_HDR}/{self.ROBOT_UUID}"
        # self.DEV_RECV_TOPIC = f"{self.MQTT_DEVICE_TOPIC_HDR}/+"
        self.USER_UUID = None

        self.time_vr_robot_offset = 0

        # self.MQTT_ROBOT_STATE_TOPIC = "robot/" + robot_uuid
        # MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")
        #        f"{MQTT_ROBOT_STATE_TOPIC}/{self.ROBOT_UUID}",
        self.MQTT_ROBOT_STATE_TOPIC = f"robot/{self.ROBOT_UUID}"

        self.lock = threading.Lock()
        self.state = EdgeState.INIT
        self.pose = np.zeros(16)

        self.subscription_list = []

        self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                transport="websockets")
        context = ssl.create_default_context()
        context.load_verify_locations(CA_CERTS_PATH)
        self.client.tls_set_context(context)

    def start_mqtt(self):
        with self.lock:
            self.state = EdgeState.STARTING
        print("MQTT Client starting...")
        # mqtt_common_opt.configure_tls(self.client)
        print(
            "MQTT Client connecting to:",
            self.args.get("host", MQTT_LOCAL_SERVER), ":",
            self.args.get("port", MQTT_LOCAL_PORT),
        )
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.connect(self.args.get("host", MQTT_LOCAL_SERVER),
                            self.args.get("port", MQTT_LOCAL_PORT), 60)
        self.client.loop_start()

    def subscribe_all_on_list(self):
        for topic in self.subscription_list:
            self.client.subscribe(topic)
            print("Subscribed to topic:", topic)

    def unsubscribe_all_on_list(self):
        for topic in self.subscription_list:
            self.client.unsubscribe(topic)
            print("Unsubscribed from topic:", topic)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        print("###### enter on_connect")
        with self.lock:
            self.state = EdgeState.REGISTRATION
        if reason_code != 0:
            print("Failed to connect, reason code:", reason_code)
            return
        # For register
        my_info = {
            "date": datetime.now(timezone.utc).isoformat(),
            "devType": "robot",
            "type": self.ROBOT_TYPE,
            "version": "0.1.1",
            "devId": self.ROBOT_UUID
        }
        self.client.publish(self.MGR_REGISTER_TOPIC, json.dumps(my_info), qos=1)
        print("Publish robot registeration:", json.dumps(my_info))
        self.client.message_callback_add(self.DEV_ACQUIRE_TOPIC, self.on_acquire_topic)
        self.client.subscribe(self.DEV_ACQUIRE_TOPIC)
        self.subscribe_all_on_list()
        # 現在はregistration完了のkeyを受け取る処理はないので、ここでstateをREGISTEREDにする
        with self.lock:
            self.state = EdgeState.REGISTERED

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    def on_acquire_topic(self, client, userdata, msg):
        try:
            js_msg = json.loads(msg.payload.decode())
            if (js_msg.get("devId", None) != None
                and js_msg.get("controller", None) == "browser"):
                from_dev_id = js_msg.get("devId", None)
            else:
                return

            print(f"Received message from {from_dev_id} on topic {msg.topic}")
            if from_dev_id and from_dev_id != self.USER_UUID:
                print(
                    f"------------------ {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} -------------------"
                )
                print(f"## Capture New Control Request: {from_dev_id}")
                # If old subscribe exists, unsubscribe
                self.unsubscribe_all_on_list()
                self.USER_UUID = from_dev_id
                # Update topics with new USER_UUID
                self.MQTT_CTRL_JOINT_TOPIC = (
                    f"control/{self.joint_topic}{self.USER_UUID}"
                )
                self.MQTT_CTRL_TOOL_TOPIC = f"control/{self.tool_topic}{self.USER_UUID}"
                self.subscription_list = [
                    self.MQTT_CTRL_JOINT_TOPIC,
                    self.MQTT_CTRL_TOOL_TOPIC,
                ]
                self.subscribe_all_on_list()
            with self.lock:
                self.state = EdgeState.ACQUIRED
        except Exception as e:
            print(f"XXX Subscribe {self.DEV_ACQUIRE_TOPIC} failed: {e}")

    def on_message(self, client, userdata, msg):
        if msg.topic == self.MQTT_CTRL_JOINT_TOPIC:
            # Message ThetaBody
            js_msg = json.loads(msg.payload)
            joints = js_msg["joint"]
            thetaBody = [joints[i] if i < len(joints) else 0 for i in range(7)]
            with self.lock:
                self.pose[VIRTUTAL_JOINT_SLICE] = thetaBody

        elif msg.topic == self.MQTT_CTRL_TOOL_TOPIC:
            # Message ThetaTool
            js_msg = json.loads(msg.payload)
            js_tool = js_msg["tool"]
            thetaTool = js_tool
            with self.lock:
                self.pose[VIRTUTAL_TOOL_INDEX] = thetaTool

        else:
            print("not subscribe msg", msg.topic)

    def publish_message(self, payload_dict):
        self.client.publish(self.MQTT_ROBOT_STATE_TOPIC, json.dumps(payload_dict))
        # if self.USER_UUID:
        #     self.MQTT_ROBOT_STATE_TOPIC = f"robot/{self.USER_UUID}"
        #     self.client.publish(self.MQTT_ROBOT_STATE_TOPIC, json.dumps(payload_dict))

    def unregister_message(self):
        unregister_payload = {
            "date": datetime.now(timezone.utc).isoformat(),
            "devType": "robot",
            "type": self.ROBOT_TYPE,
            "version": "0.1.1",
            "devId": self.ROBOT_UUID,
            "status": "unregister",
        }
        self.client.publish(
            self.MGR_UNREGISTER_TOPIC, json.dumps(unregister_payload), qos=1
        )
        print("Publish robot unregistration:", json.dumps(unregister_payload))

    def set_time_offset(self, time_offset):
        self.time_vr_robot_offset = time_offset
