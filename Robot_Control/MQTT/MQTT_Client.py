import os
import json
import time
import multiprocessing.shared_memory as sm
from datetime import datetime
import numpy as np
from paho.mqtt import client as mqtt
from . import mqtt_common_opt

# For register
# import os
# from dotenv import load_dotenv
# load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
# MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
# MQTT_DEVICE_TOPIC = os.getenv("MQTT_DEVICE_TOPIC", "dev")


MQTT_LOCAL_SERVER = "localhost"
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



class MQTT_Client():
    MQTT_DEVICE_TOPIC_HDR = "dev"
    MGR_REGISTER_TOPIC = "mgr/register"
    def __init__(self, arm, mode="local", args=default_args):
        self.args = args
        self.joint_topic = arm + 'joint/'
        self.tool_topic = arm + 'tool/'
        self.mode = mode

        self.ROBOT_TYPE = self.args.get("robot_type", "piper_right")
        self.ROBOT_UUID = self.args.get("robot_uuid", "MA1001010000190050100402")
        # self.MQTT_RECV_TOPIC = f"{self.MQTT_DEVICE_TOPIC_HDR}/{self.ROBOT_UUID}"
        self.MQTT_RECV_TOPIC = f"{self.MQTT_DEVICE_TOPIC_HDR}/+"
        self.USER_UUID = None

        self.time_vr_robot_offset = 0
        self.ping = 0
        self.current_time = 0

        self.input_count = 0

        # self.MQTT_ROBOT_STATE_TOPIC = "robot/" + robot_uuid
        # MQTT_ROBOT_STATE_TOPIC = os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot")
        #        f"{MQTT_ROBOT_STATE_TOPIC}/{self.ROBOT_UUID}",
        self.MQTT_ROBOT_STATE_TOPIC = f"robot/{self.ROBOT_UUID}"

        self.time_vr_pub = 0

        self.pose = np.zeros(16)

        self.shared_signal = 0 # shared_control_signal
        self.shared_control_flag = 0
        self.subscription_list = []
        
        self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                transport="websockets")


    def start_mqtt(self):
        mqtt_common_opt.configure_tls(self.client)
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
        if reason_code != 0:
            print("Failed to connect, reason code:", reason_code)
            return
        # For register
        my_info = {
            "date": datetime.now().strftime('%c'),
            "devType": "robot",
            "type": self.ROBOT_TYPE,
            "version": "0.1.1",
            "devId": self.ROBOT_UUID
        }
        self.client.publish(self.MGR_REGISTER_TOPIC, json.dumps(my_info), qos=1)
        print("Publish robot registeration:", json.dumps(my_info))
        self.client.message_callback_add(self.MQTT_RECV_TOPIC, self.on_recv_topic)
        self.client.subscribe(self.MQTT_RECV_TOPIC)
        self.subscribe_all_on_list()

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    def on_recv_topic(self, client, userdata, msg):
        try:
            js_msg = json.loads(msg.payload.decode())
            # from_dev_id = js_msg["devId"] <= 廃止
            # Managerがpublishするトピックを直接処理するためfrom_dev_idは、
            # payloadの"devID"がこのプロセスのROBOT_UUIDと一致している場合だけ
            # トピック名の第二段目からとりだす。ROBOT_UUIDが一致していなければ
            # 何もしない
            topic_name = msg.topic.split('/')
            if len(topic_name) >= 2 and topic_name[0] == self.MQTT_DEVICE_TOPIC_HDR:
                from_dev_id = topic_name[1]
            else:
                return
                
            print(f"Received message from {from_dev_id} on topic {msg.topic}")
            if from_dev_id and from_dev_id != self.USER_UUID:
                print(f"------------------ {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} -------------------")
                print(f"## Capture New Control Request: {from_dev_id}")
                # If old subscribe exists, unsubscribe
                self.unsubscribe_all_on_list()
                self.USER_UUID = from_dev_id
                # Update topics with new USER_UUID
                self.MQTT_CTRL_JOINT_TOPIC = f"control/{self.joint_topic}{self.USER_UUID}"
                self.MQTT_CTRL_TOOL_TOPIC = f"control/{self.tool_topic}{self.USER_UUID}"
                self.MQTT_SHARE_TOPIC = f"share/{self.USER_UUID}"
                self.subscription_list = [ self.MQTT_CTRL_JOINT_TOPIC,
                                           self.MQTT_CTRL_TOOL_TOPIC,
                                           self.MQTT_SHARE_TOPIC ]
                self.subscribe_all_on_list()
        except Exception as e:
            print(f"XXX Subscribe {self.MQTT_RECV_TOPIC} failed: {e}")
               

    def on_message(self, client, userdata, msg):
        if msg.topic == self.MQTT_CTRL_JOINT_TOPIC:
            # Message ThetaBody
            js_msg = json.loads(msg.payload)
            joints = js_msg['joint']
            thetaBody = [joints[i] if i < len(joints) else 0 for i in range(7)]
            self.pose[8:15] = thetaBody

            if self.time_vr_pub != js_msg["timestamp"]:
                current_time = int(time.time()*1000)
                self.ping = current_time - (self.time_vr_pub + self.time_vr_robot_offset)
                # print("Latency", self.ping)

                self.time_vr_pub = js_msg["timestamp"]
                self.input_count += 1

        elif msg.topic == self.MQTT_CTRL_TOOL_TOPIC:
            # Message ThetaTool
            js_msg = json.loads(msg.payload)
            js_tool = js_msg['tool']
            thetaTool = js_tool
            self.pose[15] = thetaTool

        elif msg.topic == self.MQTT_SHARE_TOPIC:
            js_share = json.loads(msg.payload)
            self.shared_control_flag = js_share["flag"]
            self.shared_signal = js_share["share"]
            self.input_count += 1

        else:
            print("not subscribe msg", msg.topic)

    def create_shared_memory(self, name_shared_memory):
        try:
            self.sm = sm.SharedMemory(name_shared_memory, create=True, size=16 * 4)
            self.pose = np.ndarray((16,), dtype=np.float32, buffer=self.sm.buf)
            self.pose[:] = 0
            print(f"Shared memory {name_shared_memory} created.")
        except FileExistsError:
            self.sm = sm.SharedMemory(name_shared_memory)
            self.pose = np.ndarray((16,), dtype=np.float32, buffer=self.sm.buf)
            print(f"Shared memory {name_shared_memory} already exists.")

    def write_shared_memory(self, name_shared_memory, data=None):
        try:
            # Connect to an existing shared memory block
            existing_sm = sm.SharedMemory(name=name_shared_memory)
            # Create a numpy view of the buffer
            pose = np.ndarray((16,), dtype=np.float32, buffer=existing_sm.buf)

            if data is not None:
                # Ensure data matches shape and dtype
                data = np.array(data, dtype=np.float32)
                if data.shape != pose.shape:
                    raise ValueError(f"Shape mismatch: expected {pose.shape}, got {data.shape}")

                # Write new values into shared memory (directly modifies memory)
                pose[:] = data[:]
                # print("Wrote data to shared memory:", pose[:])
            else:
                print("Current shared memory content:", pose[:])

        except FileNotFoundError:
            print(f"Shared memory {name_shared_memory} not found.")
        finally:
            try:
                existing_sm.close()
            except:
                pass

    def read_shared_memory(self, name_shared_memory):
        try:
            # Connect to an existing shared memory block
            existing_sm = sm.SharedMemory(name=name_shared_memory)
            # Create a numpy view of the shared buffer
            pose = np.ndarray((16,), dtype=np.float32, buffer=existing_sm.buf)
            # print("Read values from shared memory:", pose[:])

            # Optionally, do something with pose...
            return pose.copy()  # copy if you want to avoid changes when buffer updates
        except FileNotFoundError:
            print(f"Shared memory {name_shared_memory} not found.")
            return None
        finally:
            # Close handle (does NOT delete the shared memory)
            try:
                existing_sm.close()
            except:
                pass

    def verify_shared_memory(self, name, expected_shape=(16,), dtype=np.float32):
        try:
            shm = sm.SharedMemory(name=name)
            print("shm", shm )
            arr = np.ndarray(expected_shape, dtype=dtype, buffer=shm.buf)
            # test
            _ = arr[0]
            shm.close()
            print(f"✅ Shared Memory {name} available.")
            return True
        except (FileNotFoundError, ValueError, BufferError):
            print(f"❌ Shared Memory {name} not exist or unavailable.")
            return False

    def close_shared_memory(self, name_shm):
        shm = sm.SharedMemory(name=name_shm)
        shm.close()
        print(f"Shared Memory {name_shm} closed.")
        shm.unlink()
        print(f"Shared Memory {name_shm} unlinked (deleted).")
        return

    def publish_message(self, payload_dict):
        self.client.publish(self.MQTT_ROBOT_STATE_TOPIC, json.dumps(payload_dict))
        # if self.USER_UUID:
        #     self.MQTT_ROBOT_STATE_TOPIC = f"robot/{self.USER_UUID}"
        #     self.client.publish(self.MQTT_ROBOT_STATE_TOPIC, json.dumps(payload_dict))

    def set_time_offset(self, time_offset):
        self.time_vr_robot_offset = time_offset

    def set_ping_init(self, ping_init):
        self.ping = ping_init

    def set_current_time(self, looptime):
        self.current_time = looptime

    def reset_input_count(self):
        self.input_count = 0
