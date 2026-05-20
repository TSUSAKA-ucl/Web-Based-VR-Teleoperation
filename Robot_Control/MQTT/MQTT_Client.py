import json
import time
from paho.mqtt import client as mqtt
import multiprocessing.shared_memory as sm

from datetime import datetime
import numpy as np

# For register
# import os
# from dotenv import load_dotenv
# load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
# ROBOT_UUID = os.getenv("ROBOT_UUID", "no-uuid")
# ROBOT_MODEL = os.getenv("ROBOT_MODEL", "piper")
# MQTT_MANAGE_TOPIC = os.getenv("MQTT_MANAGE_TOPIC", "dev")
# MQTT_MANAGE_RCV_TOPIC = os.getenv("MQTT_MANAGE_RCV_TOPIC", "dev") + "/" + ROBOT_UUID

USER_UUID = "82156cf0-e682-49df-9933-2542f33584a4-local";
# b23d999e-7d51-4ad7-8667-86fb76def786-local";
# USER_UUID = "f5e834ab-1bd7-4cf1-9941-1b6a356a24a4-local"  # VR
# USER_UUID = "4bc148a6-10fd-4cec-9110-c42f7889d45b-local" # Browser

MQTT_LOCAL_SERVER = "localhost"
MQTT_LOCAL_PORT = 8333
MQTT_UCLAB_SERVER = "sora2.uclab.jp"
MQTT_UCLAB_PORT = 1883

class MQTT_Client():
    def __init__(self, arm, mode):
        self.joint_topic = arm + 'joint/'
        self.tool_topic = arm + 'tool/'
        self.mode = mode

        self.client = mqtt.Client(transport="websockets")

        self.MQTT_CTRL_JOINT_TOPIC = "control/" + self.joint_topic + USER_UUID
        self.MQTT_CTRL_TOOL_TOPIC = "control/" + self.tool_topic + USER_UUID

        self.time_vr_robot_offset = 0
        self.ping = 0
        self.current_time = 0

        self.input_count = 0

        self.MQTT_SHARE_TOPIC = "share/" + USER_UUID
        self.MQTT_ROBOT_STATE_TOPIC = "robot/" + USER_UUID

        self.time_vr_pub = 0

        self.pose = np.zeros(16)

        self.shared_signal = 0 # shared_control_signal
        self.shared_control_flag = 0

    def on_connect(self, client, userdata, flags, rc):
        self.client.subscribe(self.MQTT_CTRL_JOINT_TOPIC)
        self.client.subscribe(self.MQTT_CTRL_TOOL_TOPIC)
        self.client.subscribe(self.MQTT_SHARE_TOPIC)

        # For register
        # my_info = {
        #     "date": str(datetime.today()),
        #     "version": "0.0.1",
        #     "devType": "robot",
        #     "robotModel": ROBOT_MODEL,
        #     "codeType": "PiPER-control",
        #     "devId": ROBOT_UUID
        # }
        # self.client.publish("mgr/register", json.dumps(my_info))
        # print("Publish", json.dumps(my_info))
        # self.client.subscribe(MQTT_MANAGE_RCV_TOPIC)  # connected -> subscribe

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

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
                print("Latency", self.ping)

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

    def start_mqtt(self):
        if self.mode == "local":
            self.client.tls_set(cert_reqs=0)
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message
            self.client.connect(MQTT_LOCAL_SERVER, MQTT_LOCAL_PORT, 60)
            self.client.loop_start()

        elif self.mode == "uclab":
            self.client.on_connect = self.on_connect
            self.client.on_disconnect = self.on_disconnect
            self.client.on_message = self.on_message
            self.client.connect(MQTT_UCLAB_SERVER, MQTT_UCLAB_PORT, 60)
            self.client.loop_start()

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
        return

    def publish_message(self, payload_dict):
        self.client.publish(self.MQTT_ROBOT_STATE_TOPIC, json.dumps(payload_dict))

    def set_time_offset(self, time_offset):
        self.time_vr_robot_offset = time_offset

    def set_ping_init(self, ping_init):
        self.ping = ping_init

    def set_current_time(self, looptime):
        self.current_time = looptime

    def reset_input_count(self):
        self.input_count = 0
