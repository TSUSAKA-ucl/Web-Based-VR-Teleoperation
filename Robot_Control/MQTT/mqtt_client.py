import json
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Protocol

from paho.mqtt import client as mqtt


# ---------------------------------------------------------------------------
# StateBufferProtocol
#   MQTT_Clientが期待するバッファオブジェクトのインターフェース定義。
#   RobotStateBufferはこのProtocolを継承する必要はなく、
#   同名のメソッドが実装されていれば型チェックが通る。
#
#   ペイロードのパーズ・組み立ては buffer 側の責務とし、
#   MQTT_Client はバイト列の受け渡しのみを行う。
# ---------------------------------------------------------------------------
class StateBufferProtocol(Protocol):
    def parse_ctrl_payload(self, raw: bytes) -> None: ...
    def build_state_payload(self, robot_uuid: str) -> str: ...
    def write_robot(self, name: str, data) -> None: ...
    def read_robot(self, name: str): ...
    def create(self) -> None: ...
    def close(self) -> None: ...


# ---------------------------------------------------------------------------
# MQTTConfig
#   接続先サーバー・ポート・トランスポート、およびロボット識別情報と
#   トピック構成をまとめたデータクラス。
#   環境変数の読み込みは呼び出し側(main.py)の責務とし、ここには持たない。
# ---------------------------------------------------------------------------
@dataclass
class MQTTConfig:
    # 接続設定
    host: str
    port: int
    transport: str = "tcp"      # "tcp" or "websockets"

    # ロボット識別
    robot_uuid: str = ""
    robot_type: str = ""
    robot_version: str = "0.1.1"

    # トピック設定
    manage_topic: str = "mgr"
    device_topic: str = "dev"
    ctrl_topic: str = "control"
    robot_state_topic: str = "robot"

    # 接続パラメータ
    keepalive: int = 60

    @property
    def recv_topic(self) -> str:
        """このロボット宛ての受信トピック"""
        return f"{self.device_topic}/{self.robot_uuid}"

    @property
    def register_topic(self) -> str:
        return f"{self.manage_topic}/register"

    @property
    def unregister_topic(self) -> str:
        return f"{self.manage_topic}/unregister"

    def ctrl_topic_for(self, user_uuid: str) -> str:
        """コントローラーUUID別の制御トピック"""
        return f"{self.ctrl_topic}/{user_uuid}"

    def state_pub_topic(self) -> str:
        """ロボット状態パブリッシュトピック"""
        return f"{self.robot_state_topic}/{self.robot_uuid}"


# ---------------------------------------------------------------------------
# MQTT_Client
#   MQTTプロトコルの手順のみを担う。
#   ペイロードの内容・バッファの実装詳細は StateBufferProtocol 経由で隠蔽する。
# ---------------------------------------------------------------------------
class MQTT_Client:
    def __init__(self, config: MQTTConfig, buffer: StateBufferProtocol):
        self.config = config
        self.buffer = buffer

        self.client: Optional[mqtt.Client] = None
        self.user_uuid: Optional[str] = None            # 現在接続中のコントローラーUUID
        self.current_ctrl_topic: Optional[str] = None   # 現在subscribeしている制御トピック

    # ------------------------------------------------------------------
    # MQTTコールバック
    # ------------------------------------------------------------------
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code != 0:
            print(f"Connect failed with code {reason_code}")
            return

        print("MQTT Connected successfully")

        # ロボット登録
        my_info = {
            "date": datetime.now().strftime('%c'),
            "devType": "robot",
            "type": self.config.robot_type,
            "version": self.config.robot_version,
            "devId": self.config.robot_uuid,
        }
        client.publish(self.config.register_topic, json.dumps(my_info), qos=1)
        print("Robot Registered:", json.dumps(my_info))

        # 再接続時も含め常にrecvトピックをsubscribe
        client.subscribe(self.config.recv_topic)
        print(f"📡 Subscribe: {self.config.recv_topic}")

        # 再接続時: 以前のコントローラーが存在すれば制御トピックも再subscribe
        # message_callback_add はconnect前に登録済みのため再登録不要
        if self.current_ctrl_topic:
            client.subscribe(self.current_ctrl_topic)
            print(f"📡 Re-subscribe ctrl topic: {self.current_ctrl_topic}")

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("✅ Disconnected successfully")
        else:
            print(f"⚠️ Disconnected with code {reason_code}")

    def on_recv_topic(self, client, userdata, msg):
        """
        MQTT_RECV_TOPICのメッセージハンドラ。
        コントローラーからの接続要求を受け取り、制御トピックをsubscribeする。
        """
        try:
            controller_msg = json.loads(msg.payload.decode())
            from_dev_id = controller_msg["devId"]

            if not from_dev_id or from_dev_id == self.user_uuid:
                return

            print(f"------------------ {time.strftime('%Y-%m-%d %H:%M:%S')} -------------------")
            print(f"🎯 Capture New Control Request: {from_dev_id}")

            # 既存の制御トピックをunsubscribeしてcallbackも解除
            if self.current_ctrl_topic:
                client.unsubscribe(self.current_ctrl_topic)
                client.message_callback_remove(self.current_ctrl_topic)
                print(f"📴 Unsubscribe: {self.current_ctrl_topic}")

            self.user_uuid = from_dev_id
            self.current_ctrl_topic = self.config.ctrl_topic_for(from_dev_id)

            # subscribe と callback登録をセットで行う
            client.subscribe(self.current_ctrl_topic)
            client.message_callback_add(self.current_ctrl_topic, self.on_ctrl_topic)
            print(f"📡 Subscribe Control Topic: {self.current_ctrl_topic}")

        except Exception as e:
            print(f"❌ on_recv_topic error: {e}")

    def on_ctrl_topic(self, client, userdata, msg):
        """
        制御トピックのメッセージハンドラ。
        ペイロードのバイト列をそのまま buffer に渡す。
        パーズ・構造解釈は buffer の責務。
        """
        try:
            self.buffer.parse_ctrl_payload(msg.payload)
        except Exception as e:
            print(f"⚠️ on_ctrl_topic error: {msg.topic}, {e}")

    # ------------------------------------------------------------------
    # 接続
    # ------------------------------------------------------------------
    def connect(self):
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            transport=self.config.transport,
        )
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect

        # recv_topic のcallbackはここで固定登録する。
        # ctrl_topic のcallbackはコントローラーUUID確定後に on_recv_topic 内で動的登録する。
        self.client.message_callback_add(self.config.recv_topic, self.on_recv_topic)

        self.client.connect(self.config.host, self.config.port, self.config.keepalive)
        self.client.loop_start()

    # ------------------------------------------------------------------
    # パブリッシュ
    # ------------------------------------------------------------------
    def publish_robot_state(self):
        """バッファにロボット状態ペイロードの組み立てを委譲してパブリッシュする"""
        if not self.client or not self.client.is_connected():
            return
        try:
            payload = self.buffer.build_state_payload(self.config.robot_uuid)
            self.client.publish(self.config.state_pub_topic(), payload, qos=0)
        except Exception as e:
            print(f"⚠️ publish_robot_state error: {e}")

    def robot_unregister(self):
        if not self.client:
            return
        unregister_msg = {
            "time": datetime.now().strftime('%c'),
            "devId": self.config.robot_uuid,
        }
        info = self.client.publish(
            self.config.unregister_topic,
            json.dumps(unregister_msg),
            qos=1,
        )
        try:
            info.wait_for_publish(timeout=1.0)
            print(f"Robot {self.config.robot_uuid} Unregistered Successfully.")
        except RuntimeError:
            print("Unregister failed: Message not published (Timeout or Disconnected).")

    # ------------------------------------------------------------------
    # 切断
    # ------------------------------------------------------------------
    def disconnect(self):
        if self.client:
            if self.client.is_connected():
                self.client.disconnect()
            self.client.loop_stop()
