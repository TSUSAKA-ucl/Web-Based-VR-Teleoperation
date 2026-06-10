import json
import multiprocessing.shared_memory as sm
import time
from typing import List

import numpy as np


# ---------------------------------------------------------------------------
# RobotStateBuffer
#   ロボットの関節制御値・フィードバック値をShared Memoryで保持する。
#   MQTTや通信プロトコルの知識は一切持たない。
#
#   このクラスが担う責務:
#     - Shared Memoryのライフサイクル管理
#     - 制御値・フィードバック値の読み書き
#     - 制御トピックのペイロード(bytes)のパーズ  <- parse_ctrl_payload
#     - ロボット状態ペイロード(JSON文字列)の組み立て <- build_state_payload
#
#   Shared Memoryのレイアウト（各セグメント 16 x float32 = 64 bytes）:
#     [0:8]  制御値   (コントローラー → ロボット)
#     [8:16] フィードバック値 (ロボット → パブリッシュ)
# ---------------------------------------------------------------------------
class RobotStateBuffer:
    SEGMENT_SIZE = 16       # float32 x 16 = 64 bytes
    CTRL_SLICE   = slice(0, 8)
    ROBOT_SLICE  = slice(8, 16)

    # 各セグメントの有効要素数（関節数）
    # ペイロードのパーズ・組み立てにも使用する
    JOINT_COUNTS = {
        "Left_Arm":   8,
        "Left_Hand":  7,
        "Right_Arm":  8,
        "Right_Hand": 7,
        "Waist":      3,
    }

    def __init__(self, segment_names: List[str] = None):
        self.segment_names: List[str] = segment_names or list(self.JOINT_COUNTS.keys())
        self._handles: dict[str, sm.SharedMemory] = {}
        self._arrays: dict[str, np.ndarray] = {}

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------
    def create(self) -> None:
        """Shared Memoryセグメントを作成（既存なら接続）する"""
        for name in self.segment_names:
            try:
                shm = sm.SharedMemory(name=name, create=True, size=self.SEGMENT_SIZE * 4)
                print(f"✅ Shared memory '{name}' created.")
            except FileExistsError:
                shm = sm.SharedMemory(name=name)
                print(f"ℹ️ Shared memory '{name}' already exists, attached.")

            self._handles[name] = shm
            self._arrays[name] = np.ndarray(
                (self.SEGMENT_SIZE,), dtype=np.float32, buffer=shm.buf
            )
            self._arrays[name][:] = 0

    def close(self) -> None:
        """Shared Memoryハンドルをクローズしてunlinkする"""
        for name, shm in self._handles.items():
            shm.close()
            shm.unlink()
        self._handles.clear()
        self._arrays.clear()
        print("🧹 All Shared Memory handles closed.")

    # ------------------------------------------------------------------
    # ペイロードのパーズ・組み立て（MQTT_Clientから呼ばれるインターフェース）
    # ------------------------------------------------------------------
    def parse_ctrl_payload(self, raw: bytes) -> None:
        """
        制御トピックのペイロード(bytes)をパーズしてバッファに書き込む。
        ペイロードのJSON構造はこのメソッドだけが知っている。

        想定ペイロード:
        {
            "devId": "<uuid>",
            "left":  {"arm": [...], "hand": [...]},
            "right": {"arm": [...], "hand": [...]},
            "waist": {"joints": [...]}
        }
        """
        ctrl_msg = json.loads(raw.decode())
        self.write_ctrl("Left_Arm",   ctrl_msg['left']['arm'])
        self.write_ctrl("Left_Hand",  ctrl_msg['left']['hand'])
        self.write_ctrl("Right_Arm",  ctrl_msg['right']['arm'])
        self.write_ctrl("Right_Hand", ctrl_msg['right']['hand'])
        self.write_ctrl("Waist",      ctrl_msg['waist']['joints'])

    def build_state_payload(self, robot_uuid: str) -> str:
        """
        ロボット状態をフィードバック値から読み出してJSON文字列を返す。
        ペイロードのJSON構造はこのメソッドだけが知っている。

        返却ペイロード:
        {
            "header": {"timestamp": <ms>, "devId": "<uuid>"},
            "left":   {"arm": [...], "hand": [...]},
            "right":  {"arm": [...], "hand": [...]},
            "waist":  {"joints": [...]}
        }
        """
        robot_msg = {
            "header": {
                "timestamp": int(time.time() * 1000),
                "devId": robot_uuid,
            },
            "left": {
                "arm":  self.read_robot("Left_Arm")[:8].tolist(),
                "hand": self.read_robot("Left_Hand")[:7].tolist(),
            },
            "right": {
                "arm":  self.read_robot("Right_Arm")[:8].tolist(),
                "hand": self.read_robot("Right_Hand")[:7].tolist(),
            },
            "waist": {
                "joints": self.read_robot("Waist")[:3].tolist(),
            },
        }
        return json.dumps(robot_msg)

    # ------------------------------------------------------------------
    # 低レベル読み書きインターフェース
    # （ロボット本体プロセスなど外部からフィードバック値を書き込む用途）
    # ------------------------------------------------------------------
    def write_ctrl(self, name: str, data) -> None:
        """制御値（コントローラー → ロボット）をバッファに書き込む"""
        self._write(name, data, self.CTRL_SLICE)

    def read_ctrl(self, name: str) -> np.ndarray:
        """制御値をバッファから読み出す"""
        return self._read(name, self.CTRL_SLICE)

    def write_robot(self, name: str, data) -> None:
        """フィードバック値（ロボット実機 → バッファ）を書き込む"""
        self._write(name, data, self.ROBOT_SLICE)

    def read_robot(self, name: str) -> np.ndarray:
        """フィードバック値をバッファから読み出す"""
        return self._read(name, self.ROBOT_SLICE)

    # ------------------------------------------------------------------
    # 内部ユーティリティ
    # ------------------------------------------------------------------
    def _write(self, name: str, data, slc: slice) -> None:
        if name not in self._arrays:
            print(f"❌ Error: Shared memory '{name}' not initialized.")
            return
        arr = np.array(data, dtype=np.float32).flatten()
        length = slc.stop - slc.start
        self._arrays[name][slc] = arr[:length]

    def _read(self, name: str, slc: slice) -> np.ndarray:
        if name not in self._arrays:
            print(f"❌ Error: Shared memory '{name}' not initialized.")
            return np.zeros(slc.stop - slc.start, dtype=np.float32)
        return self._arrays[name][slc].copy()
