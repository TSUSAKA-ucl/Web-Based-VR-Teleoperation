from PiPER.PIPERControl import PIPERControl

# class RobotBufferProtocol(Protocol):
#     def parse_ctrl_payload(self, raw: bytes) -> None: ...
#     def build_state_payload(self, robot_uuid: str) -> str: ...
#     def write_robot(self, name: str, data) -> None: ...
#     def read_robot(self, name: str): ...
#     def create(self) -> None: ...
#     def close(self) -> None: ...
Class PiperControlBuffer(PIPERControl):
    def __init__(self):
        self.arm_dof = 6
        self.arm_read_idx_s = 8
        self.arm_read_slice = slice(self.arm_read_idx_s,
                                    self.arm_read_idx_s + self.arm_dof)
        self.arm_write_slice = slice(0, self.arm_dof)                                     
        self.tool_read_index = 15
        self.tool_write_index = 6
        self.current_joints = np.zeros(self.arm_dof)

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------
    def create(self) -> None:

    def close(self) -> None:

    # ------------------------------------------------------------------
    # ペイロードのパーズ・組み立て（MQTT_Clientから呼ばれるインターフェース）
    # ------------------------------------------------------------------
    def parse_ctrl_payload(self, raw: bytes) -> None:
        js_msg = json.loads(msg.payload)
        joints = js_msg['joint']
        thetaBody = [joints[i] if i < len(joints) else 0 for i in range(7)]
        self.pose[arm_read_slice] = thetaBody


    def build_state_payload(self, robot_uuid: str) -> str:
        """
        返却payload
        {
             "time": timestamp,
             "state_left": "initialize",
             "model_left": "agilex_piper",
             "joint_feedback_left": joint_feedback,
         }
        """
        robot_state_msg = {
            "time": int(time.time()*1000),
            "state_left": "initialize",


    # ------------------------------------------------------------------
    # 低レベル読み書きインターフェース
    # ロボットへのデータの書き込みや、ロボットからの読み出し
    # ------------------------------------------------------------------
    def write_robot(self, name: str, data: list, speed = 60) -> None:
        # フロントエンドを書き換えたときは_offsetを取る
        # return self.joint_control(data, speed)
        return self.joint_control_offset(data, speed)
    
    def joint_control_offset(self, data: list, speed = 60) -> None:
        # dataはリストで、長さは6以上を想定
        # self.pose[arm_write_slice] = data[:self.arm_dof]
        # self.pose[tool_write_index] = data[self.arm_dof] if len(data) > self.arm_dof else 0
        # self.joint_control(self.pose[arm_write_slice], speed)
        return self.joint_control(data[:self.arm_dof], speed)

    # メイン側のダックタイピングを利用し、numpyを返さないでリストを返すことにする
    def read_robot(self, name: str) -> list:
        # フロントエンドを書き換えたときは_mrを取る
        # return self.get_joint_feedback()
        return self.get_joint_feedback_mr()

    def get_joint_feedback_mr(self) -> list:
        # joint_feedback = self.get_joint_feedback() # numpy array
        joint_feedback = self.get_joint_feedback_mr() # numpy array
        return joint_feedback.tolist()
