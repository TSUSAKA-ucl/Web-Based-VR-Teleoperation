import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# constructorのオプション引数でcoppeliaSimサーバーのIP,FQDNを入れる
class CoppeliasimControl:
    def __init__(self, joint_name_list, tool_name_list, ip_address='localhost', port=23000):
        copp_client = RemoteAPIClient(host=ip_address, port=port)
        print(f"Connected to CoppeliaSim at {ip_address}:{port}")
        self.sim = copp_client.getObject('sim')
        self.joint_handles = [self.sim.getObject(joint_name) for joint_name in joint_name_list]
        self.tool_handles = [self.sim.getObject(tool_name) for tool_name in tool_name_list]
        self.joint_offset_agilex = [0, + np.radians(90), - np.radians(170), 0, 0, 0]

    def send_joint_position(self, thetaBody):
        for i, handle in enumerate(self.joint_handles):
            self.sim.setJointTargetPosition(handle, thetaBody[i])

    def send_tool_position(self, thetaTool):
        finger_pos_1 = (((thetaTool) * 0.4) / 1000) + 0.0004
        finger_pos_2 = (((thetaTool) * 0.4) / 1000) + 0.0004
        finger_poses = [finger_pos_1, finger_pos_2]
        for i, handle in enumerate(self.tool_handles):
            self.sim.setJointTargetPosition(handle, finger_poses[i])

    def get_joint_position(self):
        """读取当前的关节角（单位：弧度）"""
        joint_position = [self.sim.getJointPosition(handle) for handle in self.joint_handles]
        return joint_position

    def get_joint_position_agilex(self):
        joint_position = [self.sim.getJointPosition(handle) for handle in self.joint_handles]
        joint_position = [joint_position[i] + self.joint_offset_agilex[i] for i in range(len(joint_position))]
        return joint_position

    def send_joint_position_agilex(self, thetaBody):
        thetaBody = [thetaBody[i] - self.joint_offset_agilex[i] for i in range(len(thetaBody))]
        for i, handle in enumerate(self.joint_handles):
            self.sim.setJointTargetPosition(handle, thetaBody[i])

joint_list = ['/piper/joint1', '/piper/joint2', '/piper/joint3', '/piper/joint4', '/piper/joint5', '/piper/joint6']
tool_list = ['/piper/joint7', '/piper/joint8']
