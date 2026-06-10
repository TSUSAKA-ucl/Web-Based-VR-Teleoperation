import time
from piper_sdk import *
import numpy as np

class PIPERControl:
    def __init__(self, can_port):
        self.piper = C_PiperInterface(can_port)
        self.joint_factor = 57324.840764
        self.gripper_factor = 1000
        self.joint_offset = [0, + np.radians(90), - np.radians(170), 0, 0, 0]

    def enable_fun(self):
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while not (enable_flag):
            elapsed_time = time.time() - start_time
            print("--------------------")
            enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                          self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                          self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                          self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                          self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                          self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
            print("Enable Flag:", enable_flag)
            self.piper.EnableArm(7)

            print("--------------------")
            if elapsed_time > timeout:
                print("Time Out....")
                elapsed_time_flag = True
                enable_flag = True
                break
            time.sleep(0.5)
            pass
        if (elapsed_time_flag):
            print("Time out, exit program.")
            exit(0)

    def connect(self):
        self.piper.ConnectPort()
        self.enable_fun()

    def joint_control(self, joint_position, joint_speed):
        factor = self.joint_factor
        joint_0 = round(joint_position[0] * factor)
        joint_1 = round(joint_position[1] * factor)
        joint_2 = round(joint_position[2] * factor)
        joint_3 = round(joint_position[3] * factor)
        joint_4 = round(joint_position[4] * factor)
        joint_5 = round(joint_position[5] * factor)
        self.piper.MotionCtrl_2(0x01, 0x01, joint_speed, 0x00)
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    def joint_control_offset(self, joint_position, joint_speed):
        factor = self.joint_factor
        joint_0 = round((joint_position[0] + self.joint_offset[0]) * factor)
        joint_1 = round((joint_position[1] + self.joint_offset[1]) * factor)
        joint_2 = round((joint_position[2] + self.joint_offset[2]) * factor)
        joint_3 = round((joint_position[3] + self.joint_offset[3]) * factor)
        joint_4 = round((joint_position[4] + self.joint_offset[4]) * factor)
        joint_5 = round((joint_position[5] + self.joint_offset[5]) * factor)
        self.piper.MotionCtrl_2(0x01, 0x01, joint_speed, 0x00)
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    def gripper_control(self, gripper_position, effort):
        gripper_angle = round((gripper_position) * self.gripper_factor)  # mm * gripper_factor, 5mm offset
        self.piper.GripperCtrl(abs(gripper_angle), effort, 0x01, 0)

    def get_joint_feedback(self):
        msg = self.piper.GetArmJointMsgs()
        factor = self.joint_factor

        joint_1 = msg.joint_state.joint_1 / factor
        joint_2 = msg.joint_state.joint_2 / factor
        joint_3 = msg.joint_state.joint_3 / factor
        joint_4 = msg.joint_state.joint_4 / factor
        joint_5 = msg.joint_state.joint_5 / factor
        joint_6 = msg.joint_state.joint_6 / factor
        theta_Body_feedback = [joint_1, joint_2, joint_3, joint_4,
                               joint_5, joint_6]

        return theta_Body_feedback

    def get_joint_feedback_mr(self):
        msg = self.piper.GetArmJointMsgs()
        factor = self.joint_factor

        joint_1 = msg.joint_state.joint_1 / factor - self.joint_offset[0]
        joint_2 = msg.joint_state.joint_2 / factor - self.joint_offset[1]
        joint_3 = msg.joint_state.joint_3 / factor - self.joint_offset[2]
        joint_4 = msg.joint_state.joint_4 / factor - self.joint_offset[3]
        joint_5 = msg.joint_state.joint_5 / factor - self.joint_offset[4]
        joint_6 = msg.joint_state.joint_6 / factor - self.joint_offset[5]
        theta_Body_feedback = [joint_1, joint_2, joint_3, joint_4,
                               joint_5, joint_6]

        return theta_Body_feedback

    def left_arm_initialize(self):
        joint_position = [0 + np.radians(0),
                    -0.27473 + np.radians(20)   + np.radians(90),
                    1.44144 + np.radians(45)    - np.radians(170),
                    0 - np.radians(90),
                    1.22586 - np.radians(10),
                    np.radians(0)]
        joint_speed = 5
        self.joint_control(joint_position, joint_speed)
        self.gripper_control(65, 200)

    def right_arm_initialize(self):
        joint_position = [0 + np.radians(0),
                    -0.27473 + np.radians(20)   + np.radians(90) ,
                    1.44144 + np.radians(45)    - np.radians(170),
                    0 + np.radians(90),
                    1.22586 - np.radians(10),
                    np.radians(0)]
        self.joint_control(joint_position, 5)
        self.gripper_control(65, 200)

    def right_arm_work_position(self):
        joint_position = [-0.0929788888894261, 2.045884444456265, -1.0069980000058183, 0.9584850000055379,
                          1.2212332222292783, 0.5381611111142205]
        self.joint_control(joint_position, 5)
        self.gripper_control(65, 100)
        return joint_position

    def set_arm_zero(self):
        joint_position = [0,0,0,0,0,0]
        self.joint_control(joint_position, 5)
        self.gripper_control(0, 100)

if __name__ == "__main__":
    # Piper Initialize
    can_port = "can0"
    piper_interface = PIPERControl(can_port)
    piper_interface.connect()
    time.sleep(0.1)

    # # joint control
    # joint_position = [0.12170180876935273, 2.1025981503576543, -0.9902125513450778, 1.109575618916224, 1.2116075670111095, 0.536025519994889]
    # joint_speed = 5
    # piper_interface.joint_control(joint_position, joint_speed)
    #
    # # gripper control
    # gripper_position = 60
    # gripper_effort = 100
    # piper_interface.gripper_control(gripper_position, gripper_effort)

    piper_interface.right_arm_work_position()


