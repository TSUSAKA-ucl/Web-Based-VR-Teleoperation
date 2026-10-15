#!/usr/bin/env python3
import time
from piper_sdk import *

if __name__ == "__main__":
    piper = C_PiperInterface_V2("can_piper_r4e31")
    piper.ConnectPort()
    factor = 57295.7795 #1000*180/3.1415926
    inv_factor = 1/1000 #1/factor /Pi *180
    while True:
        msgs = piper.GetArmJointMsgs()
        jstate = msgs.joint_state
        # time stamp:1780639375.2284083
        # Hz:200.0
        # ArmMsgFeedBackJointStates:
        # Joint 1:-191
        # Joint 2:-692
        # Joint 3:-1314
        # Joint 4:0
        # Joint 5:22046
        # Joint 6:0
        joint1 = jstate.joint_1 * inv_factor
        joint2 = jstate.joint_2 * inv_factor
        joint3 = jstate.joint_3 * inv_factor
        joint4 = jstate.joint_4 * inv_factor
        joint5 = jstate.joint_5 * inv_factor
        joint6 = jstate.joint_6 * inv_factor
        joints = [joint1, joint2, joint3, joint4, joint5, joint6]
        print(joints)
        time.sleep(0.005)
