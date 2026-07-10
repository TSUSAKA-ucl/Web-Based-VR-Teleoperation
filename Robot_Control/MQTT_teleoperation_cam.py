import sys
import time
import numpy as np
from constants import TIME_OFFSET_PATH
from MQTT.MQTT_Client import MQTT_Client
from pymycobot.mycobot280 import MyCobot280  # import mycobot library,if don't have, first 'pip install pymycobot'

if __name__ == "__main__":
    name = "CamArm"
    arm_topic = 'cam/'
    mode = "local"
    client = MQTT_Client(arm_topic, mode)

    vr_time_offset = np.load(TIME_OFFSET_PATH)
    client.set_time_offset(vr_time_offset)

    try:
        client.create_shared_memory(name)
        client.start_mqtt()
        time.sleep(0.1)

        # use PC and M5 control
        # mc = MyCobot280('COM5', 115200)  # WINDOWS use ，need check port number when you PC
        mc = MyCobot280('/dev/ttyUSB0',115200)           # VM linux use
        mc.set_fresh_mode(1)
        time.sleep(0.5)
        print("Mycobot280 Info", mc)

        power = mc.is_power_on()

        # Update joint message
        arr = client.pose
        thetaBody = arr[8:14].astype(float)  # 6Dof robot
        thetaTool = arr[15].astype(float)

        joint_feedback = mc.get_angles()
        joint_feedback = np.deg2rad(joint_feedback)  # 当前角度（rad）
        arr[0:6] = joint_feedback

        # Send robot current state to MQTT
        robot_state_msg = {
            "state_cam": "initialize",
            "model_cam": "myCobot280",
            "joint_feedback_cam": joint_feedback.tolist(),
        }
        client.publish_message(robot_state_msg)
        time.sleep(1.0)

        a = arr[0:6]
        a = [round(x) for x in a]
        a = np.array(a)
        print("a:", a)

        b = arr[8:14]
        b = [round(x) for x in b]
        b = np.array(b)
        print("b:", b)

        equal = np.allclose(a, b)

        if equal:
            robot_state_msg = {
                "state": "ready",
            }
            client.publish_message(robot_state_msg)
            print("Robot Ready.")
        else:
            print("Robot Not Ready. Please check VR control communication.")

        while equal:
            # Update joint message
            thetaBody = arr[8:14].astype(float)  # 6Dof robot

            """Control Directly"""
            joint_cmd = np.rad2deg(thetaBody)
            mc.send_angles(joint_cmd.tolist(), 30)
            # print("cmd", joint_cmd)
            time.sleep(0.033)

    except KeyboardInterrupt:
        print("MQTT Recv Stopped")
        robot_state_msg = {
            "state": "stop",
        }
        client.publish_message(robot_state_msg)
        sys.exit(0)
    except Exception as e:
        print("MQTT Recv Error:", e)
        robot_state_msg = {
            "state": "error",
        }
        client.publish_message(robot_state_msg)
        sys.exit(1)
