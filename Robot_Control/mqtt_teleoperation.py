import argparse
import os
import sys
import time

from dotenv import load_dotenv
import numpy as np
import modern_robotics as mr

from MQTT.mqtt_client import MQTT_Client, MQTTConfig

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))

SERVER_PRESETS = {
    "local": {
        "host":      os.getenv("MQTT_LOCAL_HOST", "localhost"),
        "port":      int(os.getenv("MQTT_LOCAL_PORT", "9001")),
        "transport": "websockets",
    },
    "uclab": {
        "host":      os.getenv("MQTT_UCLAB_HOST", "sora2.uclab.jp"),
        "port":      int(os.getenv("MQTT_UCLAB_PORT", "1883")),
        "transport": "tcp",
    },
}


def build_config(mode: str) -> MQTTConfig:
    preset = SERVER_PRESETS[mode]
    return MQTTConfig(
        host=preset["host"],
        port=preset["port"],
        transport=preset["transport"],
        robot_uuid=os.getenv("ROBOT_UUID", "2A5PE-YUSHU008"),
        robot_type=os.getenv("ROBOT_TYPE", "Unitree-G1-Dex3"),
        manage_topic=os.getenv("MQTT_MANAGE_TOPIC", "mgr"),
        device_topic=os.getenv("MQTT_DEVICE_TOPIC", "dev"),
        ctrl_topic=os.getenv("MQTT_CTRL_TOPIC", "control"),
        robot_state_topic=os.getenv("MQTT_ROBOT_STATE_TOPIC", "robot"),
    )

class EarlyExitSuccess(Exception):
    pass

def main():
    parser = argparse.ArgumentParser(description='PiPER Teleoperation with MQTT')
    parser.add_argument('--mode', type=str, default='local', choices=SERVER_PRESETS
                        , help='MQTT server mode (default: local)')
    parser.add_argument('--can_port', type=str, default='can1', help='CAN port for PiPER (default: can1)')
    args = parser.parse_args()
    config = build_config(args.mode)

    piper = PiperControlBuffer(args.can_port) # PIPERControl(args.can_port)
    piper.connect()
    time.sleep(1)

    # MQTT_Clientはpiperのparse_ctrl_payloadとbuild_state_payloadを呼び出すため、
    # piperを引数に取る
    # parse_ctrl_payloadはctrl_topicのcallbackで呼ばれ、
    # build_state_payloadはpublish_robot_state()の中で呼ばれる
    # payloadはrobot type毎にフロントエンドに対応して異なり、piper(control buffer)がそれを処理する
    client = MQTT_Client(config, piper)

    # vr_time_offset = np.load("time_offset.npy")
    # client.set_time_offset(vr_time_offset)

    # Control Parameter
    Tf = 0.025
    dt = 0.005
    N = 5
    method = 5
    Kp = 0.75
    Kd = 0.0015

    prev_error = np.zeros(6)

    try:
        client.connect() # 旧 start_mqtt() 
        time.sleep(0.1)

        """
        Check Robot State
        """
        # Update joint message
        # piper.poseはnp.arrayのビューで、arr[8:14]とarr[15]はpiper object内部で
        # 自動的に書き換えられている。arr[0:6]をobject内部で読んでcan通信している。
        arm_read_slice = slice(8, 14)
        arm_mqtt_slice = slice(0, 6)
        tool_read_index = 15
        tool_mqtt_index = 6
        arr = piper.pose # (view) 旧 client.pose
        thetaBody = arr[arm_read_slice].astype(float)  # 6Dof robot. copy.
        thetaTool = arr[tool_read_index].astype(float)

        piper.read_robot() # set current joint state to piper.pose (arr[arm_read_slice])
        # joint_feedback = piper.get_joint_feedback_mr() # piper.read()
        # msg = piper_sdk.GetArmJointMsgs() is called in piper_get_joint_feedback_mr()
        # arr[0:6] = joint_feedback

        # timestamp = int(time.time()*1000)
        # Send robot current state to MQTT
        # robot_state_msg = {
        #     "time": timestamp,
        #     "state_left": "initialize",
        #     "model_left": "agilex_piper",
        #     "joint_feedback_left": joint_feedback,
        # }
        # piper.build_state_playoload(uuid)が自動的に呼ばれる
        client.publish_robot_state() # 旧 publish_message(robot_state_msg)
        time.sleep(1.0)


        a = arr[0:6]
        b = arr[8:14]
        equal = np.allclose(a, b)

        if equal:
            # robot_state_msg = {
            #     "state_left": "ready",
            # }
            client.publish_robot_state()
            print("Robot Ready.")
        else:
            raise EarlyExitSuccess("Robot Not Ready. Please check"
                                   "VR control communication.")

        while True:
            # Update joint message
            thetaBody = arr[8:14].astype(float)  # 6Dof robot
            # thetaBody = [round(x, 4) for x in thetaBody]
            # thetaBody = np.array(thetaBody)
            np.round(thetaBody, 4, out=thetaBody)

            thetaTool = arr[15].astype(float)

            # Get joint feedback
            # joint_feedback = piper.get_joint_feedback_mr()
            # joint_feedback = [round(x, 4) for x in joint_feedback]
            # joint_feedback = np.array(joint_feedback)
            # arr[0:6] = joint_feedback
            piper.read_robot()

            error = thetaBody - joint_feedback
            d_error = (error - prev_error) / Tf
            mse = np.mean(error ** 2)  # Mean Square Error
            rmse = np.sqrt(mse)

            # PD control
            control_signal = joint_feedback + Kp * error + Kd * d_error
            prev_error = error.copy()

            # Trajectory Plan
            theta_current = joint_feedback
            theta_target =control_signal
            if rmse > 0.0015:
                theta_traj = mr.JointTrajectory(theta_current, theta_target, Tf, N, method)

                for theta in theta_traj:
                    piper.joint_control_offset(theta, 60) # piper.write()

                    finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                    piper.gripper_control(finger_pos, 1000) #piper.write()

                    time.sleep(dt)
            else:
                finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                piper.gripper_control(finger_pos, 1000) # piper.write()
                time.sleep(dt)

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
    except EarlyExitSuccess as e:
        print(e)
        sys.exit(0)


if __name__ == "__main__":
    main()
