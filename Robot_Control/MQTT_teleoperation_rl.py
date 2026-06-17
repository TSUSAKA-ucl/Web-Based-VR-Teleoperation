import os
import sys
import time
import argparse
import numpy as np
from PiPER.PIPERControl import PIPERControl
from MQTT.MQTT_Client import MQTT_Client
import modern_robotics as mr
# import pandas as pd

import MQTT.mqtt_common_opt


# To run the code, activate can bus at first: bash can_activate.sh can0 1000000

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="wss--mosquitto--can0--piper"
    )
    parser = MQTT.mqtt_common_opt.add_common_opts(parser)
    # 追加のオプション "can0", "can1" (-c "can0")選択, "right"(-r), "left"(-l)排他的選択
    parser.add_argument(
        "-c", "--can_port", type=str, default="None",
        help="CAN port for PiPER (default: can0)"
    )
    parser.add_argument(
        "-r", "--right_arm", action="store_true",
        help="Use right arm (default: False)"
    )
    parser.add_argument(
        "-l", "--left_arm", action="store_true",
        help="Use left arm (default: False)"
    )
    lr_name = (
        "right" if parser.parse_args().right_arm
        else "left" if parser.parse_args().left_arm
        else None
    )
    # msg_key_state
    # msg_key_model
    # msg_key_jf
    if lr_name == "right":
        msg_key_state = "state"
        msg_key_model = "model"
        msg_key_jf = "joint_feedback"
    elif lr_name == "left":
        msg_key_state = "state_left"
        msg_key_model = "model_left"
        msg_key_jf = "joint_feedback_left"
    else:
        print("Please specify either --right_arm or --left_arm")
        sys.exit(1)
    
    # ROBOT_TYPE = os.getenv("ROBOT_TYPE","piper_right")
    # ROBOT_UUID = os.getenv("ROBOT_UUID","MA100101000019005100402")
    # ROBOT_TYPE = os.getenv("ROBOT_TYPE","piper_left")
    # ROBOT_UUID = os.getenv("ROBOT_UUID","MA100101000019005100010")
    if lr_name == "right":
        ROBOT_TYPE = os.getenv("ROBOT_TYPE", "piper_right")
        ROBOT_UUID = os.getenv("ROBOT_UUID", "MA100101000019005100402")
    else:
        ROBOT_TYPE = os.getenv("ROBOT_TYPE", "piper_left")
        ROBOT_UUID = os.getenv("ROBOT_UUID", "MA100101000019005100010")

    args = {
        "host": parser.parse_args().host,
        "port": parser.parse_args().port,
        "tls": True if parser.parse_args().port == 8883 else False,
        "robot_type": ROBOT_TYPE,
        "robot_uuid": ROBOT_UUID,
    }

    can_port = parser.parse_args().can_port
    if can_port == "None":
        if parser.parse_args().right_arm:
            can_port = "can_piper_r4e31"
        elif parser.parse_args().left_arm:
            can_port = "can_piper_l282a"
        else:
            can_port = "can0"
    print("Using CAN port:", can_port)
    piper = PIPERControl(can_port)
    piper.connect()
    time.sleep(1)

    name = lr_name + "_arm"
    arm_topic = lr_name + "/"
    mode = "local"
    client = MQTT_Client(arm_topic, mode, args)

    # Control Parameter
    Tf = 0.025
    dt = 0.005
    N = 5
    method = 5
    Kp = 0.75
    Kd = 0.0015

    prev_error = np.zeros(6)

    record_timestamp = int(time.time()*1000)
    # columns = ['delay']
    # csv_path = f'Delay_MQTT_{record_timestamp}.csv'

    try:
        client.create_shared_memory(name)
        client.start_mqtt()
        time.sleep(0.1)

        """
        Check Robot State
        """
        # Update joint message
        arr = client.pose
        thetaBody = arr[8:14].astype(float)  # 6Dof robot
        thetaTool = arr[15].astype(float)

        joint_feedback = piper.get_joint_feedback_mr()
        arr[0:6] = joint_feedback

        # Send robot current state to MQTT
        time_robot_pub = int(time.time()*1000)
        robot_state_msg = {
            "time": time_robot_pub,
            msg_key_state: "initialize",
            msg_key_model: "agilex_piper",
            msg_key_jf: joint_feedback,
        }
        client.publish_message(robot_state_msg)
        print("time_robot_pub", time_robot_pub)

        print("shared memory:", arr)
        a = arr[0:6]
        b = arr[8:14]
        equal = np.allclose(a, b)

        equal_count = 0
        while not equal:
            a = arr[0:6]
            b = arr[8:14]
            equal = np.allclose(a, b)
            time.sleep(0.010)
            equal_count += 0.010
            if equal_count > 600.0:  # 10 minutes
                break

        if equal:
            time_robot_recv = int(time.time() * 1000)
            print("time_robot_recv", time_robot_recv)
            ping_init = (time_robot_recv - time_robot_pub)/2
            client.set_ping_init(ping_init)
            print("ping_init:", ping_init)

            time_vr_pub = client.time_vr_pub
            print("time_vr_pub:", time_vr_pub)
            time_offset = time_robot_recv - (time_vr_pub + ping_init)
            print("time_offset:", time_offset)
            client.set_time_offset(time_offset)
            np.save('time_offset.npy', time_offset)

            robot_state_msg = {
                "time": time_robot_recv,
                msg_key_state: "ready",
            }
            client.publish_message(robot_state_msg)
            print("Robot Ready.")
        else:
            print("Robot Not Ready. Please check VR control communication.")
            print("shared memory a:", a)
            print("shared memory b:", b)

        while equal:
            # Update joint message
            thetaBody = arr[8:14].astype(float)  # 6Dof robot
            thetaBody = [round(x, 4) for x in thetaBody]
            thetaBody = np.array(thetaBody)

            # Delay Record
            # df = pd.DataFrame([[client.ping]], columns=columns)
            # df.to_csv(csv_path, mode='a', header=False, index=False)

            thetaTool = arr[15].astype(float)

            # Get joint feedback
            joint_feedback = piper.get_joint_feedback_mr()
            joint_feedback = [round(x, 4) for x in joint_feedback]
            joint_feedback = np.array(joint_feedback)
            arr[0:6] = joint_feedback

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
                    piper.joint_control_offset(theta, 60)

                    finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                    piper.gripper_control(finger_pos, 1000)

                    time.sleep(dt)
            else:
                finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                piper.gripper_control(finger_pos, 1000)
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
