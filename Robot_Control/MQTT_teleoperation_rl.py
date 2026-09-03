import os
import sys
import time
import argparse
import numpy as np
from collections.abc import Callable
from Sim.CoppeliasimControl import CoppeliasimControl
from PiPER.PIPERControl import PIPERControl
from MQTT.MQTT_Client import MQTT_Client
from MQTT.edge_state import EdgeState

import modern_robotics as mr
# import pandas as pd

from constants import TIME_OFFSET_PATH
import MQTT.mqtt_common_opt

# To run the code, activate can bus at first: bash can_activate.sh can0 1000000


def command_line_args():
    parser = argparse.ArgumentParser(description="wss--mosquitto--can0--piper")
    parser = MQTT.mqtt_common_opt.add_common_opts(parser)
    # 追加のオプション "can0", "can1" (-c "can0")選択, "right"(-r), "left"(-l)排他的選択
    parser.add_argument(
        "-L", "--Liu", action="store_true", help="Liu version topic payload's keys"
    )
    parser.add_argument(
        "-c",
        "--can_port",
        type=str,
        default="None",
        help="CAN port for PiPER (default: can0)",
    )
    parser.add_argument(
        "-s",
        "--simulation",
        action="store_true",
        help="connect to simulation without using CAN bus(default: False)",
    )
    # coppeliaSimのIP
    parser.add_argument(
        "-i",
        "--coppelia_host",
        type=str,
        default="localhost",
        help="IP address of CoppeliaSim (default: localhost)",
    )
    parser.add_argument(
        "-r", "--right_arm", action="store_true", help="Use right arm (default: False)"
    )
    parser.add_argument(
        "-l", "--left_arm", action="store_true", help="Use left arm (default: False)"
    )
    return parser.parse_args()


if __name__ == "__main__":
    parsed_args = command_line_args()
    lr_name = (
        "right" if parsed_args.right_arm else "left" if parsed_args.left_arm else None
    )
    # msg_key_state
    # msg_key_model
    # msg_key_jf
    liu = parsed_args.Liu
    if lr_name == "right":
        msg_key_state = "state"
        msg_key_model = "model"
        msg_key_jf = "joint_feedback" if liu else "joints"

        ext_value = "right/joint"
        ext_tool = "right/tool"
    elif lr_name == "left":
        msg_key_state = "state_left" if liu else "state"
        msg_key_model = "model_left" if liu else "model"
        msg_key_jf = "joint_feedback_left" if liu else "joints"
        ext_value = "left/joint"
        ext_tool = "left/tool"
    else:
        print("Please specify either --right_arm or --left_arm")
        sys.exit(1)

    if lr_name == "right":
        ROBOT_TYPE = os.getenv("ROBOT_TYPE", "piper_right")
        ROBOT_UUID = os.getenv("ROBOT_UUID", "MA100101000019005100402")
    else:
        ROBOT_TYPE = os.getenv("ROBOT_TYPE", "piper_left")
        ROBOT_UUID = os.getenv("ROBOT_UUID", "MA100101000019005100010")

    args = {
        "host": parsed_args.host,
        "port": parsed_args.port,
        "tls": True if parsed_args.port == 8883 else False,
        "robot_type": ROBOT_TYPE,
        "robot_uuid": ROBOT_UUID,
    }

    arm_topic = lr_name + "/"
    mode = "local"
    client = MQTT_Client(arm_topic, mode, args)
    use_simulation = parsed_args.simulation
    print("use_simulation:", use_simulation)
    if use_simulation:
        name = "Sim"
        joint_list = [
            "/piper/joint1",
            "/piper/joint2",
            "/piper/joint3",
            "/piper/joint4",
            "/piper/joint5",
            "/piper/joint6",
        ]
        tool_list = ["/piper/joint7", "/piper/joint8"]
        sim = CoppeliasimControl(
            joint_list, tool_list, ip_address=parsed_args.coppelia_host
        )
        joint_set_func: Callable[[np.ndarray, float], None]
        joint_read_func: Callable[[], np.ndarray]
        joint_read_func = (
            sim.get_joint_position if liu else sim.get_joint_position_agilex
        )
        joint_set_func = (
            sim.send_joint_position if liu else sim.send_joint_position_agilex
        )
    else:
        can_port = parsed_args.can_port
        if can_port == "None":
            if parsed_args.right_arm:
                can_port = "can_piper_r4e31"
            elif parsed_args.left_arm:
                can_port = "can_piper_l282a"
            else:
                can_port = "can0"
        print("Using CAN port:", can_port)
        piper = PIPERControl(can_port)
        piper.connect()
        time.sleep(1)
        name = lr_name + "_arm"
        joint_read_func = (
            piper.get_joint_feedback_mr if liu else piper.get_joint_feedback
        )
        joint_set_func = piper.joint_control_offset if liu else piper.joint_control

    # Control Parameter
    Tf = 0.025
    dt = 0.005
    N = 5
    method = 5
    Kp = 0.75
    Kd = 0.0015

    prev_error = np.zeros(6)

    record_timestamp = int(time.time() * 1000)
    # columns = ['delay']
    # csv_path = f'Delay_MQTT_{record_timestamp}.csv'

    # slice部分に名前をつける
    VIRTUAL_JOINT_SLICE = slice(8, 14)  # 仮想関節のスライス
    VIRTUAL_TOOL_INDEX = 15  # 仮想ツールのインデックス
    REAL_JOINT_SLICE = slice(0, 6)  # 実関節のスライス
    try:
        client.create_shared_memory(name)
        client.start_mqtt()
        time.sleep(0.1)

        """
        Check Robot State
        """
        # client.poseはNumpy配列だが、paho.mqttのon_messageの別スレッドと共有され
        # 排他処理を行う必要があるためコードを修正する
        # clientクラスオブジェクトにclient.lockという名のlockが存在することとする
        # Update joint message
        arr = client.pose

        # ACQUIRED and RUNNIG state loop
        while True:
            while True:
                with client.lock:
                    if client.state == EdgeState.ACQUIRED:
                        break
                time.sleep(0.1)

            with client.lock:
                thetaBody = arr[VIRTUAL_JOINT_SLICE].astype(float)  # 6Dof robot
                thetaTool = arr[VIRTUAL_TOOL_INDEX].astype(float)

            joint_feedback = joint_read_func()
            with client.lock:
                arr[REAL_JOINT_SLICE] = joint_feedback

            # Send robot current state to MQTT
            time_robot_pub = int(time.time() * 1000)
            robot_state_msg = {
                "time": time_robot_pub,
                msg_key_state: "initialize",
                msg_key_model: "agilex_piper",
                msg_key_jf: joint_feedback,
                "ext": ext_value,
            }
            tool_state_msg = {
                "time": time_robot_pub,
                msg_key_state: "initialize",
                msg_key_model: "agilex_piper",
                msg_key_jf: joint_feedback,
                "ext": ext_tool,
            }
            client.publish_message(robot_state_msg)
            client.publish_message(tool_state_msg)
            print("time_robot_pub", time_robot_pub)

            with client.lock:
                tmp_arr = arr.copy()
            print("shared memory:", tmp_arr)
            with client.lock:
                # a = arr[0:6]
                # b = arr[8:14]
                # equal = np.allclose(a, b)
                # aとbの差の全てが±0.01以内ならばequal=Trueとする
                equal = np.all(
                    np.abs(arr[REAL_JOINT_SLICE] - arr[VIRTUAL_JOINT_SLICE]) < 0.01
                )

            equal_count = 0.0
            print_timer = 0.0
            print("##### enter while loop waiting for equal #####")
            while not equal:
                with client.lock:
                    equal = np.all(
                        np.abs(arr[REAL_JOINT_SLICE] - arr[VIRTUAL_JOINT_SLICE]) < 0.01
                    )
                time.sleep(0.010)
                equal_count += 0.010
                print_timer += 0.010
                if print_timer >= 5.0:
                    print(f"Waiting for equal... {equal_count:.2f} seconds elapsed")
                    # aとbを%8.4fで表示
                    with client.lock:
                        tmp_a = arr[REAL_JOINT_SLICE].copy()
                        tmp_b = arr[VIRTUAL_JOINT_SLICE].copy()
                    print(" Real  Robot:", ["%8.4f" % x for x in tmp_a])
                    print(" WebVR Robot:", ["%8.4f" % x for x in tmp_b])
                    client.publish_message(robot_state_msg)
                    client.publish_message(tool_state_msg)
                    print("publish robot_state_msg:", robot_state_msg)
                    print_timer = 0
                if equal_count > 600.0:  # 10 minutes
                    break

            print("##### exit (not equal) while loop #####")
            with client.lock:
                client.state = EdgeState.RUNNING
            if equal:
                time_robot_recv = int(time.time() * 1000)
                print("time_robot_recv", time_robot_recv)
                ping_init = (time_robot_recv - time_robot_pub) / 2
                client.set_ping_init(ping_init)
                print("ping_init:", ping_init)

                time_vr_pub = client.time_vr_pub
                print("time_vr_pub:", time_vr_pub)
                time_offset = time_robot_recv - (time_vr_pub + ping_init)
                print("time_offset:", time_offset)
                client.set_time_offset(time_offset)
                np.save(TIME_OFFSET_PATH, time_offset)

                robot_state_msg = {
                    "time": time_robot_recv,
                    msg_key_state: "ready",
                    msg_key_model: "agilex_piper",
                    "ext": ext_value,
                }
                tool_state_msg = {
                    "time": time_robot_recv,
                    msg_key_state: "ready",
                    msg_key_model: "agilex_piper",
                    "ext": ext_tool,
                }
                client.publish_message(robot_state_msg)
                client.publish_message(tool_state_msg)
            with client.lock:
                tmp_a = arr[REAL_JOINT_SLICE].copy()
                tmp_b = arr[VIRTUAL_JOINT_SLICE].copy()
            print("Real  Robot:", tmp_a)
            print("WebVR Robot:", tmp_b)
            if equal:
                print("##### Robot Ready.")
            else:
                print("Robot Not Ready. Please check VR control communication.")

            while equal:  # running state: Main Control Loop
                print_timer += dt
                if print_timer >= 0.3:
                    print_timer = 0
                # Update joint message
                with client.lock:
                    thetaBody = arr[VIRTUAL_JOINT_SLICE].astype(float)  # 6Dof robot
                thetaBody = [round(x, 4) for x in thetaBody]
                thetaBody = np.array(thetaBody)

                # Delay Record
                # df = pd.DataFrame([[client.ping]], columns=columns)
                # df.to_csv(csv_path, mode='a', header=False, index=False)

                with client.lock:
                    thetaTool = arr[VIRTUAL_TOOL_INDEX].astype(float)
                if print_timer == 0:
                    print("thetaBody:", ["%8.4f" % x for x in thetaBody])
                    print("thetaTool:", thetaTool)

                if use_simulation:
                    # sim.send_joint_position(thetaBody)
                    joint_set_func(thetaBody, 60)
                    sim.send_tool_position(thetaTool)
                    # joint_position = sim.get_joint_position()
                    joint_position = joint_read_func()
                    time.sleep(0.0165)

                else:
                    # Get joint feedback
                    # joint_feedback = piper.get_joint_feedback_mr()
                    joint_feedback = joint_read_func()
                    joint_feedback = [round(x, 4) for x in joint_feedback]
                    joint_feedback = np.array(joint_feedback)
                    with client.lock:
                        arr[REAL_JOINT_SLICE] = joint_feedback

                    error = thetaBody - joint_feedback
                    d_error = (error - prev_error) / Tf
                    mse = np.mean(error**2)  # Mean Square Error
                    rmse = np.sqrt(mse)

                    # PD control
                    control_signal = joint_feedback + Kp * error + Kd * d_error
                    prev_error = error.copy()

                    # equal = np.all(np.abs(error) < 0.087)
                    # if not equal:
                    #     break
                    # Trajectory Plan
                    theta_current = joint_feedback
                    theta_target = control_signal
                    if rmse > 0.0015:
                        theta_traj = mr.JointTrajectory(
                            theta_current, theta_target, Tf, N, method
                        )
                        for theta in theta_traj:
                            # piper.joint_control_offset(theta, 60)
                            joint_set_func(theta, 60)
                            finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                            piper.gripper_control(finger_pos, 1000)
                            time.sleep(dt)
                        ## 本来ここでrmseを確認して>0.0015ならばbreakすべき
                    else:
                        finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                        piper.gripper_control(finger_pos, 1000)
                        time.sleep(dt)

                with client.lock:
                    state_local = client.state
                if state_local != EdgeState.RUNNING:
                    break

            print('##### exit equal while loop #####')

    except KeyboardInterrupt:
        print("SIGINT Received. Stopped")
        with client.lock:
            client.state = EdgeState.EXITING
        client.unregister_message()
        robot_state_msg = {
            "state": "stop",
        }
        client.publish_message(robot_state_msg)
        client.publish_message(robot_state_msg)
        client.close_shared_memory(name)
        sys.exit(0)
    except Exception as e:
        print("MQTT Recv Error:", e)
        robot_state_msg = {
            "state": "error",
        }
        client.publish_message(robot_state_msg)
        client.publish_message(robot_state_msg)
        client.close_shared_memory(name)
        sys.exit(1)
