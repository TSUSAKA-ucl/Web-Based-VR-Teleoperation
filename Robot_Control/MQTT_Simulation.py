import argparse
import sys
import time
import multiprocessing.shared_memory as sm
import numpy as np
from MQTT.MQTT_Client import MQTT_Client
from Sim.CoppeliasimControl import CoppeliasimControl

import MQTT.mqtt_common_opt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="WSS経由でMQTTブローカーと接続しshmでCoppeliaSimのpiper_robot_sample.tttシーンにデータを送る"
    )
    parser = MQTT.mqtt_common_opt.add_common_opts(parser)

    name = "Sim"
    arm_topic = 'right/'
    mode = "local"
    args = {
        "host": parser.parse_args().host,
        "port": parser.parse_args().port,
        "tls": True if parser.parse_args().port == 8883 else False
    }
    client = MQTT_Client(arm_topic, mode, args)

    joint_list = ['/piper/joint1', '/piper/joint2', '/piper/joint3', '/piper/joint4', '/piper/joint5', '/piper/joint6']
    tool_list = ['/piper/joint7', '/piper/joint8']
    sim = CoppeliasimControl(joint_list, tool_list)

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

        joint_feedback = sim.get_joint_position()
        arr[0:6] = joint_feedback

        # Send robot current state to MQTT
        time_robot_pub = int(time.time() * 1000)
        robot_state_msg = {
            "time": time_robot_pub,
            "state": "initialize",
            "model": "agilex_piper",
            "joint_feedback": joint_feedback,
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
            if equal_count > 1.0:
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
                "state": "ready",
            }
            client.publish_message(robot_state_msg)
            print("Robot Ready.")
        else:
            print("Robot Not Ready. Please check VR control communication.")

        while equal:
            arr = client.pose
            thetaBody = arr[8:14].astype(float)  # 6Dof robot
            thetaTool = arr[15].astype(float)

            sim.send_joint_position(thetaBody)
            sim.send_tool_position(thetaTool)

            joint_position = sim.get_joint_position()
            print("Current joint position", joint_position)

            time.sleep(0.0165)

    except KeyboardInterrupt:
        print("MQTT Recv Stopped")
        client.close_shared_memory(name)
        sys.exit(0)
    except Exception as e:
        print("MQTT Recv Error:", e)
        client.close_shared_memory(name)
        sys.exit(1)
