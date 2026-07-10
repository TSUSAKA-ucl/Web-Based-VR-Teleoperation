import time
import numpy as np
from PiPER.PIPERControl import PIPERControl
from WebRTC.WebRTC_Client import CustomMessaging
import modern_robotics as mr
import pandas as pd

from constants import TIME_OFFSET_PATH

def main():
    """
    WebRTC Client Setting
    """
    signaling_urls = ["wss://sora2.uclab.jp/signaling"]
    channel_id = "sora_liust_vr_left"
    metadata = {"access_token": "uclab_token"}

    messaging_label_recv = "#control"
    data_channels_recv = [
        {"label": messaging_label_recv, "direction": "sendrecv"}
    ]
    messaging_recv = CustomMessaging(
        signaling_urls=signaling_urls,
        channel_id=channel_id,
        data_channels=data_channels_recv,
        metadata=metadata
    )

    time_offset = np.load(TIME_OFFSET_PATH)
    messaging_recv.set_vr_time_offset(time_offset)

    shm_name = "RightArm"
    messaging_recv.create_shared_memory(shm_name)
    messaging_recv.connect()

    """
    Robot Control Setting
    """
    can_port = "can0"
    piper = PIPERControl(can_port)
    piper.connect()
    time.sleep(1)

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
    # csv_path = f'Delay_WRTC_{record_timestamp}.csv'

    try:
        """
        Check Robot State
        """
        arr = messaging_recv.pose

        joint_feedback = piper.get_joint_feedback_mr()
        joint_feedback = [round(x, 4) for x in joint_feedback]
        joint_feedback = np.array(joint_feedback)
        arr[0:6] = joint_feedback
        arr[8:14] = arr[0:6]

        if arr is not None:
            trigger = True
            print("Recv OK, Teleopration start")
        else:
            trigger = False
            print("Recv None, Please wait WebRTC messaging")

        while trigger:
            # Update joint message
            thetaBody = arr[8:14].astype(float)  # 6Dof robot
            thetaBody = [round(x, 4) for x in thetaBody]
            thetaBody = np.array(thetaBody)
            thetaTool = arr[15].astype(float)

            # Delay Record
            # delay = messaging_recv.delay
            # df = pd.DataFrame([[delay]], columns=columns)
            # df.to_csv(csv_path, mode='a', header=False, index=False)
            # print("delay", delay)

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
            theta_target = control_signal
            if rmse > 0.0015:
                theta_traj = mr.JointTrajectory(theta_current, theta_target, Tf, N, method)

                for theta in theta_traj:
                    piper.joint_control_offset(theta, 60)

                    # finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                    # piper.gripper_control(finger_pos, 1000)

                    time.sleep(dt)
            else:
                # finger_pos = ((thetaTool) * 0.85) + 0.4  # /mm
                # piper.gripper_control(finger_pos, 1000)
                time.sleep(dt)

    except KeyboardInterrupt:
        print("\nProgram Stopped")
    except Exception as e:
        print(f"❌ Got Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        messaging_recv.disconnect()
        print("WebRTC disconnected")

if __name__ == "__main__":
    main()
