import time
# import modern_robotics as mr
import numpy as np
from PiPER.PIPERControl import PIPERControl
from Modern_Robotics.ModernRoboticsIK import ModernRoboticsIK
from WebRTC.YOLOSegPose import YOLOSegPose
from constants import TIME_OFFSET_PATH
from MQTT.MQTT_Client import MQTT_Client
from WebRTC.WebRTC_Client import StereoSender
import cv2
import pandas as pd

import Modern_Robotics.core as mr

np.set_printoptions(precision=6, suppress=True)

def ik_status(code):
    status_list = [
        ('normal', (0, 255, 0)),        # 0: green
        ('velocity limit', (0, 255, 255)), # 1: yellow
        ('joint limit', (255, 0, 255)), # 2: purple
        ('IK failed', (0, 0, 255))      # 3: red
    ]
    if 0 <= code < len(status_list):
        return status_list[code]
    else:
        return 'unknown', (255, 255, 255)  # default: white


def record_joint(joint_trajectory, q):
    joint_trajectory.append(q.copy())
    if len(joint_trajectory) > 500:
        joint_trajectory.pop(0)  # remove oldest
    return joint_trajectory


class Undistortion:
    def __init__(self):
        self.K = np.array([[788.41415049, 0., 655.01692926],
                                 [0., 787.3765135, 357.82862631],
                                 [0., 0., 1.]], dtype=np.float32)
        self.dist = np.array([[-0.3506601, 0.18558038, -0.00065609, 0.00100313, -0.05786136]], dtype=np.float32)
        self.newcameramtx = None

    def get_cameramtx(self, img):
        h, w = img.shape[:2]
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 0, (w, h))
        return

    def show(self, img):
        undistorted = cv2.undistort(img, self.K, self.dist, None, self.newcameramtx)
        return undistorted

robot_model = 'piper_agilex'
piperik = ModernRoboticsIK(robot_model)
mode = "inSpace"
# mode = "inBody"

can_port = "can0"
piper= PIPERControl(can_port)
piper.connect()
time.sleep(0.5)

name_sh = "RightArm"
arm_topic = 'right/'
client = MQTT_Client(arm_topic, "local")

vr_time_offset = np.load(TIME_OFFSET_PATH)
client.set_time_offset(vr_time_offset)

undist = Undistortion()

signaling_urls = ["wss://sora2.uclab.jp/signaling"]
left_channel = "sora_liust_left"
right_channel = "sora_liust_right"

Tf = 0.025
Kp_v = 0.095
Kd_v = 0.0008

loss_threshold = 0.010
prev_loss_mag = 0

prev_time = time.time()
frame_count = 0

if __name__ == "__main__":
    # send initial pose
    model_path_seg = "WebRTC/best.pt"

    yolo_seg_left = YOLOSegPose(model_path_seg)
    yolo_seg_right = YOLOSegPose(model_path_seg)

    cap = cv2.VideoCapture(0)
    if cap.isOpened() == 0:
        exit(-1)
    # Set the video resolution to HD720 (2*1280*720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Open the Real Sense
    cap_rs = cv2.VideoCapture(6)
    if cap_rs.isOpened() == 0:
        exit(-1)
    # Set the video resolution to HD720 (1280*720)
    cap_rs.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_rs.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    stereo_sender = StereoSender(signaling_urls, left_channel, right_channel)
    stereo_sender.initialize_sora_connections()

    # connect once
    stereo_sender.left_sendonly.connect()
    stereo_sender.right_sendonly.connect()
    stereo_sender.sub_sendonly.connect()

    client.start_mqtt()
    flag_shm = client.verify_shared_memory(name_sh)
    fps = 0

    IK_msg = 'Start'
    IK_color = (0, 255, 0)

    joint_traj_record = []

    """ Record """
    # # Video
    # record_timestamp = int(time.time() * 1000)
    # right_fourcc = cv2.VideoWriter.fourcc(*'mp4v')
    # right_out = cv2.VideoWriter(f'./record/{record_timestamp}_right.mp4', right_fourcc, 30.0, (1280, 720))
    # # Joint data
    # joint_names = [f'joint_{i + 1}' for i in range(6)]
    # columns = ['timestamp'] + ['inputs'] + ['ping'] + joint_names
    # csv_path = f'./record/{record_timestamp}_joint.csv'

    time_start = int(time.time()*1000)
    while True:
        """ Get a new frame from camera """
        retval, frame = cap.read()
        left_right_image = np.split(frame, 2, axis=1)

        # Image Get
        left_image = left_right_image[0]
        right_image = left_right_image[1]

        # Image Undistortion
        left_image = undist.show(left_image)
        right_image = undist.show(right_image)

        # Image Record (Option)
        left_record = left_image.copy()
        right_record = right_image.copy()

        retval_rs, frame_rs = cap_rs.read()
        stereo_sender.send_subcam_frames(frame_rs)

        """ FPS """
        frame_count += 1
        if time.time() - prev_time > 1.0:
            fps = frame_count
            frame_count = 0
            prev_time = time.time()


        """ Shared Control """
        shared_control_flag = client.shared_control_flag
        if shared_control_flag == 1:
            shared_control_signal = client.shared_signal

            # YOLO
            yolo_seg_left.visualize(left_image)
            yolo_seg_right.visualize(right_image)

            # Sort detected objects
            pack_sorted_list_l = yolo_seg_left.get_sorted_items()
            pack_sorted_list_r = yolo_seg_right.get_sorted_items()

            pack_num_l = len(pack_sorted_list_l)
            pack_num_r = len(pack_sorted_list_r)
            pack_num = int((pack_num_l + pack_num_r)/2)

            if pack_num_l > 0 and pack_num_r > 0:
                pack_item_l = pack_sorted_list_l[0]
                pack_item_r = pack_sorted_list_r[0]

                pack_cx_l, pack_cy_l, pack_theta_l = pack_item_l['cx'], pack_item_l['cy'], np.deg2rad(pack_item_l['theta'])
                pack_cx_r, pack_cy_r, pack_theta_r = pack_item_r['cx'], pack_item_r['cy'], np.deg2rad(pack_item_r['theta'])

                cv2.putText(left_image, "target", (int(pack_cx_l), int(pack_cy_l)-30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(right_image, "target", (int(pack_cx_r), int(pack_cy_r)-30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # # left_arm
                # left_arm_cx_l, left_arm_cy_l, left_arm_theta_l = yolo_seg_left.get_tip_pose_left()
                # left_arm_cx_r, left_arm_cy_r, left_arm_theta_r = yolo_seg_right.get_tip_pose_left()
                # print(f"left_arm_pose_l: {left_arm_cx_l}, {left_arm_cy_l}, {np.rad2deg(left_arm_theta_l)}")
                # print(f"left_arm_pose_r: {left_arm_cx_r}, {left_arm_cy_r}, {np.rad2deg(left_arm_theta_r)}")

                # right_arm
                right_arm_cx_l, right_arm_cy_l, right_arm_theta_l = yolo_seg_left.get_tip_pose_right()
                right_arm_cx_r, right_arm_cy_r, right_arm_theta_r = yolo_seg_right.get_tip_pose_right()

                offset_left = right_arm_theta_l - pack_theta_l
                offset_right = right_arm_theta_r - pack_theta_r
                mean_offset_theta = (offset_left + offset_right)/2

                offset_cx_l = (right_arm_cx_l - pack_cx_l)
                offset_cx_r = (right_arm_cx_r - pack_cx_r)
                mean_offset_cx = (offset_cx_l + offset_cx_r)/2

                offset_cy_r = (right_arm_cy_r - pack_cy_r)
                offset_cy_l = (right_arm_cy_l - pack_cy_l)
                mean_offset_cy = (offset_cy_l + offset_cy_r)/2

                loss = [mean_offset_cy/1000, mean_offset_cx/1000, mean_offset_theta]
                loss_hat, loss_mag = mr.AxisAng3(loss)

                d_loss_mag = (loss_mag - prev_loss_mag)/Tf
                prev_loss_mag = loss_mag

                loss_theta = Kp_v * loss_mag + Kd_v * d_loss_mag

                if shared_control_signal == 1 and loss_mag > loss_threshold:
                    # Get current pose
                    theta_start = piper.get_joint_feedback_mr()
                    T = piperik.fk(theta_start, mode)
                    R, p = mr.TransToRp(T)

                    # Position
                    p[0] += loss_hat[0] * loss_theta
                    p[1] += loss_hat[1] * loss_theta
                    # Rotation
                    alpha = loss_hat[2] * loss_theta
                    R = piperik.z_axis_rotate(R, alpha, mode)

                    # Update target joint position
                    T_sd = mr.RpToTrans(R, p)
                    theta_end, status = piperik.IK_joint_velocity_limit(T_sd, theta_start, mode)
                    IK_msg, IK_color = ik_status(status)

                    if status == 0 or status == 1:
                        data_sh = client.read_shared_memory(name_sh)
                        data_sh[8:14] = theta_end
                        client.write_shared_memory(name_sh, data_sh)

                    joint_feedback = piper.get_joint_feedback_mr()
                    timestamp = int(time.time() * 1000)
                    robot_state_msg = {
                        "time": timestamp,
                        "state": "initialize",
                        "model": "agilex_piper",
                        "joint_feedback": joint_feedback,
                    }
                    client.publish_message(robot_state_msg)

                elif shared_control_signal == -1:
                    initial_joint_position = np.array([-0.09306611111164882, 0.47493111766136753, 1.9607769506067685, 0.9579791111166461,
                     1.2210587777848327, 0.5382308888919987])
                    # print(joint_traj_record)
                    # for joint in reversed(joint_traj_record):
                    #     print("joint record", joint)
                    data_sh = client.read_shared_memory(name_sh)
                    data_sh[8:14] = initial_joint_position
                    client.write_shared_memory(name_sh, data_sh)

                    joint_feedback = piper.get_joint_feedback_mr()
                    timestamp = int(time.time() * 1000)
                    robot_state_msg = {
                        "time": timestamp,
                        "state": "initialize",
                        "model": "agilex_piper",
                        "joint_feedback": joint_feedback,
                    }
                    client.publish_message(robot_state_msg)

                    IK_msg = "Reset"
                    IK_color = (0, 255, 0)
                    # joint_traj_record.clear()

                cv2.putText(right_image, f"Number of Detection: {pack_num} ", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(right_image, f"Loss Visual: {loss_mag: .4f} ", (20, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.putText(right_image, "IK Status: " + IK_msg, (1000, 300),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, IK_color, 2)

            else:
                cv2.putText(right_image, f"No Detection ", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


            cv2.putText(right_image, f"fps: {fps}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Visual Assist On", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Visual Assist State: {shared_control_signal}", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # stereo_sender.send_frames(left_image, right_image)
            pack_sorted_list_l.clear()
            pack_sorted_list_r.clear()

        else:
            cv2.putText(right_image, f"fps: {fps}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(right_image, f"Visual Assist OFF", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


        ping = client.ping
        input_count = client.input_count
        timestamp = int(time.time() * 1000)
        cv2.putText(right_image, f"Timestamp: {timestamp}ms", (850, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(right_image, f"Time: {(timestamp-time_start)/1000}s", (850, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(right_image, f"Inputs: {input_count}", (850, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(right_image, f"Latency: {ping}ms", (850, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        stereo_sender.send_frames(left_image, right_image)

        # cv2.imshow("right", right_image)
        # cv2.imshow("left", left_image)

        """
        Record
        """
        # right_out.write(right_image)
        # joint_record = piper.get_joint_feedback_mr()
        # df = pd.DataFrame([[timestamp] + [input_count] + [ping] + joint_record], columns=columns)
        # df.to_csv(csv_path, mode='a', header=False, index=False)

        """
        Keyboard Control
        """
        key_code = cv2.waitKey(1) & 0xFF
        if key_code != 255:
            try:
                key = chr(key_code)
            except ValueError:
                key = ''

            if key == 's':
                timestamp = int(time.time()*1000)
                cv2.imwrite(f'./record/left/zed_left_{timestamp}.jpg', left_record)
                cv2.imwrite(f'./record/right/zed_right_{timestamp}.jpg', right_record)

            elif key == 'r':
                client.reset_input_count()

            elif key == '\x1b':  # ESC (27)
                client.close_shared_memory(name_sh)
                print("Exit program.")
                break

    cap.release()
    # right_out.release() # record
    cv2.destroyAllWindows()





