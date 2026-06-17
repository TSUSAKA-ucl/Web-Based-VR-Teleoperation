from PiPER.PIPERControl import PIPERControl
import time

if __name__ == "__main__":
    # Piper Initialize
    can_port = "can_piper_r4e31"
    piper= PIPERControl(can_port)
    piper.connect()
    time.sleep(0.1)

    joint_position = piper.right_arm_work_position()
    # joint_position = piper.right_arm_initialize()

    joint_feedback = piper.get_joint_feedback_mr()
    print("joint_feedback", joint_feedback)
