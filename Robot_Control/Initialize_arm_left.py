from PiPER.PIPERControl import PIPERControl
import time

if __name__ == "__main__":
    # Piper Initialize
    can_port = "can_piper_l282a"
    piper_interface = PIPERControl(can_port)
    piper_interface.connect()
    time.sleep(0.1)

    # piper_interface.left_arm_work_position()
    piper_interface.left_arm_initialize()
