#!/usr/bin/env python3
import argparse
import time
from piper_sdk import *

def main(can_port):
    piper = C_PiperInterface_V2(can_port)
    piper.ConnectPort()
    while True:
        print(piper.GetArmGripperMsgs())
        time.sleep(0.005)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Piper Control")
    parser.add_argument(
        "--can_port", "-c", type=str, default=None,
        help="CAN port for PiPER (default: None)"
    )
    parser.add_argument(
        "-r", "--right_arm", action="store_true",
        help="Use right arm (default: False)"
    )
    parser.add_argument(
        "-l", "--left_arm", action="store_true",
        help="Use left arm (default: False)"
    )
    args = parser.parse_args()
    can_port = args.can_port
    if can_port is None:
        if parser.parse_args().right_arm:
            can_port = "can_piper_r4e31"
        elif parser.parse_args().left_arm:
            can_port = "can_piper_l282a"
        else:
            print("Please specify either --right_arm or --left_arm, or provide --can_port")
            sys.exit(1)
    print(f"Using CAN port: {can_port}")            
    main(can_port)
