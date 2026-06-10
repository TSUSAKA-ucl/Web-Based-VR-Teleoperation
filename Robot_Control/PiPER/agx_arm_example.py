import time
from platform import system
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

# Nero firmware: <= 1.10 → NeroFW.DEFAULT; 1.11 → NeroFW.V111; >= 1.12 → NeroFW.V112.
platform_system = system()
if platform_system == "Windows":
    interface = "agx_cando"
    channel = "0"
elif platform_system == "Linux":
    interface = "socketcan"
    channel = "can0"
elif platform_system == "Darwin":
    interface = "slcan"
    channel = "/dev/ttyACM0"
else:
    raise RuntimeError("pyAgxArm currently documents Linux `socketcan`, Windows `agx_cando`, and macOS `slcan`.")

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.DEFAULT,
    interface=interface,
    channel=channel,
)
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

while True:
    ja = robot.get_joint_angles()
    if ja is not None:
        print(ja.msg)
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)
