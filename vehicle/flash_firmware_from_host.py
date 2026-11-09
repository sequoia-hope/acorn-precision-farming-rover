
import os
import subprocess
import sys

# Both are required: the rp2040_combine PlatformIO project that holds the
# built firmware, and the robot to flash.
FIRMWARE_FOLDER = os.environ.get("ACORN_FIRMWARE_DIR")
ROBOT_HOSTNAME = os.environ.get("ACORN_ROBOT_HOST")
if not FIRMWARE_FOLDER or not ROBOT_HOSTNAME:
    sys.exit("error: set ACORN_FIRMWARE_DIR (the rp2040_combine project) and "
             "ACORN_ROBOT_HOST (the robot's hostname or address)")
FIRMWARE_FOLDER = os.path.join(FIRMWARE_FOLDER, "")
CAN_SUBPATH = ".pio/build/canbus/firmware.bin"
MOTOR_SUBPATH = ".pio/build/motorcontroller/firmware.bin"

CAN_DEST_NAME = "firmware_can.bin"
MOTOR_DEST_NAME = "firmware_motor.bin"

# CAN_ADDRESSES = [0x6]
# CAN_ADDRESSES = [0x7, 0x6]
CAN_ADDRESSES = [0x6, 0x7, 0x8, 0x9]

FLASH_CAN = True
FLASH_MOTOR = True


def copy_file(file_path, dest_name):
    command = ["rsync","-aP", file_path, "acorn@"+ROBOT_HOSTNAME+":/home/acorn/acorn/vehicle/"+dest_name]
    print(command)
    result = subprocess.run(command, capture_output=True)
    print(result.stdout)

if FLASH_CAN:
    copy_file(FIRMWARE_FOLDER+CAN_SUBPATH, CAN_DEST_NAME)
    print("Copied CAN firmware")
if FLASH_MOTOR:
    copy_file(FIRMWARE_FOLDER+MOTOR_SUBPATH, MOTOR_DEST_NAME)
    print("Copied MOTOR firmware")

for address in CAN_ADDRESSES:
    command = ["ssh","acorn@"+ROBOT_HOSTNAME,f"cd /home/acorn/acorn/vehicle ; python3 can_firmware.py {address} {int(FLASH_CAN)} {int(FLASH_MOTOR)}"]
    print(" ".join(command))
    result = subprocess.run(command, capture_output=True)
    # print(result.stdout)
    for line in result.stdout.splitlines():
        print(line)
