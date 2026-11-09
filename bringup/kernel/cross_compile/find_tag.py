
import os
import subprocess
import re
import sys

# The robot to query, e.g. ACORN_ROBOT_HOST=<robot-hostname>. Required.
ROBOT_HOSTNAME = os.environ.get("ACORN_ROBOT_HOST")
if not ROBOT_HOSTNAME:
    sys.exit("error: set ACORN_ROBOT_HOST to the robot's hostname or address")

match_string=r"Installed: \d:(\d.\d+)"


command = ["ssh","acorn@"+ROBOT_HOSTNAME,"apt policy raspberrypi-kernel"]
result = subprocess.run(command, capture_output=True)
# print(result.stdout)

x = re.findall(match_string, str(result.stdout))
print(x[0])
