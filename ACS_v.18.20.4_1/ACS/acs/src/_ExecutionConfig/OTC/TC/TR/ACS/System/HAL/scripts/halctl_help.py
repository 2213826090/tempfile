import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# issue command halctl
exec_status, output1 = DEVICE.run_cmd("adb shell halctl", 3)
missing_tokens = check_help_message(output1)
if len(missing_tokens) > 0:
    VERDICT = FAILURE
    OUTPUT = "The following parameters are missing from help message: {0}\n".\
                format(missing_tokens)

# issue command halctl --help
exec_status, output2 = DEVICE.run_cmd("adb shell halctl --help", 3)
if output2 != output1:
    VERDICT = FAILURE
    OUTPUT = "The help message differ between 'halctl' and 'halctl --help' commands.\n"

# issue command halctl --help
exec_status, output3 = DEVICE.run_cmd("adb shell halctl -h", 3)
if output3 != output1:
    VERDICT = FAILURE
    OUTPUT = "The help message differ between 'halctl' and 'halctl -h' commands.\n"
