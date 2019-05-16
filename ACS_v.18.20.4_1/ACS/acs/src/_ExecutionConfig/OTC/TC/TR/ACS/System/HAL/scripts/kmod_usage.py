import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# issue command halctl
exec_status, output = DEVICE.run_cmd("adb shell kmod", 3)
missing_tokens = check_kmod_usage(output)
if len(missing_tokens) > 0:
    VERDICT = FAILURE
    OUTPUT = "The following parameters are missing from help message: {0}\n".\
                format(missing_tokens)

