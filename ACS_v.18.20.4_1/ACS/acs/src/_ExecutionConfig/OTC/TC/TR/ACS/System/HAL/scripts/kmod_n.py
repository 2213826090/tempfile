import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get kmod name
exec_status, output = DEVICE.run_cmd("adb shell halctl --list", 3)
kmod_name = get_kmod_name(output)

# issue kmod verbose command
exec_status, output = DEVICE.run_cmd("adb shell kmod -vvn {0}".format(kmod_name), 3)
missing_tokens = check_kmod_verbose(output)

if len(missing_tokens) > 0:
    VERDICT = FAILURE
    OUTPUT = "The following parameters are missing from verbose output: {0}\n".\
                format(missing_tokens)

# issue kmod command
exec_status, output = DEVICE.run_cmd("adb shell kmod -n {0}".format(kmod_name), 3)
missing_tokens = check_kmod_out(output)

if len(missing_tokens) > 0:
    VERDICT = FAILURE
    OUTPUT += "The following parameters are missing from output: {0}\n".\
                format(missing_tokens)
