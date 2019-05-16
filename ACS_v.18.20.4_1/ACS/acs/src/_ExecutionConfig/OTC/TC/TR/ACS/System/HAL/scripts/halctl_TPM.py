import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""
keystore_mod_name = TC_PARAMETERS("keystore_module_name")
keystore_mod_author = TC_PARAMETERS("keystore_module_author")

# check the dmesg for HAL module
exec_status, output = DEVICE.run_cmd("adb shell dmesg|grep keystore", 3)
if "No HAL binding for [keystore] - Falling back to legacy mode" not in output:
    VERDICT = FAILURE
    OUTPUT += "The hal module 'keystore' information not found in dmesg."

# get HW module information
exec_status, output = DEVICE.run_cmd("adb shell halctl -g keystore", 3)
outcome = check_module_info(output, "keystore", keystore_mod_name, keystore_mod_author)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT = "The module info check failed. The expected was:\n" + \
    "HW module ID: keystore\n" + \
    "HW module name: {0}\n".format(keystore_mod_name) + \
    "HW module author: {0}\n".format(keystore_mod_author) + \
    "But found:\n" + output
