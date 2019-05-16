import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""
power_mod_name = TC_PARAMETERS("power_module_name")
power_mod_author = TC_PARAMETERS("power_module_author")

# check the logcat for HAL module
exec_status, output = DEVICE.run_cmd("adb logcat -d|grep power", 3)
if "Got HAL server module power.i2c.so (power)" not in output:
    VERDICT = FAILURE
    OUTPUT += "The hal module 'power' not found in logcat."

# get the module list
exec_status, output = DEVICE.run_cmd("adb shell halctl -li power", 3)
mod_type = get_module_type(output)

# check the atrtributes
if mod_type != "fallback":
    VERDICT = FAILURE
    OUTPUT += "The hal module type is not 'fallback'."

# get HW module information
exec_status, output = DEVICE.run_cmd("adb shell halctl -g power", 3)
outcome = check_module_info(output, "power", power_mod_name, power_mod_author)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT = "The module info check failed. The expected was:\n" + \
    "HW module ID: power\n" + \
    "HW module name: {0}\n".format(power_mod_name) + \
    "HW module author: {0}\n".format(power_mod_author) + \
    "But found:\n" + output
