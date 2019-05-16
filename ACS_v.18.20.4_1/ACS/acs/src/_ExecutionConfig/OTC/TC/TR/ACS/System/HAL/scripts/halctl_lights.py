import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""
lights_mod_name = TC_PARAMETERS("lights_module_name")
lights_mod_author = TC_PARAMETERS("lights_module_author")

# check the logcat for HAL module
exec_status, output = DEVICE.run_cmd("adb logcat -d|grep lights", 3)
if "Got HAL server module lights.intel.so (lights)" not in output:
    VERDICT = FAILURE
    OUTPUT += "The hal module 'lights' not found in logcat."

# get the module list
exec_status, output = DEVICE.run_cmd("adb shell halctl -li lights", 3)
mod_type = get_module_type(output)

# check the atrtributes
if mod_type != "fallback":
    VERDICT = FAILURE
    OUTPUT += "The hal module type is not 'fallback'."

# get HW module information
exec_status, output = DEVICE.run_cmd("adb shell halctl -g lights", 3)
outcome = check_module_info(output, "lights", lights_mod_name, lights_mod_author)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT = "The module info check failed. The expected was:\n" + \
    "HW module ID: lights\n" + \
    "HW module name: {0}\n".format(lights_mod_name) + \
    "HW module author: {0}\n".format(lights_mod_author) + \
    "But found:\n" + output
