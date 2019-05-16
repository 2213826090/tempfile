import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""
camera_mod_name = TC_PARAMETERS("camera_module_name")
camera_mod_author = TC_PARAMETERS("camera_module_author")

# check the logcat for HAL module camera
exec_status, output = DEVICE.run_cmd("adb logcat -d|grep camera", 3)
if "Got HAL server module camera.uvc.so (camera)" not in output:
    VERDICT = FAILURE
    OUTPUT += "The hal module 'camera' not found in logcat."

# get the module liset
exec_status, output = DEVICE.run_cmd("adb shell halctl -li camera", 3)
mod_type = get_module_type(output)

# check the atrtributes
if mod_type != "fallback":
    VERDICT = FAILURE
    OUTPUT += "The hal module type is not 'fallback'."

# get HW module information
exec_status, output = DEVICE.run_cmd("adb shell halctl -g camera", 3)
outcome = check_module_info(output, "camera", camera_mod_name, camera_mod_author)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT = "The module info check failed. The expected was:\n" + \
    "HW module ID: camera\n" + \
    "HW module name: {0}\n".format(camera_mod_name) + \
    "HW module author: {0}\n".format(camera_mod_author) + \
    "But found:\n" + output
