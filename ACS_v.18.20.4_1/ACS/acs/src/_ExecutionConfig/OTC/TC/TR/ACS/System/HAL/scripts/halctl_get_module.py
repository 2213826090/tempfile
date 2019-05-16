import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the parameters
camera_mod_name = TC_PARAMETERS("camera_module_name")
camera_mod_author = TC_PARAMETERS("camera_module_author")
sensors_mod_name = TC_PARAMETERS("sensors_module_name")
sensors_mod_author = TC_PARAMETERS("sensors_module_author")

# issue the halctl get-module command for camera
exec_status, output = DEVICE.run_cmd("adb shell halctl --get-module camera", 3)
outcome = check_module_info(output, "camera", camera_mod_name, camera_mod_author, mandatory=False)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT = "The module info check failed. The expected was:\n" + \
             "HW module ID:     camera\n" + \
             "HW module name:   {0}\n".format(camera_mod_name) + \
             "HW module author: {0}\n".format(camera_mod_author) + \
             "But found:\n" + output

# issue the halctl get-module command for camera
exec_status, output = DEVICE.run_cmd("adb shell halctl --get-module sensors", 3)
outcome = check_module_info(output, "sensors", sensors_mod_name, sensors_mod_author, mandatory=False)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT += "The module info check failed. The expected was:\n" + \
             "HW module ID:     sensors\n" + \
             "HW module name:   {0}\n".format(camera_mod_name) + \
             "HW module author: {0}\n".format(camera_mod_author) + \
             "But found:\n" + output
