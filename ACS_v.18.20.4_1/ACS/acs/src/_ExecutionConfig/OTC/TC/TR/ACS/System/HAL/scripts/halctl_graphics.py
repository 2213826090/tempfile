import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""
gralloc_mod_name = TC_PARAMETERS("gralloc_module_name")
gralloc_mod_author = TC_PARAMETERS("gralloc_module_author")

# check the dmesg for HAL module
exec_status, output = DEVICE.run_cmd("adb logcat -d|grep gralloc", 3)
if "Got HAL server module gralloc.autodetect.so (gralloc)" not in output:
    VERDICT = FAILURE
    OUTPUT += "The hal module 'gralloc' information not found in logcat.\n"

# check the gralloc in halctl list
exec_status, output = DEVICE.run_cmd("adb shell halctl -li gralloc", 3)
halid = get_module_halid(output)
if halid != "gralloc":
    VERDICT = FAILURE
    OUTPUT = "The module 'gralloc' was not found.\n"

# get HW module information
exec_status, output = DEVICE.run_cmd("adb shell halctl -g gralloc", 3)
outcome = check_module_info(output, "gralloc", gralloc_mod_name, gralloc_mod_author)
if outcome is False:
    VERDICT = FAILURE
    OUTPUT = "The module info check failed. The expected was:\n" + \
    "HW module ID: gralloc\n" + \
    "HW module name: {0}\n".format(gralloc_mod_name) + \
    "HW module author: {0}\n".format(gralloc_mod_author) + \
    "But found:\n" + output

# get module alias
exec_status, output = DEVICE.run_cmd("adb shell halctl -li gralloc", 3)
mod_alias = get_module_alias(output)

# remove and add the module
exec_status, output = DEVICE.run_cmd("adb shell halctl -s {0}".format(mod_alias), 3)

# check the halctl list
exec_status, output = DEVICE.run_cmd("adb shell halctl -li gralloc", 3)
mod_alias_empty = get_module_alias(output)
if mod_alias_empty is not None:
    VERDICT = FAILURE
    OUTPUT += "The hal module was not deleted.\n"

# reload the module
exec_status, output = DEVICE.run_cmd("adb shell halctl --add {0}".format(mod_alias), 3)

# check the halctl list
exec_status, output = DEVICE.run_cmd("adb shell halctl -li gralloc", 3)
mod_alias_gralloc = get_module_alias(output)
if mod_alias_gralloc is None:
    VERDICT = FAILURE
    OUTPUT += "The hal module was not added.\n"

