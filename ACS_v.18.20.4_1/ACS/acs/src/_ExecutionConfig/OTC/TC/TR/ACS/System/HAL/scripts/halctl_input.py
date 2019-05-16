import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get module alias
exec_status, output = DEVICE.run_cmd("adb shell halctl -li input", 3)
mod_alias = get_module_alias(output)

if mod_alias is not None:
    # check the entries count
    binding_count = get_binding_count(output)
    if binding_count is None:
        VERDICT = FAILURE
        OUTPUT += "The binding list is empty.\n"

    # delete the module
    exec_status, output = DEVICE.run_cmd("adb shell halctl -s {0}".format(mod_alias), 3)

    # check the halctl list
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li input", 3)
    binding_count_after = get_binding_count(output)
    if int(binding_count_after) != int(binding_count) - 1:
        VERDICT = FAILURE
        OUTPUT += "The hal module was not deleted.\n"

    # reload the module
    exec_status, output = DEVICE.run_cmd("adb shell halctl -a {0}".format(mod_alias), 3)

    # check the halctl list
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li input", 3)
    binding_count_after = get_binding_count(output)
    if int(binding_count_after) != int(binding_count):
        VERDICT = FAILURE
        OUTPUT += "The hal module was not deleted.\n"
else:
    VERDICT = FAILURE
    OUTPUT += "The hal module 'input' not found.\n"
