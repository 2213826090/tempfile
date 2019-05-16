import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the parameters
mod_name = TC_PARAMETERS("module_name")

# get module alias
exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
mod_alias = get_module_alias(output)
if mod_alias is not None:
    # delete the sensor module
    exec_status, output = DEVICE.run_cmd("adb shell halctl --del {0}".format(mod_alias), 3)

    # check the halctl list
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    mod_alias_empty = get_module_alias(output)
    if mod_alias_empty is not None:
        VERDICT = FAILURE
        OUTPUT += "The hal module was not deleted."

    # reload the module
    exec_status, output = DEVICE.run_cmd("adb shell halctl --add {0}".format(mod_alias), 3)
else:
    VERDICT = FAILURE
    OUTPUT += "The hal module '{0}' not found.".format(mod_name)
