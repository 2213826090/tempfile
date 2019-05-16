import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the parameters
mod_name = TC_PARAMETERS("module_name")
iterations = int(TC_PARAMETERS("iterations"))

# get module alias
exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
mod_alias = get_module_alias(output)

# ensure the module is loaded - easiest way is to kill the hal process
if mod_alias is None:
    # get the pid of the hald process
    exec_status, output = DEVICE.run_cmd("adb shell ps -C hald", 3)
    pid_before = get_process_pid(output)

    # kill HAL process
    exec_status, output = DEVICE.run_cmd("adb shell kill -9 {0}".format(pid_before), 3)
    time.sleep(5)

    # get module alias
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    mod_alias = get_module_alias(output)

if mod_alias is not None:
    # check the ref count
    ref_count = get_ref_count(output)
    if int(ref_count) > 1:
        for i in range(0, int(ref_count) - 1):
            DEVICE.run_cmd("adb shell halctl --del {0}".format(mod_alias), 3)
            time.sleep(1)

    # excute del/add <iterations> times
    for i in range(0, iterations):
        # delete the sensor module
        exec_status, output = DEVICE.run_cmd("adb shell halctl --del {0}".format(mod_alias), 3)

        # check the halctl list
        exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
        mod_alias_empty = get_module_alias(output)
        if mod_alias_empty is not None:
            VERDICT = FAILURE
            OUTPUT += "The hal module was not deleted."
        time.sleep(1)

        # reload the module
        exec_status, output = DEVICE.run_cmd("adb shell halctl --add {0}".format(mod_alias), 3)

        # check the halctl list
        exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
        mod_alias_after = get_module_alias(output)
        if mod_alias_after is None:
            VERDICT = FAILURE
            OUTPUT += "The hal module was not deleted."
        time.sleep(1)
else:
    VERDICT = FAILURE
    OUTPUT += "The hal module '{0}' not found.".format(mod_name)
