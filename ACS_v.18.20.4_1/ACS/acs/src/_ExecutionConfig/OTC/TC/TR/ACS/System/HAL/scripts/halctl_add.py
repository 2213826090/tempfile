import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the parameters
mod_name = TC_PARAMETERS("module_name")
mod_name_2 = TC_PARAMETERS("module_2")
mod_name_3 = TC_PARAMETERS("module_3")


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
    # get the refcount
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    before_ref_count = get_ref_count(output)

    # add the module
    exec_status, output = DEVICE.run_cmd("adb shell halctl --add {0}".format(mod_alias), 3)

    # get the refcount
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    after_ref_count = get_ref_count(output)

    # check the refcount incremented
    if int(before_ref_count) != int(after_ref_count) - 1:
        VERDICT = FAILURE
        OUTPUT += "The refcount incorrectly incremented: {0} {1}.\n".format(before_ref_count, after_ref_count)

    # remove the module
    for i in range(0, int(after_ref_count)):
        exec_status, output = DEVICE.run_cmd("adb shell halctl -s {0}".format(mod_alias), 3)

    # check the module was deleted
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    if get_module_alias(output) is not None:
        VERDICT = FAILURE
        OUTPUT += "The module was not deleted.\n"

    # add the module
    exec_status, output = DEVICE.run_cmd("adb shell halctl --add {0}".format(mod_alias), 3)

    # check the module was added
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    if get_module_alias(output) is None:
        VERDICT = FAILURE
        OUTPUT += "The module was not added.\n"

    # get the aliases of the modules
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name_2), 3)
    mod_alias_2 = get_module_alias(output)

    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name_3), 3)
    mod_alias_3 = get_module_alias(output)


    # create the modules file
    f = open("modules.txt", "w")
    f.writelines([mod_alias + "\n", mod_alias_2 + "\n", mod_alias_3 + "\n"])
    f.close()
    exec_status, output = DEVICE.run_cmd("adb push modules.txt /mnt/sdcard/".format(mod_alias), 3)

    # delete the module
    exec_status, output = DEVICE.run_cmd("adb shell halctl --del {0}".format(mod_alias), 3)

    # check the module was deleted
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    if get_module_alias(output) is not None:
        VERDICT = FAILURE
        OUTPUT += "The module was not deleted.\n"

    # add the module using -f option
    exec_status, output = DEVICE.run_cmd("adb shell halctl -a {0} -f {1}".format(mod_alias_3, "/mnt/sdcard/modules.txt"), 3)

    # check the module was added
    exec_status, output = DEVICE.run_cmd("adb shell halctl -li {0}".format(mod_name), 3)
    if get_module_alias(output) is None:
        VERDICT = FAILURE
        OUTPUT += "The module was not added.\n"

else:
    VERDICT = FAILURE
    OUTPUT += "The hal module '{0}' not found.".format(mod_name)
