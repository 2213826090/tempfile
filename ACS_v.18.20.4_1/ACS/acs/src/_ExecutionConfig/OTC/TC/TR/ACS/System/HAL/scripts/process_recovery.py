import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the pid of the hald process
exec_status, output = DEVICE.run_cmd("adb shell ps -C hald", 3)
pid_before = get_process_pid(output)
if pid_before is None:
    VERDICT = FAILURE
    OUTPUT = "No hald process found.\n"

# get the hal bindings
exec_status, output = DEVICE.run_cmd("adb shell halctl -l", 3)
binding_count_1 = get_binding_count(output)

# kill HAL process
exec_status, output = DEVICE.run_cmd("adb shell kill -9 {0}".format(pid_before), 3)
time.sleep(5)

# get the pid of the hald process
exec_status, output = DEVICE.run_cmd("adb shell ps -C hald", 3)
pid_after = get_process_pid(output)
if pid_after is None:
    VERDICT = FAILURE
    OUTPUT = "No hald process found.\n"

# check that the PID changed
if pid_after == pid_before:
    VERDICT = FAILURE
    OUTPUT += "The hald process PID did not change.\n"

# get the new binding count
exec_status, output = DEVICE.run_cmd("adb shell halctl -l", 3)
binding_count_2 = get_binding_count(output)

# check the binding count
if binding_count_1 != binding_count_2:
    VERDICT = FAILURE
    OUTPUT += "The hal bindings differ."

