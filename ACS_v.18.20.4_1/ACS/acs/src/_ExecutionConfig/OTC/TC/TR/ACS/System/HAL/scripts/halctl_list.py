import time
from hal_lib import *

VERDICT = SUCCESS
OUTPUT = ""

# get the hal bindings
exec_status, output = DEVICE.run_cmd("adb shell halctl --list", 3)
binding_count = get_binding_count(output)

# check the binding count
if binding_count is None:
    VERDICT = FAILURE
    OUTPUT += "The hal binding list is empty."

