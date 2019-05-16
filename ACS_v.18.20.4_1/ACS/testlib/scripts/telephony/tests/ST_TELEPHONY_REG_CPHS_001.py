from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.logcat import logcat_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
import time
import sys

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
carrier_name = args["carrier_name"]
number = args["number"]

adb_steps.wake_up_device(serial = serial)()
adb_steps.menu_to_unlock(serial = serial)()

ui_steps.press_home(serial = serial)()
ui_steps.close_all_app_from_recent(serial = serial)()
telephony_steps.check_carrier(serial = serial,
                              carrier_name = carrier_name,
                              wait_time = 10000)()

ui_steps.press_back(serial = serial,
                    times = 2)()

logcat_steps.clear_logcat(serial = serial)()
telephony_steps.call_a_number(serial = serial,
                              number = number,
                              wait_time = 10000)()
time.sleep(2)
logcat_steps.grep_for(serial = serial,
                            grep_for_text = "Dialing")()
telephony_steps.end_call(serial = serial,
                              wait_time = 10000)()
