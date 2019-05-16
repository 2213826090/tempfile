from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.logcat import logcat_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.wifi import wifi_steps
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

no_service = "No service"

adb_steps.wake_up_device(serial = serial)()
adb_steps.menu_to_unlock(serial = serial)()
wifi_steps.set_airplane_mode(serial = serial,
                             state = "ON")()

ui_steps.press_home(serial = serial)()
ui_steps.close_all_app_from_recent(serial = serial)()
telephony_steps.check_carrier(serial = serial,
                              carrier_name = no_service)()
ui_steps.press_back(serial = serial,
                    times = 2)()

logcat_steps.clear_logcat(serial = serial)()
telephony_steps.call_a_number(serial = serial,
                              number = number,
                              view_to_check = {"resourceId":"android:id/message",
                                               "text":"Turn off airplane mode to make a call."},
                              wait_time = 10000)()
ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "OK"},
                      view_to_check = {"resourceId":"android:id/message",
                                      "text":"Turn off airplane mode to make a call."},
                      view_presence = False,
                      wait_time = 10000)()
time.sleep(2)
logcat_steps.grep_for(serial = serial,
                      grep_for_text = "Dialing",
                      text_presence = False)()
wifi_steps.set_airplane_mode(serial = serial,
                             state = "OFF")()

ui_steps.press_home(serial = serial)()
ui_steps.close_all_app_from_recent(serial = serial)()
telephony_steps.check_carrier(serial = serial,
                              carrier_name = carrier_name)()
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
