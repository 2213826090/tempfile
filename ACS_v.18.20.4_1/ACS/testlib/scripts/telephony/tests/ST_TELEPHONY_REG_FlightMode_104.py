from testlib.scripts.android.adb import adb_steps
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
telephony_steps.open_messenger(serial = serial)()
telephony_steps.delete_conversation(serial = serial,
                                    number = number)()
telephony_steps.send_sms(serial = serial,
                        number = number,
                        content = "Heellllooooo",
                        view_to_check = {"resourceId": "com.google.android.apps.messaging:id/message_status", "text":"Not sent. Touch to try again."})()
wifi_steps.set_airplane_mode(serial = serial,
                             state = "OFF")()
ui_steps.press_home(serial = serial)()
ui_steps.close_all_app_from_recent(serial = serial)()
telephony_steps.check_carrier(serial = serial,
                              carrier_name = carrier_name)()
ui_steps.press_back(serial = serial,
                    times = 2)()
telephony_steps.open_messenger(serial = serial)()
telephony_steps.open_new_conversation(serial = serial,
                                  number = number)()
ui_steps.wait_for_view(serial = serial,
                       view_to_find = {"resourceId": "com.google.android.apps.messaging:id/message_status", "text":"Now"},
                       timeout = 120000)()
