from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.telephony import telephony_steps
from testlib.base.base_utils import get_args
import sys
import time

"""
    How to run:
        python ST_TELEPHONY_SYSTEM_MGT_001.py -s device_serial --script-args sim_pin="1234" number="a_phone_number sms_test_content="some_text" iterations=3
"""

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
if "sim_pin" in args:
    sim_pin = args["sim_pin"]
else:
    print "[BLOCKED] SIM PIN was not provided!!!"
    sys.exit(0)
if "number" in args:
    number = args["number"]
else:
    print "[BLOCKED] A number to send SMS was not provided!!!"
    sys.exit(0)
if "iterations" in args:
    iterations = args["iterations"]
else:
    iterations = 1
if "sms_test_content" in args:
    sms_test_content = args["sms_test_content"]
else:
    sms_test_content = "bogus"

adb_steps.wake_up_device(serial = serial,
                         blocking = True)()
adb_steps.menu_to_unlock(serial = serial,
                         blocking = True)()
ui_steps.close_all_app_from_recent(serial = serial,
                                   blocking = True)()
telephony_steps.set_sim_pin(serial = serial,
                            state = "ON",
                            pin = sim_pin,
                            blocking = True)()
telephony_steps.check_pin_is_requested(serial = serial,
                                       pin = sim_pin,
                                       blocking = True)()
telephony_steps.enter_pin(serial = serial,
                          pin = sim_pin,
                          blocking = True)()
for i in range(iterations):
    adb_steps.root_connect_device(serial = serial)()
    adb_steps.kill_process(serial = serial,
                           process_name = "watchdogd",
                           timeout = 5,
                           with_reboot = True,
                           reboot_timeout = 120)()
    local_steps.wait_for_adb(serial = serial)()
    adb_steps.wait_for_ui(serial = serial,
                          boot_to_Android = False,
                          sim_pin_enabled = True)()
    telephony_steps.check_pin_is_requested(serial = serial,
                                           enabled_pin = False,
                                           with_reboot = False)()
    telephony_steps.open_messenger(serial = serial)()
    telephony_steps.send_sms(number = number,
                             content = sms_test_content,
                             view_to_check = {"resourceId": "com.google.android.apps.messaging:id/message_status", "text":"Sent."},
                             serial = serial)()
