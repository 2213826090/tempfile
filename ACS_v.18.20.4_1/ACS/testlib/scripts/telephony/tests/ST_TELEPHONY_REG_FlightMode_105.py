from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.logcat import logcat_steps
import sys, time


"""
    How to run:
        python ST_TELEPHONY_REG_FlightMode_105.py -s device_serial --script-args carrier_name="carrier_name" number="a_phone_number" sms_test_content="some_text"
"""


### initialization ###
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

#   Mandatory params
carrier_name = args["carrier_name"]
number = args["number"]
sms_test_content = args["sms_test_content"]

# This script will check that there is no visible SW crash or reboot of the DUT whicle switching between flight mode \
# ON and flight mode off many times


try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Step1: As fast as possible, within maximum delay of 5 sec after precedent step, set mobile to flight mode ON
    #   Step2: As fast as possible, within maximum delay of 5 sec after precedent step, set mobile to flight mode OFF
    #   Step3: Repeat step 1-2 100 times
    #   Step4: Wait a maximum time of 2 minutes
    #   Step5: Initiate a MO call
    #   Step6: Release the MO call
    #   Step7: Initiate a MO SMS
    #   Step8: Initiate a MO Ping

    for i in range(99):
        wifi_steps.set_airplane_mode(serial = serial,
                             state = "ON")()
        wifi_steps.set_airplane_mode(serial = serial,
                             state = "OFF")()

    time.sleep(120)

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
    time.sleep(5)
    logcat_steps.grep_for(serial = serial,
                          grep_for_text = "Dialing")()
    telephony_steps.end_call(serial = serial,
                             wait_time = 10000)()

    telephony_steps.open_messenger(serial = serial)()
    telephony_steps.send_sms(number = number,
                             content = sms_test_content,
                             view_to_check = {"resourceId": "com.google.android.apps.messaging:id/message_status",\
                                              "text":"Now"},
                             serial = serial)()



finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

