from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
import sys
import time

### initialization ###
globals().update(vars(get_args(sys.argv)))

# mandatory params
number = '*#06#'

# Setup part
adb_steps.wake_up_device(serial=serial)()
adb_steps.menu_to_unlock(serial=serial)()
ui_steps.close_all_app_from_recent(serial=serial)()
ui_steps.press_home(serial=serial)()

# Testing the IMEI Code
telephony_steps.open_phone_dialer(serial=serial)()
telephony_steps.dial_number(serial=serial, number=number)()


imeiArray = ["004401521208153", "004401521208161"]

telephony_steps.check_imei_code(serial=serial, imei_code_array = imeiArray)()


ui_steps.click_button(serial = serial,
                      view_to_find = {"text": "OK"},
                      view_to_check = {"resourceId":"android:id/alertTitle",
                                      "text":"IMEI"},
                      view_presence = False,
                      wait_time = 10000)()

# TearDown part
ui_steps.close_all_app_from_recent(serial=serial)()
ui_steps.press_home(serial=serial)()