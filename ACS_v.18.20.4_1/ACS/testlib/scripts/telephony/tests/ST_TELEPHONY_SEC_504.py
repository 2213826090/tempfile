from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.base.base_utils import get_args
import sys


### initialization ###
globals().update(vars(get_args(sys.argv)))

# Setup part
adb_steps.wake_up_device(serial=serial)()
adb_steps.menu_to_unlock(serial=serial)()
ui_steps.close_all_app_from_recent(serial=serial)()
ui_steps.press_home(serial=serial)()


# Testing the SVN (Software Version Number) Code through DUT UI
ui_steps.open_settings_app(serial= serial,
                           view_to_find ={"text": "About phone"},
                           view_to_check = {"text": "Status"})()

ui_steps.click_button(serial=serial,
                      view_to_find = {"text": "Status"},
                      view_to_check = {"text": "IMEI information"})()

ui_steps.click_button(serial=serial,
                      view_to_find = {"text": "IMEI information"},
                      view_to_check = {"text": "IMEI (Slot1)"})()


imeiArray = ["004401521208153", "004401521208161"]

#   Verifying that the SV code from DUT UI exists and is 2 digits long
for sv_code_index in range(1, (len(imeiArray)+1)):
    sv_code = ui_utils.search_object_in_direction(serial = serial,
                                                   searched_object = {"textContains": "IMEI SV (Slot" + str(sv_code_index) + ")"},
                                                   direction_to_search = "down",
                                                   object_type = {"resourceId": "android:id/summary"}).info["text"]
    telephony_steps.checkSVCode(serial=serial, sv_code = sv_code)()


# TearDown part
ui_steps.close_all_app_from_recent(serial=serial)()
ui_steps.press_home(serial=serial)()