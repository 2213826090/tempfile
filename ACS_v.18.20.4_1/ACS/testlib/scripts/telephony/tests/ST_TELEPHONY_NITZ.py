from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# Mandatory parameters
time_zone_switch_value = args["time_zone_switch_value"]
enable_back_to_on = args["enable_back_to_on"]


try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Testing the Enable/Disable Auto update from NITZ server

    ui_steps.open_settings_app(serial=serial,
                           view_to_find ={"text": "Date & time"},
                           view_to_check = {"text": "Automatic time zone"})()

    ui_steps.enable_disable_auto_timezone(serial=serial, time_zone_switch_value = time_zone_switch_value)()

    telephony_steps.enable_disable_nitz_status(serial=serial,
                                        time_zone_switch_value = time_zone_switch_value,
                                           enable_back_to_on = enable_back_to_on)()
finally:
    ui_steps.enable_disable_auto_timezone(serial=serial, time_zone_switch_value = time_zone_switch_value)()

    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()

    ui_steps.press_home(serial=serial)()


