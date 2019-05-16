from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))


# This script will test if when changing the PIN when entering a wrong old PIN will pass or fail
try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Test: PIN - Changing PIN1 - Old PIN1 code is not correct
    telephony_steps.enter_wrong_old_pin(serial = serial,
                                        wrong_old_pin = "0000",
                                        new_pin = telephony_defaults.sim['default_pin'])()

finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()