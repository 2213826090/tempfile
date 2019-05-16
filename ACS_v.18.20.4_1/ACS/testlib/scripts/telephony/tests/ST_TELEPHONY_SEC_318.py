from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))


# This script will test that when entering incorrectly new PIN1 2 times in a row is not permitted
try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Test: Changing PIN1 - Old PIN1 is correct - New PIN1 is not entered correctly 2 times in a row
    telephony_steps.set_sim_pin(serial = serial, state = "ON", pin = telephony_defaults.sim['default_pin'])()

    telephony_steps.change_pin(serial= serial,
                               old_pin = telephony_defaults.sim['default_pin'],
                               new_pin1 = "0000",
                               new_pin2 = "1234",
                               lock_sim_switch_value = False)()
finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()
