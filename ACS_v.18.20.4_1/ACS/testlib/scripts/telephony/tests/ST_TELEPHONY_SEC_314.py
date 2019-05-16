from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))


# mandatory params
old_pin = telephony_defaults.sim['default_pin']
new_pin1 = "123456"


# This script will test if the PIN1 can be changed to any number with 4 to 8 digits

try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Test: PIN - Changing PIN1 - PIN1 can be changed to any number with 4 to 8 digits
    telephony_steps.set_sim_pin(serial = serial, state = "ON", pin = telephony_defaults.sim['default_pin'])()

    telephony_steps.change_pin(serial= serial,
                               old_pin = old_pin,
                               new_pin1 = new_pin1,
                               new_pin2 = new_pin1,
                               lock_sim_switch_value = False)()

finally:
    #   TearDown part
    telephony_steps.set_sim_pin(serial = serial, state = "ON", pin = new_pin1)()

    #   Changing the SIM PIN to the default value
    telephony_steps.change_pin(serial= serial,
                               old_pin = new_pin1,
                               new_pin1 = telephony_defaults.sim['default_pin'],
                               new_pin2 = telephony_defaults.sim['default_pin'],
                               lock_sim_switch_value = False)()

    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

