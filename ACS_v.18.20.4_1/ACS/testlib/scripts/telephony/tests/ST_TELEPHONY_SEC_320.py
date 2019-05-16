from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))

# Mandatory params
puk_code ="82018884"

# This script will test that entering incorrectly new PIN1 3 times blocks PIN1
try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Test: Changing PIN1 - Enter wrong PIN1 3 times to block PIN1
    telephony_steps.set_sim_pin(serial = serial, state = "ON", pin = telephony_defaults.sim['default_pin'])()

    for i in range(3):
        telephony_steps.enter_wrong_old_pin(serial = serial,
                                        wrong_old_pin = "0000",
                                        new_pin = telephony_defaults.sim['default_pin'])()

    telephony_steps.unlock_sim(serial = serial, puk = puk_code, new_pin = telephony_defaults.sim['default_pin'])()

    telephony_steps.set_sim_pin(serial = serial, state = "OFF", pin = telephony_defaults.sim['default_pin'])()

finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()
