from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
import sys

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
carrier_name = args["carrier_name"]
sim_pin = args["sim_pin"]

adb_steps.wake_up_device(serial = serial)()
adb_steps.menu_to_unlock(serial = serial)()
ui_steps.close_all_app_from_recent(serial = serial)()
telephony_steps.set_sim_pin(serial = serial, state = "ON", pin = sim_pin)()
telephony_steps.check_pin_is_requested(serial = serial, pin = sim_pin)()
telephony_steps.enter_pin(serial = serial, pin = sim_pin)()
telephony_steps.check_carrier(serial = serial,
                              carrier_name = carrier_name,
                              wait_time = 10000)()
telephony_steps.set_sim_pin(serial = serial, state = "OFF", pin = sim_pin)()
ui_steps.press_home(serial = serial)()

