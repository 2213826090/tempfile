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
reference_cellular_data_switch_state_value = args["reference_cellular_data_switch_state_value"]
ntp_switch_state_value = args["ntp_switch_state_value"]

#   Setup part
adb_steps.wake_up_device(serial=serial)()
adb_steps.menu_to_unlock(serial=serial)()
ui_steps.close_all_app_from_recent(serial=serial)()
ui_steps.press_home(serial=serial)()

#   Specific telephony setup
telephony_steps.setup_set_time_and_date(serial=serial,
                                        reference_cellular_data_switch_state_value = reference_cellular_data_switch_state_value)()


#   Testing the Enable Auto update from NTP server
ui_steps.set_date_and_time(serial = serial,
                            year = "2019",
                            day = "13",
                            ntp_switch_state_value = ntp_switch_state_value)()

telephony_steps.get_device_date(reference_cellular_data_switch_state_value = reference_cellular_data_switch_state_value,
                                ntp_switch_state_value = ntp_switch_state_value,
                                serial=serial)()

#   TearDown part
ui_steps.close_all_app_from_recent(serial=serial)()

ui_steps.press_home(serial=serial)()

