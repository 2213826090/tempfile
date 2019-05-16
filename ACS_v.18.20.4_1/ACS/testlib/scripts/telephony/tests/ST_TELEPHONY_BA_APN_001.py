from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
import sys

"""
    How to run:
        python ST_TELEPHONY_BA_APN_001.py -s device_serial
"""

### initialization ###
globals().update(vars(get_args(sys.argv)))


# This script will ensure that user can manually configure the data connection settings (APN)

try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Test: APN settings entered manually
    telephony_steps.create_or_modify_APN(serial = serial,
                                         new_apn = True,
                                         restore_to_default = False,
                                         no_of_default_apns = 3)()

finally:
    #   TearDown part
    telephony_steps.delete_apn(serial = serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()