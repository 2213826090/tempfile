from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
from testlib.scripts.connections.local import local_steps
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))

#   Mandatory params
carrier_name = "Vodafone RO"

# This script will test that PIN1 is asked when rebooting with PIN enabled - B2B
try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Step1: Power cycle the DUT / Check that DUT boot. Check that the PIN is required
    #   Step2: Enter correct PIN1 / DUT is unlocked and camps on the network
    #   Step3: Turn off the DUT / Dut is off
    #   Step4: Perform steps 1-3 20 times

    telephony_steps.set_sim_pin(serial = serial,
                                state = "ON",
                                pin = telephony_defaults.sim['default_pin'])()

    for i in range(20):
        adb_steps.command(serial = serial,
                         command = "reboot")()
        adb_steps.check_device_reboots(serial = serial,
                                       reboot_timeout = 120)()
        local_steps.wait_for_adb(serial = serial,
                                 reboot_timeout = 120)()
        adb_steps.wait_for_ui(serial = serial,
                              boot_to_Android = False,
                              sim_pin_enabled = True)()
        telephony_steps.check_pin_is_requested(serial = serial,
                                               enabled_pin = True,
                                               with_reboot = False)()
        telephony_steps.wake_up_device_with_sim_pin(serial = serial,
                                                    sim_pin = telephony_defaults.sim['default_pin'])()
        telephony_steps.check_carrier(serial = serial, carrier_name = carrier_name)()

finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()
