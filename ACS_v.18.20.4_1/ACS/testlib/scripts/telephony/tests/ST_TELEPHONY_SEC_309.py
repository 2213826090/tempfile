from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.connections.local import local_steps
import sys

### initialization ###
globals().update(vars(get_args(sys.argv)))

# Mandatory params
carrier_name = "Vodafone RO"

# This script will test that PIN1 is asked when setting flight mode off after rebooting in flight mode
try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Ensure PIN is enabled
    telephony_steps.set_sim_pin(serial = serial,
                                state = "ON",
                                pin = telephony_defaults.sim['default_pin'])()

    #   Test: PIN - Enabling PIN - PIN1 is asked when setting flight mode off after rebooting in flight mode
    #   Step 1: Set flight mode on
    wifi_steps.set_airplane_mode(serial=serial, state = "ON")()

    #   Step 2: Reboot DUT
    adb_steps.reboot(serial=serial,
                     reboot_timeout = 200,
                     sim_pin = telephony_defaults.sim['default_pin'],
                     sim_pin_enabled = True,
                     no_ui = True)()

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

    #   Step 3: Allow System Boot completion
    #   Step 4: If PIN1 is requested after step 3, enter correct PIN1
    telephony_steps.wake_up_device_with_sim_pin(serial = serial,
                                                sim_pin = telephony_defaults.sim['default_pin'])()

    #   Step 5: Turn off the flight mode (case a: if PIN1 has already been entered in step 4, then DUT camps on network\
    #   case b: otherwise, PIN1 is requested to user
    wifi_steps.set_airplane_mode(serial=serial, state = "OFF")()
    #   Step 6: If case b has applied at step 5, enter correct PIN1

    telephony_steps.check_carrier(serial = serial, carrier_name = carrier_name)()


finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()