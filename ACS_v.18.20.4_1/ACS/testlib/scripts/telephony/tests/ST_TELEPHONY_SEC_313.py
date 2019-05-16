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

# This script will check that following reboot, DUT ask or do not ask PIN according to PIN security status

try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Step1: Enable PIN security / PIN is enabled after entering correct PIN1
    #   Step2: Reboot the DUT
    #   Step3: Allow system boot completion / Check that the DUT display the input screen for PIN1
    #   Step4: Enter correct PIN1 / DUT is unlocked and camps on network
    #   Step5: Disable PIN security / PIN is disabled after entering correct PIN1
    #   Step6: Reboot the DUT
    #   Step7: Allow system boot completion / Check that the DUT is unlocked and camps on the network, without PIN query
    #   Step8: Perform steps 1-7 20 times

    for i in range(20):
        telephony_steps.set_sim_pin(serial = serial,
                                state = "ON",
                                pin = telephony_defaults.sim['default_pin'])()

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

        telephony_steps.set_sim_pin(serial = serial,
                                state = "OFF",
                                pin = telephony_defaults.sim['default_pin'])()

        adb_steps.command(serial = serial,
                         command = "reboot")()
        adb_steps.check_device_reboots(serial = serial,
                                       reboot_timeout = 120)()
        local_steps.wait_for_adb(serial = serial,
                                 reboot_timeout = 120)()
        adb_steps.wait_for_ui(serial = serial,
                              boot_to_Android = False,
                              sim_pin_enabled = False)()
        telephony_steps.check_pin_is_requested(serial = serial,
                                               enabled_pin = False,
                                               with_reboot = False)()

        telephony_steps.check_carrier(serial = serial, carrier_name = carrier_name)()

finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()
