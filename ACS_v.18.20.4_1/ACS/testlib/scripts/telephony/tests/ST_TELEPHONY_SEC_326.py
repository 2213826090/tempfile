from testlib.scripts.android.adb import adb_steps
from testlib.scripts.telephony import telephony_steps
from testlib.scripts.android.ui import ui_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import telephony_defaults
from testlib.scripts.wireless.wifi import wifi_steps
import sys, time

### initialization ###
globals().update(vars(get_args(sys.argv)))

# Mandatory params
carrier_name = "Vodafone RO"
puk_code = "82018884"
wrong_pin = "0000"
sim_pin = telephony_defaults.sim['default_pin']
mmi_unlock_code = "*" + "*05*" + puk_code + "*" + sim_pin + "*" + sim_pin + "#"


# This script will test that the DUT locked due to incorrect SIM PIN1 can be unlocked using appropriate MMI code

try:
    #   Setup part
    adb_steps.wake_up_device(serial=serial)()
    adb_steps.menu_to_unlock(serial=serial)()
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()

    #   Ensure PIN1 is enabled
    telephony_steps.set_sim_pin(serial = serial,
                                state = "ON",
                                pin = telephony_defaults.sim['default_pin'])()

    #   Test:   Unlocking SIM PIN1 entering the PUK thru MMI code
    #   Step 1: Power up the DUT
    #   Step 2: Allow system to boot
    #   Step 3: Enter incorrect PIN1 3 times (until system locks)
    for i in range(3):
        telephony_steps.enter_wrong_old_pin(serial = serial,
                                        wrong_old_pin = wrong_pin,
                                        new_pin = telephony_defaults.sim['default_pin'])()

    #   Step 4: Use phone dialing screen to enter PUK: **05*PIN_UNBLOCKING_KEY*NEW_PIN*NEW_PIN#
    ui_steps.click_button(serial = serial,
                          view_to_find = {"resourceId":"com.android.systemui:id/emergency_call_button"},
                          view_to_check = {'resourceId':"com.android.phone:id/floating_action_button"})()

    telephony_steps.enter_puk_with_mmi_code(serial = serial,
                                            puk_code = puk_code,
                                            pin_code = sim_pin)()
    ui_steps.unlock_device_swipe(serial = serial)()

    #   Step 5: Check DUT's screen state
    #   Step 6: Check network indicator
    telephony_steps.check_carrier(serial = serial,
                                  carrier_name = carrier_name,
                                  wait_time = 10000)()
    ui_steps.unlock_device_swipe(serial = serial)()

finally:
    #   TearDown part
    ui_steps.close_all_app_from_recent(serial=serial)()
    ui_steps.press_home(serial=serial)()