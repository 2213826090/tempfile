#######################################################################
#
# @filename:    GMS_account_management_Disable_enable_Wifi_BT.py
# @description: Disable wifi and BT
# @author:      dpanday@intel.com
#
######################################################################



#Build in libraries
import sys


# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.wireless.bluetooth import bluetooth_steps
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps

from testlib.scripts.android.ui import ui_steps


# ############# Get parameters ############

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

#setup
    if "scenario" in args.keys():
        scenario = args["scenario"]
    else:
        scenario = ""

    if "iteratios" in args.keys():
        iterations = args["iterations"]
    else:
        iterations = 1
    iterations = int(iterations)


while not ui_steps.remove_user(user_name="New user",optional=True, serial=serial)():
    break


#run

    #setup new user and switch

    ui_steps.press_home(serial=serial)()
    ui_steps.press_car(serial=serial)()
    ui_steps.open_settings(serial=serial)()
    ui_steps.click_button_with_scroll(view_to_find={"text":"Users & accounts"},view_to_check={"text":"Users"},serial=serial)()
    ui_steps.click_button_common(view_to_find={"text":"Users"},view_to_check={"text":"Add user"},serial=serial)()
    ui_steps.click_button(view_to_find={"textContains":"Add user"},view_to_check={
        "text":"OK"},serial=serial)()

    ui_steps.click_button_if_exists(view_to_find={"text":"OK"},view_to_check={"text":"NOT NOW"},
                           serial=serial)()
    ui_steps.click_button(view_to_find={"text":"SET UP NOW"},serial=serial)()
    ui_steps.press_home(serial=serial)()
    ui_steps.press_car(serial=serial)()

# check if BT and wifi is disable for new user as well


if scenario =="Disable":
    ui_steps.press_home(serial=serial)()
    if not bluetooth_steps.SetBT(state='Off', use_gui=True,serial=serial)():
        print "BT is not disable in new user"
    else:
        "BT was already OFF"

    if not wifi_generic_steps.set_wifi(state="OFF", serial = serial)():
        print "wifi is disable  in new user"

        else:
        print "wifi was already turn off"


else:

    wifi_generic_steps.set_wifi(state="ON", serial=serial)()


# check if BT and wifi is enable for new user as well
if scenario == "Enable":
    ui_steps.press_home(serial=serial)()

    if not bluetooth_steps.SetBT(state='ON', use_gui=True, serial=serial)():
        print "BT is not enable in new user"

    else:
        print "BT was already ON"

    if not wifi_generic_steps.set_wifi(state="ON", serial=serial)():
        print "wifi is enable in new user"

    else:
        print "wifi was turn off"


else:

    bluetooth_steps.SetBT(state='ON', use_gui=True,serial=serial)()
    wifi_generic_steps.set_wifi(state="ON", serial=serial)():

# Teardown

while not ui_steps.remove_user(user_name="New user", optional=True, serial=serial)():
    break
ui_steps.press_home(serial=serial)()
bluetooth_steps.SetBT(state='ON', use_gui=True, serial=serial)()
wifi_generic_steps.set_wifi(state="ON", serial=serial)()

