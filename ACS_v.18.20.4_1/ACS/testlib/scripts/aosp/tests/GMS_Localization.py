# coding=utf-8
# Build in libraries
import sys

# Used defined libraries
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.wifi import wifi_steps
from uiautomator import device as d
from testlib.scripts.wireless.bluetooth import bluetooth_steps, bt_utils

# ############# Get parameters ############
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args[0].upper() != 'NONE':
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val

# BT turn on
bluetooth_steps.SetBT(state='ON', use_gui=True,serial=serial)()


#Wi-Fi turn on
wifi_steps.set_wifi_state(state="ON",serial=serial)()

# Language Set Up
ui_steps.press_home(serial=serial)()
ui_steps.open_settings(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"System"},view_to_check={"text":"Languages & input"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Languages & input"},view_to_check={"text":"Languages"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Languages"},view_to_check={"text":"Language preferences"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"Add a language"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"हिन्दी"},serial=serial)()
d.drag(967,154,980,245)

# check for Wi-Fi settings
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"सेटिंग"},view_to_check={"text":"नेटवर्क और इंटरनेट"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"नेटवर्क और इंटरनेट"},view_to_check={"text":"वाई-फ़ाई"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"वाई-फ़ाई"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"नेटवर्क जोड़ें"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"वाई-फ़ाई संबंधी प्राथमिकताएं"},serial=serial)()

# check for BT settings
ui_steps.press_home(serial=serial)()
ui_steps.press_car(serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"सेटिंग"},view_to_check={"text":"कनेक्ट किए गए डिवाइस"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"कनेक्ट किए गए डिवाइस"},view_to_check={"text":"ब्लूटूथ"},serial=serial)()
ui_steps.click_button_common(view_to_find={"text":"ब्लूटूथ"},view_to_check={"text":"चालू"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"डिवाइस नाम"},serial=serial)()
ui_steps.wait_for_view_with_scroll(view_to_find={"text":"मिलने वाली फ़ाइलें"},serial=serial)()

# Phone Details(Displays in default language)
ui_steps.press_dialer(serial=serial)()
ui_steps.wait_for_view(view_to_find={"text":"To make or receive calls, pair your phone with the car."},
                           serial=serial)()

# Tear Down
ui_steps.press_car(serial=serial)()
ui_steps.click_button_common(view_to_find={"text": "सेटिंग"}, view_to_check={"text": "सेटिंग में सर्च करें"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"text": "सिस्‍टम"}, view_to_check={"text": "भाषाएं और इनपुट"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"text": "भाषाएं और इनपुट"}, view_to_check={"text": "भाषाएं"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"text": "भाषाएं"}, view_to_check={"text": "भाषा प्राथमिकताएं"},
                             serial=serial)()
ui_steps.click_button_common(view_to_find={"descriptionContains": "ज़्यादा विकल्प"}, view_to_check={"text": "निकालें"}
                             , serial=serial)()
ui_steps.click_button_common(view_to_find={"text": "निकालें"}, view_to_check={"text": "भाषा प्राथमिकताएं"},
                             serial=serial)()
ui_steps.click_checkbox_button(view_to_find={"text": "हिन्दी (भारत)"},
                               state="ON", serial=serial)()
ui_steps.click_button_common(view_to_find={"descriptionContains": "निकालें"},
                             view_to_check={"text": "चयनित भाषाएं निकालें?"}
                             , serial=serial)()
ui_steps.click_button_common(view_to_find={"text": "ठीक है"}, serial=serial)()


#Turn of BT
bluetooth_steps.SetBT(state='OFF', use_gui=True,serial=serial)()

#Turn of Wi-Fi
wifi_steps.set_wifi_state(state="OFF",serial=serial)()






