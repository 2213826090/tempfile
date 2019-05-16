#!/usr/bin/env python

##############################################################################
#
# @filename:
#
# @description: Settings/Language and input
#
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui.settings import steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.utils.ui.uiandroid import UIDevice as ui_device

import sys
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()
globals().update({"version": adb_utils.get_android_version()})

ui_steps.press_home()()

ui_steps.open_settings(print_error = "Error - Settings page was not "
                                     "displayed")()

ui_steps.open_app_from_settings(print_error = "Failed to open Language & input",
                                view_to_find = {"text":"Language & input"},
                                wait_time = 5000,
                                view_to_check = {"textContains":"Spell checker"})()

#change language
ui_steps.click_button(print_error = "Failed to open Language - hey, I might "
                                    "have used a language that's now removed",
                      view_to_find = {"text":"Language"},
                      view_to_check = {"text":"Afrikaans"},
                      critical = False)()
ui_steps.click_button_with_scroll(view_to_find = {"text":"Italiano (Italia)"},
                                  wait = 5000,
                                  critical = False)()

ui_device()(text = "Lingua").wait.exists(timeout = 20000)

#restore English
ui_steps.click_button(print_error = "Failed to open Language "
                                    "- hey, I might have used a "
                                    "language that's now removed",
                      view_to_find = {"text":"Lingua"},
                      view_to_check = {"textContains":"Afrikaans"},
                      critical = False)()

ui_steps.click_button_with_scroll(view_to_find = {"text":
                                                 "English (United States)"},
                                 critical = False)()
ui_device()(text = "Language").wait.exists(timeout = 20000)


#spell checker: toggle it twice in order to leave it in the initial
#state

if version == "L":
    ui_steps.click_button(view_to_find = {"text":"Spell checker"})()
    if ui_device()(text="Off").exists:
        ui_steps.click_button(view_to_find = {"text":"Off"})()
        ui_steps.click_button(view_to_find = {"text":"On"})()
        ui_steps.click_button(view_to_find = {"text":"Spell checker"})()

    if ui_device()(text="On").exists:
        ui_steps.click_button(view_to_find = {"text":"On"})()
        ui_steps.click_button(view_to_find = {"text":"Off"})()
        ui_steps.click_button(view_to_find = {"descriptionContains":"Navigate up"})()


elif version == "K":
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Spell checker"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Spell checker"})()
    #Google Pinyin Input: toggle it twice in order to leave it in the
    #initial state
    steps.click_checkbox_button_with_ok(view_to_find = {"textContains":
                                                        "Google Pinyin"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"textContains":
                                                        "Google Pinyin"})()

    #google voice: toggle it twice in order to leave it in the initial
    #state
    steps.click_checkbox_button_with_ok(view_to_find = {"text":
                                                    "Google voice typing"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"text":
                                                    "Google voice typing"})()

    #Japanese IME: toggle it twice in order to leave it in the initial
    #state
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Japanese IME"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Japanese IME"})()



    #Auto-replace: toggle it twice in order to leave it in the initial
    #state
    ui_utils.is_text_visible("Auto-replace")
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Auto-replace"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Auto-replace"})()

    #Auto-capitalization: toggle it twice in order to leave it in the
    #initial state
    steps.click_checkbox_button_with_ok(view_to_find = {"text":
                                                        "Auto-capitalization"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"text":
                                                        "Auto-capitalization"})()

    #Auto-punctuate: toggle it twice in order to leave it in the initial
    #state
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Auto-punctuate"})()
    steps.click_checkbox_button_with_ok(view_to_find = {"text":"Auto-punctuate"})()

    #open Voice Search
    ui_steps.click_button(print_error = "Failed to open Voice Search",
                          view_to_find = {"text":"Voice Search"},
                          view_to_check = {"text":"Speech output"})()
    ui_steps.press_back(view_to_check = {"text":"Voice Search"})()

ui_steps.press_home()()

