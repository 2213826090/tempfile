#!/usr/bin/env python

#######################################################################
#
# @filename:    test_i18n_user_can_change_language.py
# @description: Tests if the user can change the OS language from
#                   Settings
# @author:      tudorx.a.simion@intel.com
#######################################################################


import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.i18n import i18n_steps
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

adb_steps.connect_device(
    serial = serial,
    port = adb_server_port
)()
globals().update({"version": adb_utils.get_android_version()})

#This should be used as:
#    toogle_through_languages(desired_language_key_word="xxx", 
#                             desired_original_key_word = "xxx")
#        where:
#              desired_language_key_word should be the country to 
#                    which you want to change the language. Please note
#                    it should be translated into that respective 
#                    language.
#              desired_language_key_word should be the original 
#                    language Country. Please note it should be
#                    translated into that respective language.
i18n_steps.toogle_through_languages(desired_language_key_word =\
                                        "Deutschland",
                                    desired_language_check_word =\
                                        "Sprache", 
                                    desired_original_key_word =\
                                        "United States",
                                    desired_original_check_word =\
                                        "Language")()

ui_steps.press_home()()

