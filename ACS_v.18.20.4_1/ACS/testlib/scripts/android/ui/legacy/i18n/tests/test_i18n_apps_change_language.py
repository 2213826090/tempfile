#!/usr/bin/env python

#######################################################################
#
# @filename:    test_i18n_apps_change_language.py
# @description: Tests if the user can change the OS language from
#                   settings and the impact is seen accross all
#                   the applications across the system
# @author:      tudorx.a.simion@intel.com
#
#######################################################################


import sys
from testlib.utils.atf import get_parms

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

#Parameter relevance:
#  desired_language_key_word = The language to which the switch is done
#  desired_os_key_word = A key word in Settings that is translated in the
#    desired language
#  desired_os_language_translation = the word "Language" translated in the
#    target language  
i18n_steps.check_desired_language(desired_language_key_word = "France", 
                                  desired_os_key_word = "Localisation",
                                  desired_os_language_translation = "Langue",
                                  language_section = "United States",
                                  print_error = "Error - Desired language "
                                                "switch failed")()

#Parameter relevance:
#  desired_language_key_word = The language to which the switch is done
#  desired_os_key_word = the word "Language" translated in the
#    target language
#  original_country = the language that was present on the system when the
#    test was lanuched
i18n_steps.return_to_default_language(desired_language_key_word = "United "
                                                                  "States", 
                                      desired_os_key_word = "Langue", 
                                      original_country = "France", 
                                      print_error = "Error - Desired language "
                                                    "switch failed")()
ui_steps.press_home()()

