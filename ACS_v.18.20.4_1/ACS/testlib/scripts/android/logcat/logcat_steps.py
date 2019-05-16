#!/usr/bin/env python

#######################################################################
#
# @filename:    logcat_steps.py
# @description: Logcat test steps
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.android.adb.adb_step import step as adb_step

import time


class clear_logcat(adb_step):

    """ description:
            clears logcat

        usage:
            clear_logcat()()

        tags:
            logcat, android, clear
    """

    def do(self):
        self.adb_connection.clear_logcat()

    def check_condition(self):
        return True

class get_logcat(adb_step):

    """ description:
            gets logcat

        usage:
            get_logcat()()

        tags:
            logcat, android, clear
    """

    def do(self):
        self.step_data = self.adb_connection.parse_logcat()

    def check_condition(self):
        return True


class get_dmesg(adb_step):

    """ description:
            gets dmesg

        usage:
            get_dmesg()()

        tags:
            logcat, android, clear
    """

    def do(self):
        self.step_data = self.adb_connection.parse_dmesg()

    def check_condition(self):
        return True


class grep_for(adb_step):

    """ description:
            checks if logcat contains <grep_for_text>
            if <text_presence> is False it will check the absence of the
            given text

        usage:
            grep_for(grep_for_text = "SIGSEGV", text_presence = False)

        tags:
            logcat, android, grep
    """

    def __init__(self, grep_for_text, text_presence = True, **kwargs):
        adb_step.__init__(self, **kwargs)
        self.grep_for_text = grep_for_text
        self.text_presence = text_presence
        if self.text_presence:
            self.set_errorm("", "Logcat does not contain {0}".format(self.grep_for_text))
            self.set_passm("Logcat contains {0}".format(self.grep_for_text))
        else:
            self.set_errorm("", "Logcat contains {0}".format(self.grep_for_text))
            self.set_passm("Logcat does not contain {0}".format(self.grep_for_text))

    def do(self):
        self.exists = self.grep_for_text in\
            self.adb_connection.parse_logcat(grep_for = self.grep_for_text)

    def check_condition(self):
        return self.exists if self.text_presence else not self.exists
