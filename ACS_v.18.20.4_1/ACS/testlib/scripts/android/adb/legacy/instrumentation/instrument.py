#!/usr/bin/env python

#######################################################################
#
# @filename:    instrumentation_runner.py
# @description: Instrumentation runner
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android import android_utils

class InstrumentTest(adb_step):

    """ description:
        runs an instrumentation test:
        - creates "am instrument" command with <argument_list>
        (-e arg1 val1 ... -e arg2 val2) waiting for script to finish
        (-w script)
        - runs the test
        - checks the output for pass message ("." default)

        usage:
        InstrumentTest(serial = "10.237.100.212:5555",
                       port = 17002,
                       argument_list = "-e class
com.android.music.tests.functional.TestPlaylist#testDeletePlaylist",
                       script =
"com.android.music.tests/android.test.InstrumentationTestRunner",
                       timeout = 100)()

        tags:
            android, instrument, instrumentation, runner
    """

    def __init__(self, argument_list = None, script = None, pass_message = ".",
            **kwargs):
        adb_step.__init__(self, **kwargs)
        self.am_command = "am instrument "
        if argument_list:
            for arg in argument_list:
                self.am_command += arg + " "
        if script:
            self.am_command += " -w " + script
        self.script = script
        self.pass_message = pass_message
        self.set_passm("Running instrumentation test " + self.am_command)

    def do(self):
        self.step_data =\
            self.adb_connection.parse_cmd_output(cmd = self.am_command,
                                                 grep_for = self.pass_message)

    def check_condition(self):
        print "{" + self.step_data +  "}"
        test_runner = self.script.split(".")[-1]
        return "Test results for " + test_runner + "="\
                + self.pass_message in self.step_data


import sys

def main(argv = None):
    if argv is None:
        argv = sys.argv

    argument_list = []
    for arg in sys.argv[4:]:
        argument_list.append(arg)
    InstrumentTest(serial = sys.argv[1],
                   port = sys.argv[2],
                   script = sys.argv[3],
                   argument_list = argument_list,
                   critical = False,
                   blocking = False)()

if __name__ == "__main__":
    main()
