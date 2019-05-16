#!/usr/bin/env python

#######################################################################
#
# @filename:    test_01_dalvik_suite.py
# @description: Runs Dalvik suite
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local.local_step import step as local_step
from testlib.scripts.android.adb import adb_steps

DALVIK_TEST_SUITE_PATH = \
    "/home/sys_spjenkins/work/testlib/resources/ABT_Dalvik/"
DALVIK_SCRIPT = "run_all"

class dalvik_test(local_step):

    def __init__(self, script, name, pass_message = "succeeded!",
            fail_message = "FAILED", **kwargs):
        local_step.__init__(self, **kwargs)
        self.script = script
        self.name = name
        self.pass_message = pass_message
        self.fail_message = fail_message
        self.set_passm("Running Dalvik test " + self.name)
        self.set_errorm("", "Running Dalvik test " + self.name)

    def do(self):
        command = self.script + " " + self.name
        self.so, self.se = local_steps.command(command = command,
                                               stderr_grep = self.pass_message)()

    def check_condition(self):
        if self.pass_message in self.se:
            return True
        if self.fail_message in self.se:
            return False
        print "uepa"

import sys

def main(argv = None):
    if argv is None:
        argv = sys.argv

    argument_list = []
    for arg in sys.argv[4:]:
        argument_list.append(arg)
    dalvik_test(script = sys.argv[3],
                name = sys.argv[4],
                critical = False,
                blocking = False)()

if __name__ == "__main__":
    main()
