#!/usr/bin/env python

#######################################################################
#
# @filename:    test_01_parse_logcat.py
# @description: Checks logcat for SIGSEGV
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.logcat import logcat_steps

import sys

logcat_steps.grep_for(serial = sys.argv[1] + ":5555",
                      port = sys_argv[2],
                      grep_for_text = sys.argv[3],
                      text_presence = False,
                      verbose = True,
                      blocking = False)()
