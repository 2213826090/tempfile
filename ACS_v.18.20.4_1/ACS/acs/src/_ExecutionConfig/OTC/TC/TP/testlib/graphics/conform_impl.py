# Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for Conform Operation
@since: 01/06/2015
@author: Shuang He
'''
import os
import time
from testlib.util.common import g_common_obj

class TimeoutError(Exception): pass

class ConformImpl:
    ''' Implementation to run OpenGL ES 1.1 conformance
    Support to run OpenGL ES 1.1 conformance testsuite
    '''

    def __init__(self):
        self._device = g_common_obj.get_device()
        self.base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)

    def run_case(self, case_name):
        """ Run given test case
        Run given test case
        """
        cmd = "shell /data/app/conform -r 32555 -1 %s | grep 'SUMMARY: NO tests failed at any path level'" % (case_name)
        diag_output = g_common_obj.adb_cmd_common(cmd, 3000)
        assert diag_output.find('SUMMARY: NO tests failed at any path level') >= 0, "%s failed" % (case_name)

        cmd = "shell /data/app/conform -p 1 -r 32556 -1 %s | grep 'SUMMARY: NO tests failed at any path level'" % (case_name)
        diag_output = g_common_obj.adb_cmd_common(cmd, 700)
        assert diag_output.find('SUMMARY: NO tests failed at any path level') >= 0, "%s failed" % (case_name)

        cmd = "shell /data/app/conform -p 2 -r 32557 -1 %s | grep 'SUMMARY: NO tests failed at any path level'" % (case_name)
        diag_output = g_common_obj.adb_cmd_common(cmd, 700)
        assert diag_output.find('SUMMARY: NO tests failed at any path level') >= 0, "%s failed" % (case_name)

        cmd = "shell /data/app/conform -p 3 -r 32558 -1 %s | grep 'SUMMARY: NO tests failed at any path level'" % (case_name)
        diag_output = g_common_obj.adb_cmd_common(cmd, 700)
        assert diag_output.find('SUMMARY: NO tests failed at any path level') >= 0, "%s failed" % (case_name)

    def run_covegl(self):
        """ Run covegl tests
        """
        cmd = "shell /data/app/covegl | grep 'covegl passed'"
        diag_output = g_common_obj.adb_cmd_common(cmd)
        assert diag_output.find('covegl passed') >= 0, "run_covegl failed"

    def run_covgl(self):
        """ Run covgl tests
        """
        cmd = "shell /data/app/covgl | grep 'covgl passed'"
        diag_output = g_common_obj.adb_cmd_common(cmd)
        assert diag_output.find('covgl passed') >= 0, "run_covgl failed"

    def run_primtest(self):
        """ Run primetest tests
        """
        cmd = "shell /data/app/primtest | grep 'primtest (single precision) passed'"
        diag_output = g_common_obj.adb_cmd_common(cmd)
        assert diag_output.find('primtest (single precision) passed') >= 0, "run_primtest failed"
