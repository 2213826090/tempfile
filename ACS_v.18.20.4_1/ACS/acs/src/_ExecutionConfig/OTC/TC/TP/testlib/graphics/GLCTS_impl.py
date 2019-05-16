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
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for GLCTS Operation
@since: 01/06/2015
@author: Shuang He
'''
import os
import re
from testlib.util.common import g_common_obj
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)


class TimeoutError(Exception):
    pass


class GLCTSImpl(object):

    ''' Implementation to run GLCTS
    Support to run GLES 2.0/3.0/3.1 GLCTS tests
    '''

    def __init__(self):
        self.device = g_common_obj.get_test_device()
        self._device = g_common_obj.get_device()
        self.base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)

    def check_extension(self, es_version, extension_name):
        """ Check given extension support
        Check if given extension is supported with given es version
        """
        cmd = "shell /data/app/oglconform -minFmt -%s -diag | grep -n '%s:'" % (es_version, extension_name)
        diag_output = g_common_obj.adb_cmd_common(cmd)
        start_line = diag_output.split(':')[0]
        end_line = int(start_line) + 3
        cmd = "shell /data/app/oglconform -minFmt -%s -diag | sed -n '%s,%sp'" % (es_version, start_line, end_line)
        diag_output = g_common_obj.adb_cmd_common(cmd)
        support_check = diag_output.split('%s' % (extension_name + ':'))[1].split(':')[0]
        assert support_check.find('reported') >= 0, "%s not reported" % (extension_name)
        assert support_check.find('supported') >= 0, "%s not supported" % (extension_name)

    def run_case(self, case_name):
        """ Run given test case
        Run given test case with given es version
        """
        cmd = "/data/app/android_standalone_x86/glcts --deqp-base-seed=1 --deqp-surface-width=64 --deqp-surface-height=64 --deqp-case=%s" % (case_name)
        output = self.device.adb_cmd_capture_msg_ext(repr(cmd))
        assert 'Test run totals:' in output, "unexpected results, no summary printed\n%s" % (output)
        summary_msg = output.split("Test run totals:")[1]
        print summary_msg
        summary = [m.groupdict() for m in re.finditer(r"(?P<key>\w+):\s+(?P<value>\d+)", summary_msg)]
        LOG.debug(summary)

        failures = ['failed']
        failed = any([each['key'].lower() in failures and int("0" + each['value']) > 0 for each in summary])

        assert not failed, (case_name, summary_msg)
