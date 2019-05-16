# -*- coding: utf-8 -*-


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
@summary: Class for Oglconform Operation
@since: 12/01/2015
@author: Zhao Xiangyi
'''

import os
import time
import re
import tempfile
from subprocess import Popen, PIPE
from testlib.util.log import Logger
from testlib.util.common import g_common_obj
from testlib.common.base import getTmpDir
from testlib.graphics.common import pkgmgr, get_current_focus_window, get_resource_from_atifactory
from testlib.androidframework.common import EnvironmentUtils


LOG = Logger.getlogger(__name__)
PACKAGENAME = "com.drawelements.deqp"

class TimeoutError(Exception):
    pass


class dEQPImpl(object):

    def __init__(self):
        self._device = g_common_obj.get_device()
        self.tmp = tempfile.gettempdir()
        self.tmpdir = getTmpDir()
        self.inside_output = "/sdcard/TestResult.qpa"
        self.device = g_common_obj.get_test_device()
        self.env = EnvironmentUtils()
        self._mustpass_path = '/sdcard'
        self._mustpass_list = ['vk-master.txt','egl-master.txt','gles2-master.txt','gles3-master.txt','gles31-master.txt']
        self._failure_log = os.path.join(g_common_obj.globalcontext.user_log_dir, 'deqp_failures.log')
        self._raw_deqp_result = os.path.join(g_common_obj.globalcontext.user_log_dir, 'deqp_test_result.log')
        self._extension_list = os.path.join(g_common_obj.globalcontext.user_log_dir, 'extension_list.log')

    def setup(self):
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        g_common_obj.adb_cmd_capture_msg("logcat -c")
        self._check_dependency()

    def run_case(self, case_name, check_extensions=False, extension_name=''):

        _inside_output = self.inside_output
        # Add inside function for further check extension tests.
        def _get_extension_list():
            inside_content = g_common_obj.adb_cmd_capture_msg('cat %s' % _inside_output)
            return re.findall(r'\ +?<Text>(.+)</Text>', inside_content)
        # Start run deqp test.
        _results = {'Passed': 0, 'Failed': 0, 'Not supported': 0, 'Warnings': 0}
        LOG.debug("Testcase: %s" % (case_name))
        setlog_cmd = "setprop log.tag.dEQP DEBUG"
        cmd = "am start -S -n com.drawelements.deqp/android.app.NativeActivity -e cmdLine \"" \
              "deqp --deqp-log-filename=%s --deqp-case=%s\"" % (self.inside_output, case_name)
        self.device.adb_cmd_capture_msg_ext(repr(setlog_cmd))
        self.device.adb_cmd_capture_msg_ext(repr(cmd))
        # failures = ['Fail', 'ResourceError', 'Crash', 'Timeout', 'InternalError']
        s_time = time.time()
        while time.time() - s_time < 900:
            cur_window = get_current_focus_window()
            time.sleep(2)
            if PACKAGENAME not in cur_window:
                LOG.debug("Test finished.")
                # Save deqp results to log.
                g_common_obj.adb_cmd_capture_msg('logcat -d -s dEQP > %s' % self._raw_deqp_result)
                # Handle results.
                logs = Popen('cat %s' % self._raw_deqp_result, shell=True, stdout=PIPE, stderr=PIPE).communicate()[0]
                for i in _results.keys():
                    chk_log = r"logcat -d -s dEQP | grep -oe '%s:\ \+[0-9]\+/[0-9]\+'" % i
                    try:
                        output = g_common_obj.adb_cmd_capture_msg(chk_log)
                        num = output.split(' ')[-1].split('/')[0]
                        _results[i] = int(num)
                    except ValueError as e:
                        raise Exception("Got error when running tests: %s" % e)

                assert sum(_results.values()) != 0, LOG.debug("Test not run.")

                LOG.debug("Raw summary: %s"  % _results)
                if _results['Failed'] > 0:
                    raw_failures = re.findall(r'Test case .(dEQP-.*[\d|\w]).+\n.+?dEQP +: +Fail.+?.', logs)
                    head_name = raw_failures[0].split('.')[0].split('-')[-1].lower()
                    real_failures = []
                    for i in raw_failures:
                        _is_mustpass = "cat %s/%s-master.txt | grep -I \'%s\'" % (self._mustpass_path, head_name, i)
                        chk = g_common_obj.adb_cmd_capture_msg(_is_mustpass)
                        if chk == '':
                            _results['Failed'] -= 1
                            _results['Not supported'] += 1
                        else:
                            real_failures.append(i)
                    if len(real_failures) > 0:
                        f = open(self._failure_log, 'w')
                        for i in real_failures:
                            f.write(i + '\n')
                        f.close()
                    LOG.debug("Final summary: %s" % _results)
                # Handle event for extension list check.
                if check_extensions:
                    ext_list = _get_extension_list()
                    f = open(self._extension_list, 'w') # Save extension list to logfile.
                    for el in ext_list:
                        f.write(el + '\n')
                    f.close()
                    LOG.info("Extension list saved in: %s" % self._extension_list)
                    assert extension_name in ext_list, "%s is not in this test." % extension_name
                    LOG.info("%s is found in this test." % extension_name)
                    return True
                else:
                    if _results['Failed'] == 0:
                        LOG.info("All tests passed.")
                        return True
                    else:
                        raise Exception("dEQP test failed, details refer to log file: %s" % self._failure_log)
            else:
                time.sleep(3)
        raise Exception("Test timeout.")

    def _check_dependency(self):

        if not pkgmgr._package_installed(pkgName=PACKAGENAME):
            android_version = self.env.get_android_version()
            selection = {'L': 'apk_l',
                         'M': 'apk_m',
                         'N': 'apk_n',
                         'O-MR0': 'apk_omr0',
                         'O-MR1': 'apk_omr1'}
            try:
                file_path = get_resource_from_atifactory('tests.common.dEQP.conf', 'dEQP', selection[android_version])
                pkgmgr.apk_install(file_path)
            except KeyError, e:
                print "DEQP app version not match with android os: %s" % str(e)
            # Need to update above when new version is released.
        for i in self._mustpass_list:
            op = g_common_obj.adb_cmd_capture_msg('ls %s/%s' %(self._mustpass_path, i))
            if 'No such file' in op:
                file_path = get_resource_from_atifactory('tests.common.dEQP.conf', 'dEQP',
                                                         i.split('-')[0] + '_mustpass')
                g_common_obj.push_file(file_path, self._mustpass_path + '/')

deqp_impl = dEQPImpl()