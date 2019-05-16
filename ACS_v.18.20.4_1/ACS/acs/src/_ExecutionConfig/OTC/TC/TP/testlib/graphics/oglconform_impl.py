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
@since: 01/06/2015
@author: Shuang He
'''
import os
import re
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import osversion
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)


class TimeoutError(Exception):
    pass


class OglconformImpl:

    ''' Implementation to run Oglconform
    Support to run Oglconform
    '''

    def __init__(self):
        self.device = g_common_obj.get_test_device()
        self._device = g_common_obj.get_device()
        self.base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)
        chk_64_supported = "getprop | grep ro.product.cpu.abi\]"
        result = g_common_obj.adb_cmd_capture_msg(chk_64_supported)
        self.exec_bin = "/data/app/oglconform" if "64" in result else "/data/app/oglconform_x86"

    def check_extension(self, es_version, extension_name):
        """ Check given extension support
        Check if given extension is supported with given es version
        """
        cmd = "shell \"%s -minFmt -%s -diag | grep -n '%s:'\"" % (self.exec_bin, es_version, extension_name)
        diag_output = g_common_obj.adb_cmd_common(cmd)
        start_line = diag_output.split(':')[0]
        end_line = int(start_line) + 3
        cmd = "shell \"%s -minFmt -%s -diag | sed -n '%s,%sp'\"" % (self.exec_bin, es_version, start_line, end_line)
        diag_output = g_common_obj.adb_cmd_common(cmd)
        support_check = diag_output.split('%s' % (extension_name + ':'))[1].split(':')[0]
        assert support_check.find('reported') >= 0, "%s not reported" % (extension_name)
        assert support_check.find('supported') >= 0, "%s not supported" % (extension_name)

    def get_dirs_libGL(self):
        cmd = 'cd /system/ && find /system/ -name "libGLES_*.so"; cd /system/vendor/ && find /system/vendor/ -name "libGLES_*.so"'
        output = self.device.adb_cmd_capture_msg_ext(repr(cmd))
        print "=" * 60
        print output
        print "=" * 60
        files = re.findall(r".+\.so", output)
        assert len(files) > 0, "Not found libGLES_*.so in /system"
        dirs = [os.path.dirname(each) for each in files if 'egl' in each]
        dirs = list(set(dirs))
        assert len(dirs) > 0, "Not found libGLES_*.so in egl subfoler"
        return dirs

    def check_extension_libGL(self, es_version, extension_name):
        """ Check given extension support
        Check if given extension is supported with given es version
        """
        dirs = self.get_dirs_libGL()

        test_result = False
        results = []
        for so_dir in dirs:
            result = {'cmd': '', 'error': '', 'section_msg': ''}
            if "64" in so_dir:
                test_cmd = "/data/app/oglconform -minFmt -%s -diag -libGL %s/libGLES_*.so; echo exit_code=$?" % (
                    es_version, so_dir)
            else:
                test_cmd = "/data/app/oglconform_x86 -minFmt -%s -diag -libGL %s/libGLES_*.so; echo exit_code=$?" % (
                    es_version, so_dir)
            result['cmd'] = test_cmd
            test_output = self.device.adb_cmd_capture_msg_ext(repr(test_cmd), time_out=600)
            # print ">>>>>>>>>>>>>test output: %s" % test_output
            if 'exit_code=0' not in test_output:
                result['error'] = test_output
                results.append(result)
                continue

            pattern = r"%s:.+?\n.+?\n.+?\n" % (extension_name)
            match = re.search(pattern, test_output, re.MULTILINE)
            if match:
                section_msg = match.group()
                result['section_msg'] = section_msg
                print "=" * 60
                print section_msg
                print "=" * 60
                if 'supported' in section_msg and 'reported' in section_msg:
                    test_result = True
            else:
                result['section_msg'] = 'Not found %s' % (pattern)
            results.append(result)
            if test_result == True:
                break

        assert test_result == True, '\n'.join([str(each) for each in results])

    def run_case(self, es_version, case_name, sub_case_name=""):
        """ Run given test case
        Run given test case with given es version
        """
        cmd = "%s -v 4 -minFmt -%s -test %s %s" % (self.exec_bin, es_version, case_name, sub_case_name)
        diag_output = self.device.adb_cmd_capture_msg_ext(repr(cmd), time_out=600)
        summary = [m.groupdict() for m in re.finditer(r"(?P<key>Total [\w+\s]+):\s+(?P<value>\w+)", diag_output)]
        LOG.debug(summary)
        assert diag_output.find('Total Failed : 0') >= 0, ("%s failed" % (case_name))

    def run_case_libGL(self, es_version, case_name, sub_case_name=""):
        """ Run given test case
        Run given test case with given es version
        """
        dirs = self.get_dirs_libGL()

        test_result = False
        results = []
        for so_dir in dirs:
            result = {'cmd': '', 'error': '', 'summary': ''}
            test_cmd = "%s -v 4 -minFmt -libGL %s/libGLES_*.so -%s -test %s %s; echo exit_code=$?" %\
                       (self.exec_bin, so_dir, es_version, case_name, sub_case_name)
            result['cmd'] = test_cmd
            test_output = self.device.adb_cmd_capture_msg_ext(repr(test_cmd), time_out=600)
            if 'exit_code=0' not in test_output:
                result['error'] = test_output
                results.append(result)
                continue

            summary = [m.groupdict() for m in re.finditer(r"(?P<key>Total [\w+\s]+):\s+(?P<value>\w+)", test_output)]
            LOG.debug(summary)

            result['summary'] = summary

            if 'Total Failed : 0' in test_output:
                test_result = True

            results.append(result)
            if test_result == True:
                break

        assert test_result == True, '\n'.join([str(each) for each in results])

    def run_case_egl_config(self):
        """ Run given test case
        Run given test case with given es version
        """
        cmd = "%s -v 4 -minFmt -es2 -testList /data/app/egl-config" % self.exec_bin
        diag_output = self.device.adb_cmd_capture_msg_ext(repr(cmd), time_out=600)
        summary = [m.groupdict() for m in re.finditer(r"(?P<key>Total [\w+\s]+):\s+(?P<value>\w+)", diag_output)]
        LOG.debug(summary)
        assert diag_output.find('Total Failed : 0') >= 0, "Failed test_EGL_1_4_egl_config"

    def run_case_program_binary_es3(self):
        """ Run given test case
        Run given test case with given es version
        """
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            cmd = "shell %s -v 4 -minFmt -libGL /system/vendor/lib/egl/libGLES_*.so -es3 -testList " \
                  "/data/app/get-program-binary_es3 | grep 'Total Failed : 0'" % self.exec_bin
        elif androidversion == 6:
            print "osversion is M"
            cmd = "shell %s -v 4 -minFmt -libGL /system/vendor/lib/egl/libGLES_*.so -es3 -testList " \
                  "/data/app/get-program-binary_es3 | grep 'Total Failed : 0'" % self.exec_bin
        elif androidversion == 5:
            print "osversion is L"
            cmd = "shell %s -v 4 -minFmt -libGL /system/lib/egl/libGLES_intel*.so -es3 -testList " \
                  "/data/app/get-program-binary_es3 | grep 'Total Failed : 0'" % self.exec_bin
        else:
            print "osversion is %s" % (androidversion)
            cmd = "shell %s -v 4 -minFmt -libGL /system/lib/egl/libGLES_intel*.so -es3 -testList " \
                  "/data/app/get-program-binary_es3 | grep 'Total Failed : 0'" % self.exec_bin
        diag_output = g_common_obj.adb_cmd_common(cmd, 600)
        assert diag_output.find('Total Failed : 0') >= 0, "run_case_program_binary_es3 failed"

    def check_extension_with_verbosity_level(self, verbosity_level, es_version, extension_name):
        """ Check given extension support
        Check if given extension is supported with given es version
        """
        cmd = "shell \"%s -v %d -minFmt -%s -diag | grep -n '%s:'\""\
              % (self.exec_bin, int(verbosity_level), es_version, extension_name)
        start_line = g_common_obj.adb_cmd_common(cmd).split(':')[0]
        end_line = int(start_line) + 3
        cmd = "shell \"%s -v %d -minFmt -%s -diag | sed -n '%s,%sp'\""\
              % (self.exec_bin, int(verbosity_level), es_version, start_line, end_line)
        diag_output = g_common_obj.adb_cmd_common(cmd)
        support_check = diag_output.split('%s' % (extension_name + ':'))[1].split(':')[0]
        assert support_check.find('reported') >= 0, "%s not reported" % extension_name
        assert support_check.find('supported') >= 0, "%s not supported" % extension_name

    def check_extension_with_verbosity_level_get_result(self, verbosity_level, es_version, extension_name):
        """
        This method is aim to judge final result for running multiple cases.
        """
        cmd = "shell \"%s -v %d -minFmt -%s -diag | grep -n '%s:'\""\
              % (self.exec_bin, int(verbosity_level), es_version, extension_name)
        start_line = g_common_obj.adb_cmd_common(cmd).split(':')[0]
        end_line = int(start_line) + 3
        cmd = "shell \"%s -v %d -minFmt -%s -diag | sed -n '%s,%sp'\""\
              % (self.exec_bin, int(verbosity_level), es_version, start_line, end_line)
        diag_output = g_common_obj.adb_cmd_common(cmd)
        support_check = diag_output.split('%s' % (extension_name + ':'))[1].split(':')[0]
        if (support_check.find('reported') >= 0) *\
                (support_check.find('supported') >= 0):
            return [extension_name, 'PASS']
        else:
            return [extension_name, 'FAIL']
