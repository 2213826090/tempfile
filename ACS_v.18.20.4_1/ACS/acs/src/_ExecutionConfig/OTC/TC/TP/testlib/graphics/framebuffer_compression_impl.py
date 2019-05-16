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
@summary: Class for FBC Operation
@since: 06/13/2017
@author: rui
'''
import time
from testlib.util.log import Logger
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.graphics.common import adb32
from testlib.androidframework.common import EnvironmentUtils


LOG = Logger.getlogger(__name__)


class FBCImpl:

    '''
    Implementation to run framebuffer compression tests
    '''

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.android_version = EnvironmentUtils().get_android_version()
        self.dut_binary_path = "/data/app"
        self.dut_lib_path = "/system/lib64"
        self.dut_vendor_lib_path = "/vendor/lib64"
        self.config = "tests.tablet.artifactory.conf"
        self.tag = "framebuffer_dependency_m" if self.android_version == "M" else "framebuffer_dependency_o"
        self.arti = Artifactory(TestConfig().read(section='artifactory').get('location'))
        self.remote_bin_path = TestConfig().read(self.config, self.tag).get("binary_kms")
        self.remote_libcairo_path = TestConfig().read(self.config, self.tag).get("libcairo")
        self.remote_libft2_path = TestConfig().read(self.config, self.tag).get("libft2")
        try:
            self.remote_libkmod_path = TestConfig().read(self.config, self.tag).get("libkmod")
            self.remote_libpciaccess_path = TestConfig().read(self.config, self.tag).get("libpciaccess")
        except:
            pass

    def get_binary_file_name(self):
        return self.remote_bin_path.split("/")[-1]

    # def get_lib_file_name(self):
    #     libcairo = self.remote_libcairo_path.split("/")[-1]
    #     libft2 = self.remote_libft2_path.split("/")[-1]
    #     libkmod = self.remote_libkmod_path.split("/")[-1]
    #     libpciaccess = self.remote_libpciaccess_path.split("/")[-1]
    #     return libcairo, libft2, libkmod, libpciaccess

    def push_dependency(self):
        '''
        Push libs and binary to device.
        '''
        bin_file = self.get_binary_file_name()
        # Push binary
        g_common_obj.push_file(self.arti.get(self.remote_bin_path), self.dut_binary_path)
        g_common_obj.adb_cmd_common("shell chmod 755 %s/%s" % (self.dut_binary_path, bin_file))
        # Push must libs
        g_common_obj.push_file(self.arti.get(self.remote_libcairo_path), self.dut_lib_path)
        g_common_obj.push_file(self.arti.get(self.remote_libft2_path), self.dut_lib_path)
        # Push optional libs
        try:
            g_common_obj.push_file(self.arti.get(self.remote_libkmod_path), self.dut_vendor_lib_path)
            g_common_obj.push_file(self.arti.get(self.remote_libpciaccess_path), self.dut_vendor_lib_path)
        except:
            pass

    def run_test(self, subtest=''):
        self.push_dependency()
        bin_file = self.get_binary_file_name()
        LOG.debug("Dependency is ready.\nStart subtest %s" % subtest)
        time.sleep(2)
        adb32.adb_shell_stop()
        output = g_common_obj.adb_cmd_capture_msg("%s/%s --run-subtest %s | tail -1" % \
                                                (self.dut_binary_path, bin_file, subtest))
        print output
        adb32.adb_shell_start()
        assert 'SUCCESS' in output, "Subtest: %s FAILED!" % subtest
