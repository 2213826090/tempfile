# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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
@summary: Class for ChromeCast operation
@since: 03/30/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import osversion


class GrallocImpl:

    '''
    classdocs
    '''

    def init_environment(self):
        """ Init the test environment
        """
        self.config = TestConfig()
        self.cfg_file = 'tests.tablet.gralloc.conf'
        cfg_arti = self.config.read(self.cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')

        for each in ['binary', 'binary_m', 'binary_n']:
            cfg = self.config.read(self.cfg_file, each)
            arti = Artifactory(cfg_arti.get('location'))
            binary_name = cfg.get("name")
            file_path = arti.get(binary_name)
            print "%s" % file_path
            g_common_obj.adb_cmd_common('push ' + file_path + ' /data/app/')
            g_common_obj.adb_cmd('chmod 777 /data/app/gralloc_test_val*')

    def run_case(self, case):
        """ run the binary file and check the result
        """
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        cfg = self.config.read(self.cfg_file, case)
        cmd = str(cfg.get('cmd'))
        if androidversion == 7:
            print "osversion is N"
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val_n ' + cmd + "| grep OK"
            ), "The test failed"
        elif androidversion == 6:
            print "osversion is M"
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val_m ' + cmd + "| grep OK"
            ), "The test failed"
        elif androidversion == 5:
            print "osversion is L"
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val ' + cmd + "| grep OK"
            ), "The test failed"
        else:
            print "osversion is %s" % (androidversion)
            output = g_common_obj.adb_cmd_capture_msg('/data/app/gralloc_test_val_m ' + cmd)
            print "=" * 60
            print output
            print "=" * 60
            assert "OK" in output, "The test failed"

    def run_case_special(self, case):
        """ run the binary file and check the result
        """
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        cfg = self.config.read(self.cfg_file, case)
        cmd = str(cfg.get('cmd'))
        if androidversion == 7:
            print "osversion is N"
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val_m | grep ' + cmd + "| grep OK"
            ), "The test failed"
        elif androidversion == 6:
            print "osversion is M"
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val_m | grep ' + cmd + "| grep OK"
            ), "The test failed"
        elif androidversion == 5:
            print "osversion is L"
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val | grep ' + cmd + "| grep OK"
            ), "The test failed"
        else:
            print "osversion is %s" % (androidversion)
            assert g_common_obj.adb_cmd_capture_msg(
                '/data/app/gralloc_test_val_m | grep ' + cmd + "| grep OK"
            ), "The test failed"
