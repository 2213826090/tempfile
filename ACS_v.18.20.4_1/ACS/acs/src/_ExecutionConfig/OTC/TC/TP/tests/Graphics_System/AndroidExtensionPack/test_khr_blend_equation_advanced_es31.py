#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

'''
@summary: run GLBenchmark
@since: 10/08/2014
@author: Grace Yi(gracex.yi@intel.com)
'''
import os
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.binary.binary_impl import BinaryImpl

class AndroidExtensionPackTest(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(AndroidExtensionPackTest, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_android_extension_pack')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        tar_path = cfg.get("tar_path")
        BinaryImpl.push_binary(file_path, tar_path)
        self.cmd_path = tar_path

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(AndroidExtensionPackTest, self).tearDownClass()

    def setUp(self):
        super(AndroidExtensionPackTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.binary.conf'
        self.binary = BinaryImpl(
            self.config.read(cfg_file, "android_extension_pack_khr"))

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AndroidExtensionPackTest, self).tearDown()

    def testKHRBlendEquationAdvanced(self):
        """
        @summary: test_KHR_blend_equation_advanced
        """
        print "[RunTest]: %s" % self.__str__()
        cmd = self.cmd_path + " " + self.binary.cfg.get("running_cmd")
        key = self.binary.cfg.get("key")
        key_result = self.binary.cfg.get("key_result")
        file_name = self.binary.cfg.get("file_name")
        result_name = self.binary.cfg.get("result_name")
        line = self.binary.cfg.get("line")
        file_path = os.path.join(g_common_obj.get_user_log_dir(), file_name)
        result_path = os.path.join(g_common_obj.get_user_log_dir(), result_name)
        self.binary.run_cmd_and_collect_output(cmd, file_path)
        assert self.binary.get_result_paragraph_from_log(
            key, file_path, result_path, line)
        assert self.binary.check_key_in_file(key_result, result_path)
