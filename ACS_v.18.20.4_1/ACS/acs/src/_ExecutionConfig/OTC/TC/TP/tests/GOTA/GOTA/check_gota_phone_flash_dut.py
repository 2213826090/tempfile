#Copyright (C) 2014  Lan, SamX <samx.lan@intel.com>
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
@summary: test it can phone flash DUT successfully
@since: 12/08/2014
@author: Sam Lan(samx.lan@intel.com)
'''
import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.util.common import g_common_obj
from testlib.util.device import TestDevice

class PhoneFlashDUT(UIATestBase):
    """
    @summary: Test it can phone flash DUT successfully
    """

    def setUp(self):
        super(PhoneFlashDUT, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system=SystemImpl(self.cfg)
        self.apk=ApkInstallUninstallImpl(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PhoneFlashDUT, self).tearDown()
        self.cfg = None

    def testPhoneFlashDUT(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        The workstation installed phone fash tool

        Test Case Step:
        1. test phone flash dut

        Expect Result:
        1. it can phone flash DUT successfully

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()

        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_build=self.cfg.get("base_build")
        host_oem_file=self.cfg.get("host_oem_file")
        client_oem_file=self.cfg.get("client_oem_file")
        gota_test_run_by_testplan=self.cfg.get("gota_test_run_by_testplan")
        tmp_name=time.strftime('%Y-%m-%d_%H:%M:%S',time.localtime(time.time()))
        before_ota_info_file="~/ota/"+"beforeOTA_"+tmp_name+".log"
        self.gota.phone_flash_tool_build_zip_file_3(base_build)
        g_common_obj.root_on_device()
        self.gota.mount_device()
        self.system.adb_push_file(host_oem_file, client_oem_file)
        self.gota.factory_reset()
        self.gota.wait_for_android_os()
        self.gota.setup_connection()
        self.gota.push_uiautomator_jar()
        g_common_obj.set_vertical_screen()
        self.gota.skip_initial_screen_after_flash()
        self.gota.enable_developer_option()
        self.gota.keep_awake()
        self.gota.accept_unknow_resource()
        self.gota.close_lock_screen()
        self.gota.unverify_apps_over_USB()