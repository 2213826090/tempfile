#Copyright (C) 2015 haley.han
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
@summary: test apk data unerase after GOTA update
@since: 10/3/2015
@author: haley.han(yi.a.han@intel.com)
'''
import os
import sys
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.systemui.systemui_impl import SystemUI
from testlib.system.system_impl import SystemImpl
from testlib.gota.gota_impl import gotaImpl
class ApkDataUneraseAfterGOTA(UIATestBase):
    """
    @summary: test apk data unerase after GOTA update
    """

    def setUp(self):
        super(ApkDataUneraseAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self._test_name = __name__
        self.cfg = self.config.read(cfg_file, 'gota')
	self.d=g_common_obj.get_device()
        self.serial=g_common_obj2.getSerialNumber()
	self.system=SystemImpl(self.cfg)
        self.gota= gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ApkDataUneraseAfterGOTA, self).tearDown()
        self.cfg = None

    def testApkDataUneraseAfterGOTA(self):
        """
        This test case is to test test apk data unerase after GOTA update

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
	1. run youtube
        1. process GOTA update
	2. test youtube's data after GOTA update

        Expect Result:
        1. youtube's data exist after GOTA

        """

        print "[RunTest]: %s" % self.__str__()
	#os.system("adb -s %s shell rm -rf /sdcard/android/data/com.google.android.youtube" %self.serial)
	#self.d.press.home()
	#exist=self.system.file_exists("/sdcard/android/data/com.google.android.youtube","DEVICE")
	#print "exist %s" %exist
	#g_common_obj.launch_app_from_home_sc("YouTube")
	mapsdata=self.gota.get_info("Maps data")
	print "mapsdata's data exist %s" %mapsdata
	if mapsdata=="exist":
	    exist=self.system.file_exists("/sdcard/android/data/com.google.android.apps.maps","DEVICE")
	    assert exist
	youtubedata=self.gota.get_info("Camera data")
	print "youtubedata's data exist %s" %mapsdata
	if youtubedata=="exist":
	    exist=self.system.file_exists("/sdcard/android/data/com.google.android.GoogleCamera","DEVICE")
	    assert exist
	if mapsdata==False and youtubedata==False:
	    print "not app data exist"
	    assert False
