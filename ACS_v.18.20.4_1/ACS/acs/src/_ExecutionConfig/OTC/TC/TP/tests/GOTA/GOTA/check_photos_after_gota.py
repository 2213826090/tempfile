#Copyright (C) 2015 Song, GuimeiX Z 
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
@summary: test photo function after gota
@since: 25/5/2015
@author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import string
import commands
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.common.common import g_common_obj2
from testlib.systemui.systemui_impl import SystemUI
from testlib.photos.photos_impl import PhotosImpl
from testlib.util.common import g_common_obj

class CheckPhotosAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckPhotosAfterGOTA, self).setUp()
        self._test_name = __name__
        self.gota= gotaImpl(self)
        self.photo=PhotosImpl()
        self.d=g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckPhotosAfterGOTA, self).tearDown()

    def testCheckPhotosAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        g_common_obj2.launch_app_from_home_sc("Photos")
        if self.d(resourceId="android:id/button1").exists:
            self.d(resourceId="android:id/button1").click()
        if self.d(resourceId="com.google.android.apps.plus:id/photos_app_later").exists:
            self.d(resourceId="com.google.android.apps.plus:id/photos_app_later").click()
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click()
            self.photo.display_image(1)
