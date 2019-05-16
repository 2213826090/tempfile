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
@summary: test clock function after gota
@since: 27/4/2015
@author: haley.han(yi.a.han@intel.com)
'''
import os
import sys
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.systemui.systemui_impl import SystemUI
from testlib.system.system_impl import SystemImpl
from testlib.clock.clock import Clock
from testlib.gota.gota_impl import gotaImpl
from testlib.common.base import UiWindows

class CheckClockAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckClockAfterGOTA, self).setUp()
        self._test_name = __name__
        self.system=SystemImpl(self)
        self.gota=gotaImpl(self)
        self.d=g_common_obj.get_device()
        self.clock=Clock()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckClockAfterGOTA, self).tearDown()

    def testCheckClockAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.unlock_screen_gota()
        self.clock.lunchAlarm()
        self.clock.clickAdd()
        self.d(text="OK").click()
        self.clock.getAlarmTime()
        self.clock.setAlarm2()
        print "Wait for alarm..."
        time.sleep(50)
        if self.d(resourceId="com.android.deskclock:id/alarm").exists:
            x,y = UiWindows(self.d(resourceId="com.android.deskclock:id/alarm").info).getMidPoint()
            endX = self.d.info[u'displayWidth']
            self.d.swipe(x,y,endX,y)
            return True
        self.clock.returnToHomeScreenAndSleep()