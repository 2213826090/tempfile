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
@summary: test add widget successfull after GOTA update
@since: 10/3/2015
@author: haley.han(yi.a.han@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
from testlib.systemui.systemui_impl import SystemUI
class ApkAddWidgetAfterGOTA(UIATestBase):
    """
    @summary: add widget successfull after GOTA update
    """

    def setUp(self):
        super(ApkAddWidgetAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.d=g_common_obj.get_device()
        #self.systemui=SystemUI()
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ApkAddWidgetAfterGOTA, self).tearDown()
        self.cfg = None

    def testApkAddWidgetAfterGOTA(self):
        """
        This test case is to test add widget successfull after GOTA update

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. process GOTA update
        2. add widget successfull after GOTA update

        Expect Result:
        1. add widget and launch it successfully

        """

        print "[RunTest]: %s" % self.__str__()

        #self.systemui.add_clock_to_widget()
        print("1 %s" %self.d.displayHeight)
        print("2 %s" %self.d.displayWidth)
        if self.d(className = "android.appwidget.AppWidgetHostView").exists:
            self.d(className = "android.appwidget.AppWidgetHostView").drag.to(self.d.displayWidth/2,80)
            #self.d(className = "android.appwidget.AppWidgetHostView").drag.to(400,100)
        if self.d(resourceId = "com.android.deskclock:id/analog_appwidget").exists:
            self.d(resourceId = "com.android.deskclock:id/analog_appwidget").drag.to(self.d.displayWidth/2,80)
            #self.d(resourceId = "com.android.deskclock:id/analog_appwidget").drag.to(400,100)
        self.d(resourceId = "com.google.android.googlequicksearchbox:id/active").long_click()
        self.d(text = "Widgets").click.wait()
        self.d(text = "Analog clock").long_click()
        assert self.d(resourceId = "com.android.deskclock:id/analog_appwidget").exists

        #self.systemui.launch_clock_from_widget()
        self.d.press.home()
        self.d(resourceId = "com.android.deskclock:id/analog_appwidget").click.wait()
        assert self.d(description = "Alarm").exists

        #self.systemui.rm_clock_widget()
        self.d.press.home()
        if self.d(resourceId = "com.android.deskclock:id/analog_appwidget").exists:
            self.d(resourceId = "com.android.deskclock:id/analog_appwidget").drag.to(self.d.displayWidth/2,80)
        assert not self.d(resourceId = "com.android.deskclock:id/analog_appwidget").exists

