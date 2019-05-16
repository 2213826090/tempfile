# -*- coding: utf-8 -*-
'''
Created on 05/06/2015
@author: Ding, JunnanX
'''

import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import logcat, get_resource_from_atifactory, pkgmgr
from testlib.graphics.common import launch_aosp_home, remove_aosp_launcher
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.photos_impl import get_photo_implement


class CardviewRecentsTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(CardviewRecentsTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        cls.config_file = 'tests.ComposeUI.CardviewRecentsTest.conf'
        cls.test_conf = cls.config.read(cls.config_file, "CardviewRecentsTest")

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(CardviewRecentsTest, self).setUp()
        self.actor_apps = self.test_conf.get('actor_apps')
        self.calendar_path = get_resource_from_atifactory(self.config_file, "apps", "calendar")
        self.clock_path = get_resource_from_atifactory(self.config_file, "apps", "clock")
        pkgmgr.apk_install(self.calendar_path)
        pkgmgr.apk_install(self.clock_path)
        self.systemui = SystemUiExtendImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()
        self.mark_time = logcat.get_device_time_mark()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0, \
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
        super(CardviewRecentsTest, self).tearDown()
        self.photos.rm_delete_photos()
        remove_aosp_launcher()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(CardviewRecentsTest, cls).tearDownClass()

    def test_Cardview_Recents(self):
        """
        test_Cardview_Recents

        Steps:
        1. launch 5 apps 
        # change to 5 apps due to some of apps in step are not preinstalled.
            all launch successfully
        2. launch Recents to swtich between different app
            Check CardView in Recents are working correctly
        """
        print "[RunTest]: %s" % self.__str__()

        print "[Init APP]"
        print "Init Photos"
        g_common_obj.launch_app_am(
            " com.google.android.apps.photos", "com.google.android.apps.photos.home.HomeActivity")
        time.sleep(2)
        self.photos.stop_photos_am()
        time.sleep(1)
        print "Init Calendar"
        g_common_obj.launch_app_am(
            "com.google.android.calendar", "com.android.calendar.AllInOneActivity")
        if not self.d(description="Create new event and more").exists:
            for _ in range(0, 3):
                if self.d(description="next page").exists:
                    self.d(description="next page").click.wait()
            if self.d(text="Got it").exists:
                self.d(text="Got it").click.wait()
        g_common_obj.stop_app_am("com.google.android.calendar")
        time.sleep(2)
        print """[Step] 1. launch 5 apps
            all launch successfully"""
        for each in self.actor_apps.split(','):
            if "com.android.launcher3" not in pkgmgr.get_packages():
                launch_aosp_home()
            self.systemui.launch_desk_app(each)
            time.sleep(2)

        print """[Step] 2. launch Recents to swtich between different app
            Check CardView in Recents are working correctly"""
        for each in self.actor_apps.split(','):
            self.systemui.switch_recent_app(each)
            time.sleep(2)
