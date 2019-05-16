# -*- coding: utf-8 -*-
'''
Created on Feb 15, 2015
@author: Ding, JunnanX
'''
import time

from testlib.util.common import g_common_obj
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.chromecast.chromecast_test_template import ChromeCastTestBase
from testlib.graphics.photos_impl import PhotosImpl


class TaskManagerTest(ChromeCastTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(TaskManagerTest, cls).setUpClass()

        config_file = 'tests.basicfunction.taskmanager.conf'
        cls.test_conf = cls.config.read(config_file, "close_recent_apps")

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(TaskManagerTest, self).setUp()

        self.cast_model = self.test_conf.get('cast_model')
        self.actor_apps = self.test_conf.get('actor_apps')

        cmd = 'date +"%m-%d %H:%M:%S"'
        self.test_time_mark = g_common_obj.adb_cmd_capture_msg(repr(cmd))

        self.chrome_cast = self.IChromeCastImpl(self.cast_model)
        self.systemui = SystemUiExtendImpl()
        self.photos = PhotosImpl()

        self.chrome_cast.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        cmd = 'logcat -d -v time -t "%s.000" *:F|grep -i Fatal' % (self.test_time_mark)
        fatal_msg = g_common_obj.adb_cmd_common(cmd)
        assert not fatal_msg, \
            "occurred Fatal error during testing:%s" % (fatal_msg)
        super(TaskManagerTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(TaskManagerTest, cls).tearDownClass()

    def test_TaskManager_CloseRecentApps(self):
        """
        test_TaskManager_CloseRecentApps

        Steps:
        1. Turn on Wireless Display on DUT.
                The Chromecast adapter is shown on the scan list.
        2. Connect to the Chromecast adapter
                The DUT is connected to the Chromecast adapter.
        3. Open a few applications
                The applications are opened
        4. Push the recent applications button
                The recent application screen is shown
        5. Close all applications
                The applications are closed and screencasting is successful
        """
        print "[RunTest]: %s" % self.__str__()
        print """[Step] 1. Turn on Wireless Display on DUT.
                    The Chromecast adapter is shown on the scan list."""
        self.turn_on_wireless()

        self.chrome_cast.launch()
        cast_real_name = self.chrome_cast.choose_castscreen(self.cast_model)

        print """[Step] 2. Connect to the Chromecast adapter
                   The DUT is connected to the Chromecast adapter."""
        self.chrome_cast.connect_cast_screen(cast_real_name)

        print "[Init APP]"
        print "Init Photos"
        g_common_obj.launch_app_am(
            " com.google.android.apps.photos", "com.google.android.apps.photos.home.HomeActivity")
        time.sleep(2)
        self.photos.stop_photos_am()
        time.sleep(1)
        print "Init Calendar"
        g_common_obj.launch_app_am(
            " com.google.android.calendar", "com.android.calendar.AllInOneCalendarActivity")
        if not self.d(description="Create new event and more").exists:
            for _ in range(0, 3):
                if self.d(description="next page").exists:
                    self.d(description="next page").click.wait()
            if self.d(text="Got it").exists:
                self.d(text="Got it").click.wait()
        g_common_obj.stop_app_am("com.google.android.calendar")
        time.sleep(2)

        print """[Step] 3. Open a few applications
                    The applications are opened"""
        for each in self.actor_apps.split(','):
            self.d.press.home()
            self.d(description="Apps").click()
            if self.d(scrollable=True).scroll.horiz.to(text=each):
                self.d(text=each).click()
                time.sleep(2)
                print "[Debug] open app %s" % (each)

        print """[Step] 4. Push the recent applications button
                        The recent application screen is shown"""
        for each in self.actor_apps.split(','):
            self.d.press.home()
            self.d.press.recent()
            self.d().scroll.vert.toEnd()
            assert self.d().scroll.vert.to(text=each), \
                "[FAILURE] %s Not appear in recent list" % (each)
            print "[Debug] app %s existed in recent" % (each)
            time.sleep(1)

        print """[Step] 5. Close all applications
                    The applications are closed and screencasting is successful"""
        self.systemui.close_all_recent_apps()

#         print """[Step] 6. Verify connection.
#                     The connection is not lost."""
#         self.chrome_cast.watch_connection_status(count=1, step=1)
#         self.chrome_cast.verify_connection_from_quick_settings(self.cast_model)
# 
#         self.chrome_cast.resume()
#         self.chrome_cast.disconnect_cast_screen()
