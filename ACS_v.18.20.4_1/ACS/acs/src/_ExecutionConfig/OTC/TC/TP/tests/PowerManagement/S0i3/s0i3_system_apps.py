#! /usr/bin/env python
# coding:utf-8

#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0ix import get_s0ix_obj
from testlib.em.settings import DisplaySetting

class S0i3SystemApps(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0ix = get_s0ix_obj()
        self.s0ix.adb_root()
        self.s0ix.set_screen_status("on")
        self.s0ix.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(S0i3SystemApps, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.s0ix.reconnect_usb()
        super(S0i3SystemApps, self).tearDown()

    def test_enter_s0i3_notification(self):
        self.s0ix.open_notification()
        time.sleep(2)
        assert self.s0ix.check_notification_status()
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"
        # drag notification again
        self.s0ix.open_notification()
        time.sleep(2)
        assert self.s0ix.check_notification_status()

    def test_enter_s0i3_wifi_bt_gps_on(self):
        from testlib.em.settings import WifiSetting, BTSetting, LocationSetting
        wifi = WifiSetting()
        bt = BTSetting()
        gps = LocationSetting()
        wifi.switch_wifi("ON")
        bt.switch_bt("ON")
        gps.switch_GPS("ON")
        time.sleep(2)
        stat_inc = self.s0ix.suspend_resume()
        bt.switch_bt("OFF")
        gps.switch_GPS("OFF")
        assert stat_inc > 0, "Not enter S3"

    def test_task_manager_work_wakeup_from_s0i3(self):
        from testlib.em.settings import Settings
        Settings().launch()
        app1 = self.s0ix.get_focused_app()
        self.s0ix.press_home_button()
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc, "Not enter S3"
        self.s0ix.resume_app_from_task_manager("Settings")
        app2 = self.s0ix.get_focused_app()
        assert app1 == app2

    def test_enter_s0i3_airplane_mode_on(self):
        from testlib.em.settings import AirplaneModeSetting
        air = AirplaneModeSetting()
        air.switch_airplane("ON")
        stat_inc = self.s0ix.suspend_resume()
        air.check_airplane_status("ON")
        air.switch_airplane("OFF")
        assert stat_inc, "Not enter S3"

    def test_s0i3_resume_by_power_button(self):
        from testlib.em.apps import EMToolsScreen
        app = EMToolsScreen()
        app.install()
        app.grant_permissions()
        app.start_monitor()
        stat_inc = self.s0ix.suspend_resume_special(sleep_flags = self.s0ix.RESUME_BY_POWER)
        assert stat_inc
        history = app.get_history()
        assert history.count("ON") == 3

    def test_s0i3_resume_by_charger_plug(self):
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"
        self.s0ix.verify_screen_status("on")

    def test_enter_s0i3_only_battery(self):
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"

    def test_power_icon_check_charger_resume(self):
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.tools import get_tmp_dir, remove_tmp_dir
        from testlib.em.crop_battery_icon import CropBatteryImage
        import os
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"
        tmp_dir = get_tmp_dir()
        screenshot = os.path.join(tmp_dir, "screenshot.png")
        rect = DisplaySetting().get_status_bar_rect()
        self.s0ix.capture_screen(screenshot)
        crop_battery = CropBatteryImage(screenshot, rect)
        crop_battery.crop_battery()
        crop_battery.check_charging_status_by_icon(True)
        remove_tmp_dir(tmp_dir)

    def test_enter_s0i3_auto_rotation(self):
        DisplaySetting().set_auto_rotate(True)
        stat_inc = self.s0ix.suspend_resume()
        assert stat_inc > 0, "Not enter S3"
        self.s0ix.rotate("l")
        self.s0ix.rotate("n")

