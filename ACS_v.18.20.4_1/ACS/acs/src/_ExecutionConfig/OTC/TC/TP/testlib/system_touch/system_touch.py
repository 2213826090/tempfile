# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
from system_touch_comm import SystemTouchComm
import time
import os
import subprocess
import multiprocessing

d = g_common_obj.get_device()
dsn = d.server.adb.device_serial()

class SystemTouch(SystemTouchComm):

    def __init__(self):
        SystemTouchComm.__init__(self)

    def get_config_value(self, tag, name):
        cfg = TestConfig().read(self.cfg_file, tag)
        return cfg.get(name)

    def get_state(self):
        state = g_common_obj.adb_cmd_common("get-state")
        print "[info]--- device state:", state
        if 'device' in state:
            return True
        else:
            return False

    def get_fake_touch_status(self):
        class_name = "com.intel.test.apitests.tests.TouchscreenTestsDriver#testHasFaketouch"
        runner = "com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner"
        cmd = "am instrument -e class %s -w %s" % (class_name, runner)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "FAILURES" in msg:
            return False
        if not "OK (1 test)" in msg:
            return False
        return True

    def touch_screen_api_features(self):
        class_name = "com.intel.test.apitests.tests.TouchscreenTestsDriver#testTouchScreenApiFeatures"
        runner = "com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner"
        cmd = "am instrument -e class %s -w %s" % (class_name, runner)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "FAILURES" in msg:
            return False
        if not "OK (1 test)" in msg:
            return False
        return True

    def get_touch_screen_presence(self):
        class_name = "com.intel.test.apitests.tests.TouchscreenTestsDriver#testHasTouchscreen"
        runner = "com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner"
        cmd = "am instrument -e class %s -w %s" % (class_name, runner)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "FAILURES" in msg:
            return False
        if not "OK (1 test)" in msg:
            return False
        return True

    def get_touch_screen_type(self):
        class_name = "com.intel.test.apitests.tests.TouchscreenTestsDriver#testTouchscreenFinger"
        runner = "com.intel.test.apitests/com.intel.test.apitests.runners.GenericArgumentPassingTestRunner"
        cmd = "am instrument -e class %s -w %s" % (class_name, runner)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if "FAILURES" in msg:
            return False
        if not "OK (1 test)" in msg:
            return False
        return True

    def open_notification_by_swipe(self):
        info = d.info
        x = info["displayWidth"] / 2
        y = info["displayHeight"] / 2
        d.open.notification()
        time.sleep(2)
        d.swipe(x, 0, x, y)
        assert d(resourceId = "com.android.systemui:id/notification_stack_scroller").exists

    def long_press_on_home_for_L(self):
        d.press.home()
        if d(resourceId="com.android.vending:id/error_msg").exists:
            if d(description="Apps").exists:
                bounds_app = d(description="Apps").bounds
                x1 = (bounds_app["right"] + bounds_app["left"]) / 2
                y1 = bounds_app["top"] - 6
                d.long_click(x1, y1)
                time.sleep(1)
                assert d(text="Wallpapers").exists
        else:
            bounds = d(className="android.view.View").bounds
            x = (bounds["right"] + bounds["left"]) / 2
            y = (bounds["bottom"] - bounds["top"]) / 2
            d.long_click(x, y)
            time.sleep(1)
            assert d(text="Wallpapers").exists

    def long_press_on_home_screen(self):
        d.press.home()
        bounds = d(className = "android.view.ViewGroup").bounds
        width1 = bounds["right"] - bounds["left"]
        height1 = bounds["bottom"] - bounds["top"]
        print width1, height1
        x = (bounds["right"] + bounds["left"]) / 2
        y = (bounds["bottom"] - bounds["top"]) / 2
        d.long_click(x, y)
        time.sleep(1)
        if d(resourceId="android:id/contentPanel").exists:
            return True
        else:
            bounds = d(className = "android.view.ViewGroup").bounds
            width2 = bounds["right"] - bounds["left"]
            height2 = bounds["bottom"] - bounds["top"]
            print width2, height2
            assert width1 > width2 and height1 > height2

    def long_press_gallery_for_ivi_O(self, hold_time = None):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.screenshot_png("screenshot_1")
        g_common_obj.launch_app_am("com.android.gallery3d", "com.android.gallery3d.app.GalleryActivity")
        time.sleep(2)
        bounds = d(className="android.view.View").bounds
        width_center = ((bounds["right"] - bounds["left"]) / 2) + bounds["left"] -10
        height_center = ((bounds["bottom"] - bounds["top"]) / 2) + bounds["top"]
        if d(className="android.widget.ImageButton").exists:
            d(className="android.widget.ImageButton").click.wait()
            if d(text="Select album").exists:
                d(text="Select album").click.wait()
            time.sleep(1)
            d.long_click(width_center, height_center)
            time.sleep(1)
            if d(text="1 item selected").exists:
                time.sleep(1)
                d(text="DONE").click.wait()
            else:
                self.reboot_devices()
                g_common_obj.launch_app_am("com.android.gallery3d", "com.android.gallery3d.app.GalleryActivity")
        if hold_time is None:
            d.long_click(width_center, height_center)
            time.sleep(1)
            assert d(description="Delete").exists
        else:
            d.swipe(width_center, height_center, width_center, height_center, steps=hold_time)
            time.sleep(1)
            assert d(description="Delete").exists

    def long_press_photos_for_ivi_O(self, hold_time = None):
        self.launch_photos_app()
        if d(text="No Photos").exists:
            g_common_obj.launch_app_am("com.android.settings", ".Settings")
            self.screenshot_png("screenshot_1")
            self.launch_photos_app()
        if d(text="No Photos").exists:
            self.reboot_devices()
            self.launch_photos_app()
        if d(descriptionContains="Photo").exists:
            bounds = d(descriptionContains="Photo").bounds
            width_center = ((bounds["right"] - bounds["left"]) / 2) + bounds["left"]
            height_center = ((bounds["bottom"] - bounds["top"]) / 2) + bounds["top"]
            if hold_time is None:
                d.long_click(width_center, height_center)
                assert d(resourceId="com.google.android.apps.photos:id/move_to_trash").exists
            else:
                d.swipe(width_center, height_center, width_center, height_center, steps=hold_time)
                assert d(resourceId="com.google.android.apps.photos:id/move_to_trash").exists

    def check_long_press_gallery_photos_for_O(self, hold_time = None):
        gallery_pack = "com.android.gallery3d"
        photos_pack = "com.google.android.apps.photos"
        if self.check_deivces_app_exist(photos_pack) is True:
            self.long_press_photos_for_ivi_O(hold_time)
            return
        if self.check_deivces_app_exist(gallery_pack) is True:
            self.long_press_gallery_for_ivi_O(hold_time)

    def long_press_build_comm(self):
        product_name = self.check_product()
        build_release = self.check_build_release()
        print "[INFO]product_name: %s" % product_name
        print "[INFO]Build release: %s" % build_release
        if "celadon" in product_name or "androidia" in product_name:
            self.long_press_on_home_screen()
            return
        if "5." in build_release:
            self.long_press_on_home_for_L()
        elif "6." in build_release or "7." in build_release:
            self.long_press_on_home_screen()
        elif "8." in build_release:
            # O build "gordon_peak", "icl_presi_kbl"
            self.check_long_press_gallery_photos_for_O()
        else:
            self.long_press_on_home_screen()

    def check_icon_long_press_build_comm(self, sub_icon = None, hold_time = None):
        product_name = self.check_product()
        build_release = self.check_build_release()
        print "[INFO]product_name: %s" % product_name
        print "[INFO]Build release: %s" % build_release
        if "celadon" in product_name or "androidia" in product_name:
            self.long_press_icon_on_home(sub_icon, hold_time)
            return
        if "6." in build_release:
            self.long_press_icon_on_home(sub_icon, hold_time)
        elif "7." in build_release:
            self.long_press_icon_on_home(sub_icon, hold_time)
        elif "8." in build_release:
            # O build "gordon_peak", "icl_presi_kbl"
            self.check_long_press_gallery_photos_for_O(hold_time)
        else:
            self.long_press_icon_on_home(sub_icon, hold_time)

    def check_swipe_vertical(self):
        self.launch_developer_option()
        time.sleep(3)
        assert d(text="Stay awake").exists
        d().swipe.up(steps=10)
        assert not d(text="Stay awake").exists

    def check_swipe_horizontal(self):
        g_common_obj.launch_app_am("com.google.android.deskclock", "com.android.deskclock.DeskClock")
        time.sleep(2)
        product_name = self.check_product()
        print "[INFO]product_name: %s" % product_name
        #if "androidia" in product_name:
        if d(text="CLOCK").exists:
            d(text="CLOCK").click()
            assert d(text="CLOCK").selected
            d().swipe.right(steps=10)
            assert not d(text="CLOCK").selected
        else:
            d(description="Clock").click()
            assert d(description="Clock").selected
            d().swipe.right(steps=10)
            assert not d(description="Clock").selected

    def launch_chrome_skip_accept(self):
        # skip chrome's two options
        os.system("adb -s %s shell am start -S com.android.chrome/com.google.android.apps.chrome.Main" % dsn)
        time.sleep(5)
        while d(text="ACCEPT & CONTINUE").exists:
            d(text="ACCEPT & CONTINUE").click.wait()
        '''if d(resourceId="com.android.chrome:id/positive_button"):
            d(resourceId="com.android.chrome:id/positive_button").click.wait()'''
        if d(text="Done").exists:
            d(text="Done").click.wait()
        if d(text="NO THANKS").exists:
            d(text="NO THANKS").click.wait()

    def grant_permission(self, package, permission):
        g_common_obj.adb_cmd("pm grant %s %s" % (package, permission))

    def grant_permissions_for_chrome_app(self):
        g_common_obj.adb_cmd("pm grant com.android.chrome android.permission.READ_EXTERNAL_STORAGE")

    def close_chrome_tabs(self):
        g_common_obj.adb_cmd("rm -rf /data/data/com.android.chrome/app_tabs/")

    def open_webpage(self, url):
        print "[Info] ---Open webpage %s." % url
        cmd = "am start -S -n com.android.chrome/com.google.android.apps.chrome.Main -d %s" % url
        g_common_obj.adb_cmd(cmd)
        time.sleep(5)
        for i in range(10):
            if d(resourceId="com.android.chrome:id/url_bar").exists:
                return
            if d(resourceId="com.android.chrome:id/terms_accept").exists:
                d(resourceId="com.android.chrome:id/terms_accept").click.wait()
            if d(resourceId="com.android.chrome:id/negative_button").exists:
                d(resourceId="com.android.chrome:id/negative_button").click.wait()
            if d(text="Done").exists:
                d(text="Done").click.wait()
            if d(text="Next").exists:
                d(text="Next").click.wait()
            time.sleep(2)
        assert False , "Input URL failed"

    def check_gesture_zoom(self):
        time.sleep(2)
        if d(resourceId="header").exists:
            bounds1 = d(resourceId="header").bounds
        else:
            print "[Info]------Web header not exists, no bounds"
        info = d.info
        x_unit = info["displayWidth"] / 10
        x1 = x_unit * 4
        x2 = x_unit * 6
        x3 = x_unit * 1
        x4 = x_unit * 9
        y = info["displayHeight"] / 2
        d().gesture((x1, y), (x2, y)).to((x3, y), (x4, y))
        d(className="android.webkit.WebView").swipe.right(steps=10)
        x = info["displayWidth"] / 2
        y2 = info["displayHeight"]
        y1 = y2 / 2
        d.swipe(x, y1, x, y2, 10)
        time.sleep(2)
        build_release = self.check_build_release()
        if "5." in build_release:
            print "[Info]------ gesture screen zoom success"
            return
        bounds2 = d(resourceId="header").bounds
        height1 = bounds1["bottom"] - bounds1["top"]
        height2 = bounds2["bottom"] - bounds2["top"]
        print height1, height2
        assert height1 < height2

    def screenshot_png(self, screenshot_name):
        self.adb_root()
        sc_cmd = "/system/bin/screencap -p sdcard/Pictures/%s.png" % screenshot_name
        g_common_obj.adb_cmd(sc_cmd)
        self.set_workaround()

    def set_workaround(self):
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    def capture_screen(self,):
        script_name = "capture_screen.sh"
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        exec_dir = "/mnt/sdcard/"
        screenshot_on_device = "/mnt/sdcard/screenshot.png"
        push_to_dir = "/mnt/sdcard/"
        cmd = "push %s %s" % (script_path, push_to_dir)
        g_common_obj.adb_cmd_common(cmd)
        #np = NohupProcess(self.testDevice, script_path, exec_dir, "5 %s" % screenshot_on_device)
        #np.start()

    def launch_photos_app(self):
        for i in range(2):
            g_common_obj.launch_app_am("com.google.android.apps.photos", ".home.HomeActivity")
            time.sleep(5)
            if d(className="android.widget.Button").exists:
                d.press.back()
            if d(textContains="KEEP OFF").exists:
                d(textContains="KEEP OFF").click.wait()
            time.sleep(3)
            if d(packageName="com.google.android.apps.photos").exists:
                return
        assert d(packageName="com.google.android.apps.photos").exists

    def check_gesture_double_tap_chrome_page(self, url=None):
        #self.open_webpage(url)
        #d.press.menu()
        '''if d(descriptionContains="Turn on Request desktop site").exists:
            d(resourceId="com.android.chrome:id/menu_item_icon").click.wait()
            d.press.menu()
            assert d(descriptionContains="Turn off Request desktop site").exists
            d.press.back()
        print 111
        '''
        #d(className="android.webkit.WebView").click.wait(timeout=0.001)
        time.sleep(10)

        #time.sleep(float(0.0000000000000001))
        d.click(300,300).wait().click(300,300).perform()
        #d.press(300, 300).release().wait(500).press(300, 300).release().perform()
        time.sleep(10)
        print 123

    def enable_devoption_touch_location(self, status=True):
        self.unlock_screen()
        time.sleep(3)
        self.launch_developer_option()
        time.sleep(2)
        M_touch = "Show touches"
        N_touch = "Show taps"
        build_release = self.check_build_release()
        print "[INFO]Build release: %s" % build_release
        if "5." in build_release or "6." in build_release:
            self.scrollable_select_text("Pointer location")
            self.setting_ON_OFF("Pointer location", enable=status)
            self.scrollable_select_text(M_touch)
            self.setting_ON_OFF(M_touch, enable=status)
        elif "7." in build_release or "8." in build_release:
            self.scrollable_select_text("Pointer location")
            self.setting_ON_OFF("Pointer location", enable=status)
            self.scrollable_select_text(N_touch)
            self.setting_ON_OFF(N_touch, enable=status)
        else:
            self.scrollable_select_text("Pointer location")
            self.setting_ON_OFF("Pointer location", enable=status)
            self.scrollable_select_text(N_touch)
            self.setting_ON_OFF(N_touch, enable=status)

    def home_to_app_list(self):
        d.press.home()
        if d(description="Apps").exists:
            d(description="Apps").click.wait()
        else:
            self.single_swipe_to_app_list()

    def long_press_icon_on_home(self, sub_icon, hold_time):
        d.press.home()
        if d(textContains=sub_icon).exists:
            print "[INFO]:Sub icon has been exist"
        else:
            self.home_to_app_list()
            self.drag_icon_on_ui(sub_icon)
        build_release = self.check_build_release()
        print "[INFO]Build release: %s" % build_release
        coordinates = self.bounds_icon_coordinates(sub_icon)
        print "[INFO]Before Hold: %s " % coordinates
        x_center = coordinates[0]
        y_center = coordinates[1]
        d.swipe(x_center, y_center, x_center, y_center, steps=hold_time)
        time.sleep(3)
        if "6." in build_release:
            assert d(textContains=sub_icon).exists
            coordinates_new = self.bounds_icon_coordinates(sub_icon)
        elif "7." in build_release:
            assert d(resourceId="com.android.launcher3:id/deep_shortcut").exists
            d.press.home()
            coordinates_new = self.bounds_icon_coordinates(sub_icon)
        else:
            print "[INFO]Other build_release"
            d.press.home()
            assert d(textContains=sub_icon).exists
            coordinates_new = self.bounds_icon_coordinates(sub_icon)
        print "[INFO]After  Hold: %s " % coordinates_new
        x_center_new = coordinates_new[0]
        y_center_new = coordinates_new[1]
        if x_center_new == x_center and y_center_new == y_center:
            return True
        else:
            assert False, "Test Result : hold icon is jitter"

    def drag_icon_on_ui(self, sub_icon):
        #self.home_to_app_list()
        self.scrollable_select_text(sub_icon)
        coordinates = self.bounds_icon_coordinates(sub_icon)
        x_center = coordinates[0]
        y_center = coordinates[1]
        if x_center < 400:
            if y_center < 400:
                x_center_to = x_center + 200
                y_center_to = y_center + 200
            elif y_center > 1000:
                x_center_to = x_center + 200
                y_center_to = y_center - 200
            else:
                x_center_to = x_center + 200
                y_center_to = y_center - 100
        elif x_center > 1000:
            if y_center < 400:
                x_center_to = x_center - 200
                y_center_to = y_center + 200
            elif y_center > 1000:
                x_center_to = x_center - 200
                y_center_to = y_center - 200
            else:
                x_center_to = x_center - 200
                y_center_to = y_center - 100
        else:
            if y_center < 400:
                x_center_to = x_center + 100
                y_center_to = y_center + 100
            elif y_center > 1000:
                x_center_to = x_center + 100
                y_center_to = y_center - 100
            else:
                x_center_to = x_center + 100
                y_center_to = y_center - 100
        d.drag(x_center, y_center, x_center_to, y_center_to)
        time.sleep(5)
        assert d(textContains=sub_icon).exists

    def drag_icon_on_home_(self, sub_icon):
        coordinates = self.bounds_icon_coordinates(sub_icon)
        x_center = coordinates[0]
        y_center = coordinates[1]
        d.drag(x_center, y_center)

    def bounds_icon_coordinates(self, sub_text):
        bounds = d(textContains=sub_text).bounds
        x1 = bounds["left"]
        x2 = bounds["right"] - 1
        y1 = bounds["top"] + 1
        y2 = bounds["bottom"] - 1
        #coordinates = [x1, y1, x2, y2]
        x_center = (x2 - x1) / 2 + x1
        y_center = (y2 - y1) / 2 + y1
        coordinates = [x_center, y_center]
        return coordinates

    def single_touch_settings_search_check_coordinates(self, sub_text=None):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        product_name = self.check_product()
        build_release = self.check_build_release()
        print "[INFO]product_name: %s" % product_name
        print "[INFO]Build release: %s" % build_release
        if "5." in build_release or "6." in build_release:
            bounds = d(resourceId="com.android.settings:id/search").bounds
        elif "7." in build_release:
            bounds = d(resourceId="com.android.settings:id/search").bounds
        elif "8." in build_release:
            bounds = d(description="Search settings").bounds
        else:
            bounds = d(description="Search settings").bounds
        x1 = bounds["left"]
        x2 = bounds["right"] - 1
        y1 = bounds["top"] + 1
        y2 = bounds["bottom"] - 1
        print x1, x2, y1, y2
        d.click(x1, y1)
        time.sleep(1)
        assert d(resourceId="android:id/search_src_text").exists
        d.press.back()
        d.press.back()
        time.sleep(1)
        d.click(x2, y1)
        time.sleep(1)
        assert d(resourceId="android:id/search_src_text").exists
        d.press.back()
        d.press.back()
        time.sleep(1)
        d.click(x1, y2)
        time.sleep(1)
        assert d(resourceId="android:id/search_src_text").exists
        d.press.back()
        d.press.back()
        time.sleep(1)
        d.click(x2, y2)
        time.sleep(1)
        assert d(resourceId="android:id/search_src_text").exists

    def check_touch_cannot_wake_up_when_screen_off(self):
        build_release = self.check_build_release()
        product_name = self.check_product()
        print "[INFO]product_name: %s" % product_name
        print "[INFO]Build release: %s" % build_release
        if "8." in build_release:
            if "celadon" in product_name or "androidia" in product_name:
                self.screen_turn_off_option_AIA("Sleep")
            else:
                self.check_enter_s0i3_state_for_ivi()
            print"[INFO]Click touch screen %s" % d.click(300,300)
            print"[INFO]Click touch screen %s" % d.click(600, 800)
        else:
            self.enter_screen_off()
            print"[INFO]Click touch screen %s" % d.click(300, 300)
            print"[INFO]Click touch screen %s" % d.click(600, 800)
            g_common_obj.adb_cmd_capture_msg("input tap 300 600")
            time.sleep(5)
            self.check_screen_off()
            self.set_screen_status("on")
            self.unlock_screen()

    def screen_on_off_getevent_no_output(self):
        product_name = self.check_product()
        if "cht_mrd" in product_name:
            self.check_touch_screen_response_comm()
        else:
            self.check_single_touch_sendevent_screen_response_comm()
        self.check_getevent_no_output()
        self.screen_turn_on_off_comm()
        if "cht_mrd" in product_name:
            self.check_touch_screen_response_comm()
        else:
            self.check_single_touch_sendevent_screen_response_comm()
        self.check_getevent_no_output()

    def check_getevent_no_output(self):
        wait_time = 60
        cmd = "timeout %s getevent | grep -e /dev/input/event[0-9]:" % wait_time
        msg = g_common_obj.adb_cmd_capture_msg(cmd, time_out=wait_time + 3)
        assert "" == msg

    def check_single_touch_sendevent_screen_response_comm(self):
        self.push_send_event_script("touch_click_sendevent.sh")
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if type(self.event_info) == dict:
            print "[Screen_Event_Info]---Screen event info type is Dict, connect to 1 TS"
            self.check_sendevent_touch_screen_response(self.event_info)
            self.check_sendevent_touch_screen_Security(self.event_info)
        if type(self.event_info) == tuple:
            print "[Screen_Event_Info]---Screen event info type is Tuple, connect to > 1 TS"
            for event_info in self.event_info:
                print "<<<Multi_TS>>> Test Event Info: {}".format(event_info)
                self.check_sendevent_touch_screen_response(event_info)
                self.check_sendevent_touch_screen_Security(event_info)

    def check_sendevent_touch_screen_response(self, event_info, sp_num = None):
        if sp_num == None:
            scale_x = (event_info["scale_x"])
        else:
            scale_x = (event_info["scale_x"]) * sp_num
        scale_y = (event_info["scale_y"])
        ave_x, ave_y = self.open_text_get_coordinates_in_setting("Display", scale_x, scale_y)
        max_cmd = self.cmd_touch_click_sendevent(event_info["dev_num"], ave_x, ave_y)
        # fixed to not click touch screen on CHT-T3_MRD by sendevent commands
        for i in range(3):
            g_common_obj.adb_cmd_capture_msg(max_cmd)
            time.sleep(3)
            if d(textContains="Sleep").exists:
                return
        assert d(textContains="Sleep").exists

    def check_sendevent_touch_screen_Security(self, event_info, sp_num = None):
        if sp_num == None:
            scale_x = event_info["scale_x"]
        else:
            scale_x = (event_info["scale_x"]) * sp_num
        scale_y = event_info["scale_y"]
        ave_x, ave_y = self.open_text_get_coordinates_in_setting("Security", scale_x, scale_y)
        max_cmd = self.cmd_touch_click_sendevent(event_info["dev_num"], ave_x, ave_y)
        # fixed to not click touch screen on CHT-T3_MRD by sendevent commands
        for i in range(3):
            g_common_obj.adb_cmd_capture_msg(max_cmd)
            time.sleep(3)
            self.scrollable_select_text("Encryption")
            if d(textContains="Encryption").exists:
                return
        assert d(textContains="Encryption").exists

    def open_text_get_coordinates_in_setting(self, target_text, scale_x, scale_y):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.scrollable_select_text(target_text)
        sett_coordinates = self.bounds_icon_coordinates(target_text)
        if d(text="Device").exists:
            d(text="Device").click.wait()
        ave_x = int(sett_coordinates[0] * scale_x)
        ave_y = int(sett_coordinates[1] * scale_y)
        time.sleep(2)
        return ave_x, ave_y

    def back_ui_from_setting_text(self, event_info, target_text, touch_sp, sp_num = None):
        # touch_sp(screen : 1, screen : 2)
        if sp_num == None:
            scale_x = int(event_info["scale_x"])
        else:
            scale_x = int(event_info["scale_x"]) * sp_num
        scale_y = int(event_info["scale_y"])
        ave_x, ave_y = self.open_text_get_coordinates_in_setting(target_text, scale_x, scale_y)
        max_cmd = self.cmd_touch_click_sendevent(event_info["dev_num"], ave_x, ave_y)
        g_common_obj.adb_cmd_capture_msg(max_cmd)
        time.sleep(3)
        back_cmd = self.cmd_touch_click_sendevent(event_info["dev_num"], ave_x, ave_y)
        g_common_obj.adb_cmd_capture_msg(back_cmd)
        if touch_sp == 1:
            assert not d(textContains="Sleep").exists






    def check_touch_screen_response_step(self):
        self.unlock_screen()
        self.home_to_app_list()
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        if d(textContains="Display").exists:
            d(textContains="Display").click.wait()
        if d(textContains="Sleep").exists:
            print "[INFO]Touch screen response: Passed"
            assert d(textContains="Sleep").exists

    def check_touch_screen_response_comm(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        for i in range(3):
            self.scrollable_select_text("Display")
            if d(textContains="Display").exists:
                d(textContains="Display").click.wait()
                assert d(textContains="Sleep").exists
            d.press.back()
            self.scrollable_select_text("Security")
            if d(textContains="Security").exists:
                d(textContains="Security").click.wait()
                assert d(textContains="Encryption").exists
            d.press.back()
        if d(resourceId="com.android.settings:id/search").exists:
            d(resourceId="com.android.settings:id/search").click.wait()
            if not d(resourceId="android:id/search_src_text").exists:
                assert False, d(resourceId="android:id/search_src_text").exists

    def check_swipe_vertical_settings(self):
        g_common_obj.adb_cmd_capture_msg("am start -a android.settings.SETTINGS")


    def check_virtual_keyboard_display(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        if d(resourceId="com.android.settings:id/search").exists:
            d(resourceId="com.android.settings:id/search").click.wait()
        time.sleep(3)

    def launch_touch_screen_test_circle(self):
        ts_test_package = "com.mycompany.anton_mokshyn.touchscreentest"
        self.install_artifactory_app("fullcircle_touchscreentest", ts_test_package)
        # main ui, .MainActivity
        g_common_obj.launch_app_am(ts_test_package, ".MainActivity")
        d(resourceId="com.mycompany.anton_mokshyn.touchscreentest:id/start_button").click.wait()

    def check_touch_screen_log_pass(self):
        g_common_obj.adb_cmd_common("logcat -c")
        pass_log = " logcat -d |grep 'Type, name: event, test passed' "
        test_pass_log = g_common_obj.adb_cmd_common(pass_log)
        if not test_pass_log is None:
            assert False, "[Info] test is Failed"

    def check_M_touch_screen_position_no_jitter_edge_to_edge(self):
        self.launch_touch_screen_test_circle()
        self.adb_root()
        index = 1
        time.sleep(2)
        d.click(30,28)
        loop_click_top = 19*2 -1
        for line1 in range(loop_click_top):
            #d(index=index).click.wait()
            index += 1
        # swipe
        loop_swipe = 19
        d(index=loop_swipe).swipe.right(steps=10)

    def check_comm_touch_screen_position_no_jitter_edge_to_edge(self):
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            return
        else:
            self.check_M_touch_screen_position_no_jitter_edge_to_edge()

    def check_no_line_interruption_when_sliding(self):
        self.push_send_event_script("draw_pattern_lock.sh")
        if self.set_screen_lock_pattern() is False:
            if self.check_set_screen_lock_success() is False:
                self.set_screen_lock_pattern()
        self.check_swipe_vertical()
        if self.remove_screen_lock_pattern() is False:
            if self.check_remove_screen_lock_success() is False:
                if self.remove_screen_lock_pattern() is False:
                    self.remove_screen_lock_pattern()

    def launch_Multitouch_test_app(self):
        multi_touchme_package = "com.slydroid.multitouch"
        self.install_artifactory_app("ts_slydroid_tme", multi_touchme_package)
        self.clear_app_data(multi_touchme_package)
        g_common_obj.launch_app_am("com.slydroid.multitouch", ".MainActivity")

    def touch_screen_fingers(self):
        cfg = TestConfig().read(self.cfg_file, "touch_fingers")
        max_finger = cfg.get("max_finger")
        # official_ts_ivi support 4 fingers
        official_ts_ivi = cfg.get("official_ts_ivi")
        return cfg

    def check_maximum_fingers_support_sendevent(self):
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if type(self.event_info) == dict:
            print "[Screen_Event_Info]---Screen event info type is Dict, connect to 1 TS"
            get_text_fingers = self.maximum_fingers_support(self.event_info)
            return get_text_fingers
        if type(self.event_info) == tuple:
            print "[Screen_Event_Info]---Screen event info type is Tuple, connect to > 1 TS"
            get_text_fingers = {}
            print "<<<Multi_TS>>> Test Event Info: {}".format(self.event_info[0])
            get_text_fingers = self.maximum_fingers_support(self.event_info[0])
            print get_text_fingers
            return get_text_fingers

    def maximum_fingers_support(self, event_info):
        product_name = self.check_product()
        touch_fingers = self.touch_screen_fingers()
        self.launch_Multitouch_test_app()
        #if d(resourceId="com.slydroid.multitouch:id/RelativeLayout1").exists:
        bounds = d(resourceId="com.slydroid.multitouch:id/RelativeLayout1").bounds
        ave_x = ((bounds["right"]) / 12 )
        ave_y = (((bounds["bottom"]) / 10 ))
        for _ in range(3):
            d.click((bounds["right"]) - 100, (bounds["bottom"]) - 100 )
            d.click((bounds["right"]) - 300, (bounds["bottom"]) - 300)
        time.sleep(2)
        scale_x = int(event_info["scale_x"])
        scale_y = int(event_info["scale_y"])
        view_n = 3
        get_text_fingers = {}
        get_text_fingers["ave_coordinates"] = [ave_x, ave_y]
        try:
            if "bxt" in product_name or "gordon_peak" in product_name or "icl_presi_kbl" in product_name \
                    or "androidia" in product_name or "celadon" in product_name:
                fingers_num = int(touch_fingers["official_ts_ivi"])
                max_cmd = self.cmd_multi_touch_max_support(event_info["dev_num"], ave_x, ave_y, scale_x, scale_y, fingers_num)
                g_common_obj.adb_cmd_capture_msg(max_cmd)
                for i in range(1, (fingers_num + 1)):
                    resourceid_x = "com.slydroid.multitouch:id/textView%s" % view_n
                    resourceid_y = "com.slydroid.multitouch:id/textView%sa" % view_n
                    get_text1 = d(resourceId=resourceid_x).text
                    get_text2 = d(resourceId=resourceid_y).text
                    print get_text1, get_text2
                    get_text_x = int((get_text1.split("=")[1].split())[0])
                    get_text_y = int((get_text2.split("=")[1].split())[0])
                    get_text_fingers["fingers_" + str(i)] = [get_text_x, get_text_y]
                    view_n += 1
            else:
                fingers_num = int(touch_fingers["max_finger"]) + 2
                max_cmd = self.cmd_multi_touch_max_support(event_info["dev_num"], ave_x, ave_y, scale_x, scale_y, fingers_num)
                g_common_obj.adb_cmd_capture_msg(max_cmd)
                for i in range(1, 11):
                    resourceid_x = "com.slydroid.multitouch:id/textView%s" % view_n
                    resourceid_y = "com.slydroid.multitouch:id/textView%sa" % view_n
                    get_text1 = d(resourceId=resourceid_x).text
                    get_text2 = d(resourceId=resourceid_y).text
                    get_text_x = int((get_text1.split("=")[1].split())[0])
                    get_text_y = int((get_text2.split("=")[1].split())[0])
                    get_text_fingers["fingers_" + str(i)] = [get_text_x, get_text_y]
                    view_n += 1
        finally:
            print get_text_fingers
            #untouch points
            untouch_cmd = "sh /mnt/sdcard/multi_touch_event_untouch.sh %s " % (event_info["dev_num"])
            g_common_obj.adb_cmd_capture_msg(untouch_cmd)
        time.sleep(10)
        max_num = d(resourceId="com.slydroid.multitouch:id/textView1").text
        print "[INFO]Devices maximum fingers current support: %s" % max_num
        time.sleep(2)
        get_text_fingers["max_num"] = int(max_num)
        d.click((bounds["right"]) - 200, (bounds["bottom"]) - 300)
        return get_text_fingers

    def check_maximum_fingers_support(self):
        product_name = self.check_product()
        get_text_fingers = self.check_maximum_fingers_support_sendevent()
        max_num = get_text_fingers["max_num"]
        if "bxt" in product_name or "gordon_peak" in product_name or "icl_presi_kbl" in product_name \
                or "androidia" in product_name or "celadon" in product_name:
            # due to screen max support 4 fingers for ivi in official screen
            if int(max_num) == 4:
                print "[INFO]Devices [%s] maximum fingers support: %s" % (product_name, max_num)
            else:
                assert False, "[INFO]Devices [%s] maximum support is not 4 fingers" % product_name
        else:
            if int(max_num) == 10:
                print "[INFO]Devices [%s] maximum fingers support: %s" % (product_name, max_num)
            else:
                assert False, "[INFO]Devices [%s] maximum support is not 10 fingers" % product_name

    def check_multi_touch_with_coordinates(self):
        product_name = self.check_product()
        touch_fingers = self.touch_screen_fingers()
        get_text_fingers = self.check_maximum_fingers_support_sendevent()
        ave_coordinates = get_text_fingers["ave_coordinates"]
        fingers_multi = {}
        fingers_multi_ivi = {}
        ave_x = ave_coordinates[0] - 1
        ave_y = ave_coordinates[1] - 1
        if "bxt" in product_name or "gordon_peak" in product_name or "icl_presi_kbl" in product_name \
                or "androidia" in product_name or "celadon" in product_name:
            fingers_num = int(touch_fingers["official_ts_ivi"])
            ivi_dx = ave_x * 10
            #ivi_dy = ave_y - 90
            ivi_dy = ave_y - 20
            print ivi_dx, ivi_dy
            for i in range(1, fingers_num+1):
                fingers = "fingers_%s" % i
                print "[Check] finger touch coordinates dx: {} > {}".format(get_text_fingers[fingers][0], ivi_dx)
                assert get_text_fingers[fingers][0] > ivi_dx
                ivi_dx -= 50
                ivi_dx -= 50
                if "icl_presi_kbl" in product_name or "gordon_peak_cwp" in product_name \
                        or "androidia" in product_name or "celadon" in product_name:
                    print "[Check] finger touch coordinates dy: {} < {}".format((get_text_fingers[fingers][1]), ivi_dy)
                    assert (get_text_fingers[fingers][1]) < ivi_dy
                    ivi_dy += 88
                    print "------------------------------------------------------"
                    print
                else:
                    print "[Check] finger touch coordinates dy: {} < {}".format((get_text_fingers[fingers][1])*2, ivi_dy+20)
                    assert (get_text_fingers[fingers][1])*2 < ivi_dy+20
                    ivi_dy += 100
                    print "------------------------------------------------------"
                    print
        else:
            fingers_num = int(touch_fingers["max_finger"])
            for i in range(1, fingers_num+1):
                fingers_multi["fingers_"+str(i)] = [ave_x, ave_y]
                ave_x += 50
                ave_y += 50
            print "------------------------------"
            print fingers_multi
            try:
                if int(self.event_info["scale_x"]) == 1 or int(self.event_info["scale_y"]) == 1:
                    for i in range(1, fingers_num):
                        fingers = "fingers_%s" %i
                        assert get_text_fingers[fingers] == fingers_multi[fingers]
                else:
                    a = []
                    b = []
                    for i in range(1, fingers_num+1):
                        fingers = "fingers_%s" % i
                        a.append(fingers_multi[fingers][0] -get_text_fingers[fingers][0])
                    print a
                    for i in range(len(a)-1):
                        b.append(a[i+1]-a[i])
                    print b
                    for i in range(len(b)-1):
                        assert b[i+1] == b[i]
            finally:
                self.reboot_devices()

    def open_screen_pinning(self, enable=True):
        self.launch_settings("Settings")
        self.scrollable_select_text("Security")
        d(textContains="Security").click.wait()
        self.scrollable_select_text("Screen pinning")
        d(text="Screen pinning").click.wait()
        widget = d(className="android.widget.Switch")
        if widget:
            if enable:
                if not widget.checked:
                    widget.click()
            else:
                if widget.checked:
                    widget.click()

    def do_action_screen_pinning(self, sub_pinning):
        #g_common_obj.close_background_apps()
        for i in range(3):
            d.press.home()
            time.sleep(2)
            d.press.recent()
            time.sleep(3)
            if d(resourceId="com.android.systemui:id/lock_to_app_fab").exists:
                d(resourceId="com.android.systemui:id/lock_to_app_fab").click.wait()
                if (d(packageName="com.android.settings").exists or d(packageName="com.android.chrome").exists):
                    break
            else:
                d().swipe.up(steps=6)
                time.sleep(2)
                if d(resourceId="com.android.systemui:id/lock_to_app_fab").exists:
                    d(resourceId="com.android.systemui:id/lock_to_app_fab").click.wait()
                    if (d(packageName="com.android.settings").exists or d(packageName="com.android.chrome").exists):
                        break
        time.sleep(2)
        d.click(200, 300)
        time.sleep(2)
        for i in range(5):
            d.press.back()
            d.press.home()
            d.press.recent()
        d.open.notification()
        assert not d(resourceId = "com.android.systemui:id/notification_stack_scroller").exists
        if "settings" in sub_pinning:
            time.sleep(2)
            #assert (d(textContains="Display").exists and d(textContains="Sound").exists)
            assert (d(resourceId="com.android.settings:id/search").exists or d(description="Search settings").exists)
            self.stop_app("com.android.settings")
        if "chrome" in sub_pinning:
            assert d(packageName="com.android.chrome").exists
            #assert d(resourceId="com.android.chrome:id/toolbar_container").exists

        #pinning chrome app
    def chrome_deactivate_screen_pinning(self):
        self.install_artifactory_app("chrome_google", "com.android.chrome")
        self.grant_permissions_for_chrome_app()
        g_common_obj.close_background_apps()
        self.close_chrome_tabs()
        self.open_webpage("file:///mnt/sdcard/")
        self.do_action_screen_pinning("chrome")

    def touch_deactivate_screen_pinning_mode(self):
        self.open_screen_pinning(enable=True)
        try:
            self.do_action_screen_pinning("settings")
            self.chrome_deactivate_screen_pinning()
        finally:
            self.stop_app("com.android.settings")
            self.stop_app("com.android.chrome")
