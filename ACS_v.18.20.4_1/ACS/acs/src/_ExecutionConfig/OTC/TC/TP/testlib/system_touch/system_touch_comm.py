# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
from testlib.dut_init.dut_init_impl import Function
import commands
import relay08
import time
import os
import re

d = g_common_obj.get_device()
dsn = d.server.adb.device_serial()

class SystemTouchComm(object):


    def __init__(self):
        self.cfg_file = 'system_touch.conf'
        formatter = "[%(asctime)s - %(levelname)s] %(message)s"
        self.logger = Logger("System_Touch", formatter)
        self.relay = None
        self.official_key = 'name:     "eGalax Inc. eGalaxTouch EXC7200-7368v1.010          "'
        self.td_key = 'name:     "Weida Hi-Tech'
        #self.func = Function()

    def check_adb_connection(self):
        '''
        check adb connection status.
        '''
        result = g_common_obj.adb_cmd_capture_msg("echo hello").find("hello")
        if (result != -1):
            self.logger.info("%s : adb connected" % dsn)
            return True
        else:
            self.logger.error("%s : adb disconnected" % dsn)
            #assert False, "Test Result : Failed"
            return False

    def download_artifactory_content(self, option_file):
        from testlib.util.repo import Artifactory
        cfg = TestConfig().read(self.cfg_file, "artifactory")
        arti_obj = Artifactory(cfg.get("location"))
        ret_file = arti_obj.get(cfg.get(option_file))
        return ret_file

    def install_artifactory_app(self, option_file, package):
        cmd = "pm list package %s" % package
        check_str = "package:" + package
        msg_list = g_common_obj.adb_cmd_capture_msg(cmd).splitlines()
        if check_str in msg_list:
            print "[info]---App %s already installed." % package
            return
        print "[info]---Install %s." % package
        ret_file = self.download_artifactory_content(option_file)
        assert os.path.isfile(ret_file)
        g_common_obj.adb_cmd_common("install -r %s" % ret_file)
        time.sleep(2)
        msg_list = g_common_obj.adb_cmd_capture_msg(cmd).splitlines()
        assert check_str in msg_list, "Install app failed!"

    def push_artifactory_resource(self, option_file, remote_dir="/mnt/sdcard/Movies/"):
        cfg = TestConfig().read(self.cfg_file, "artifactory")
        res_file = cfg.get(option_file).split("/")[-1]
        cmd = "'ls %s 2>/dev/null'| grep %s" % (remote_dir, res_file)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        if msg != "":
            print "[info]---res file %s already existed." % res_file
            return os.path.join(remote_dir, res_file)
        ret_file = self.download_artifactory_content(option_file)
        assert os.path.isfile(ret_file)
        g_common_obj.adb_cmd("mkdir -p %s" % remote_dir)
        g_common_obj.adb_cmd_common("push %s %s" % (ret_file, remote_dir), 600)
        return os.path.join(remote_dir, res_file)

    def check_product(self):
        cmd = "getprop ro.product.name"
        product = g_common_obj.adb_cmd_capture_msg(cmd)
        return product

    def check_version(self):
        cmd = "getprop ro.build.version.incremental"
        version = g_common_obj.adb_cmd_capture_msg(cmd)
        return version

    def check_build_release(self):
        cmd = "getprop ro.build.version.release"
        release = g_common_obj.adb_cmd_capture_msg(cmd)
        return release

    def unlock_screen(self):
        if d(description="Unlock").exists:
            d.press("menu")

    def single_swipe_to_app_list(self):
        info = d.info
        x = info["displayWidth"] / 2
        y = info["displayHeight"]
        d.swipe(x, y, x, 0)
        time.sleep(2)

    def unlock_screen_comm(self):
        self.single_swipe_to_app_list()
        if d(resourceId="com.android.systemui:id/notification_stack_scroller").exists:
            self.unlock_screen()
            self.single_swipe_to_app_list()

    def adb_root(self):
        g_common_obj.root_on_device()

    def check_enter_to_fastboot_mode(self):
        for i in range(3):
            g_common_obj.adb_cmd_common("reboot fastboot")
            time.sleep(20)
            fastboot_dvs = commands.getoutput("fastboot devices")
            if fastboot_dvs != '':
                print "[Info]------Fastboot devices: {}".format(fastboot_dvs)
                break
            else:
                if self.check_adb_connection() is True:
                    print "[info]--- adb connection success"
                else:
                    os.system("fastboot reboot")
                    time.sleep(20)

    def remount_disable_dm_verity_for_OMR1(self):
        ret_file = self.download_artifactory_content("remount_file")
        print ret_file
        self.check_enter_to_fastboot_mode()
        os.system("fastboot flashing unlock")
        time.sleep(1)
        os.system("fastboot flash vbmeta_a {}".format(ret_file))
        time.sleep(1)
        os.system("fastboot flashing lock")
        time.sleep(1)
        os.system("fastboot reboot")
        time.sleep(10)
        if self.check_boot_completed() is True:
            print "[info]--- reboot completed"
            time.sleep(3)
            self.adb_root()

    def check_adb_remount(self):
        self.adb_root()
        for i in range(3):
            remount_str = g_common_obj.adb_cmd_common("remount")
            if "remount succeeded" in remount_str:
                print "[Info]------DUT remount succeeded..."
                break
            else:
                self.remount_disable_dm_verity_for_OMR1()

    def start_RPCServer(self):
        """
        Start RPC server
        """
        self.adb_root()
        # self.func.push_uiautomator_jar()
        from testlib.security.security_impl import SecurityImpl
        self.securityImpl = SecurityImpl()
        self.securityImpl.start_RPCServer()

    def stop_app(self, app_package):
        g_common_obj.stop_app_am(app_package)

    def clear_app_data(self, app_package):
        g_common_obj.adb_cmd("pm clear %s"% app_package)
        time.sleep(2)

    def check_deivces_app_exist(self, package):
        cmd = "pm list package %s" % package
        check_str = "package:" + package
        msg_list = g_common_obj.adb_cmd_capture_msg(cmd).splitlines()
        if check_str in msg_list:
            print "[info]---App %s already installed." % package
            return True
        else:
            return False

    def set_screen_status(self, status):
        d.screen(status)

    def setting_ON_OFF(self, _text, enable=True):
        widget = d(text=_text).right(className="android.widget.Switch")
        if widget:
            if enable:
                if not widget.checked:
                    widget.click()
            else:
                if widget.checked:
                    widget.click()
        status = widget.checked
        return status

    def scrollable_select_text(self, sub_text):
        time.sleep(2)
        if d(textContains=sub_text).exists:
            return True
        build_release = self.check_build_release()
        if not d(textContains=sub_text).exists:
            if "7." in build_release:
                d().swipe.up(steps=2)  # resolve N build issue for app list
            d(scrollable=True).scroll.vert.to(textContains=sub_text)
        assert d(scrollable=True).scroll.vert.to(textContains=sub_text)

    def launch_settings(self, settings_action = "Settings"):
        action_map = {
            "Settings": "android.settings.SETTINGS",
            "Wi-Fi": "android.settings.WIFI_SETTINGS",
            "Bluetooth": "android.settings.BLUETOOTH_SETTINGS",
            "More": "android.settings.WIRELESS_SETTINGS",
            "Display": "android.settings.DISPLAY_SETTINGS",
            "Apps": "android.settings.MANAGE_ALL_APPLICATIONS_SETTINGS",
            "Battery": "android.intent.action.POWER_USAGE_SUMMARY",
            "Location": "android.settings.LOCATION_SOURCE_SETTINGS",
            "Security": "android.settings.SECURITY_SETTINGS",
            "Date": "android.settings.DATE_SETTINGS",
            "Accessibility": "android.settings.ACCESSIBILITY_SETTINGS",
            "Developer options": "android.settings.APPLICATION_DEVELOPMENT_SETTINGS",
            "Screen lock": "com.android.settings.SETUP_LOCK_SCREEN",
            "Battery saver": "android.settings.BATTERY_SAVER_SETTINGS",
            "About tablet" : "android.settings.DEVICE_INFO_SETTINGS"
        }
        self.stop_app("com.android.settings")
        g_common_obj.adb_cmd("am start -a %s" % action_map[settings_action])
        time.sleep(2)

    def launch_developer_option(self):
        g_common_obj.stop_app_am("com.android.settings")
        devoption_cmd = "am start -a android.settings.APPLICATION_DEVELOPMENT_SETTINGS"
        g_common_obj.adb_cmd_capture_msg(devoption_cmd)

    def choose_screen_lock(self, screen_lock_kind):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.scrollable_select_text("Security")
        d(textContains="Security").click.wait()
        time.sleep(2)
        d(text="Screen lock").click()
        d(textContains=screen_lock_kind).click()
        if d(textContains="Require").exists:
            d(textContains="Require").click()
            time.sleep(2)
            if d(text="YES").exists:
                d(text="YES").click.wait()
            time.sleep(1)
            if d(resourceId="android:id/button1").exists:
                d(resourceId="android:id/button1").click.wait()
            if d(text="OK").exists:
                d(text="OK").click()
        else:
            if d(text="YES").exists:
                d(text="YES").click()
            if d(resourceId="com.android.settings:id/encrypt_require_password").exists:
                d(resourceId="com.android.settings:id/encrypt_require_password").click()
        if d(text="Continue").exists:
            d(text="Continue").click()

    def get_event_info(self):
        '''
            1. get screen size ,  2. get touch event num , 3. get 35, 36, max var, 4. calc size scale
            other devices: 1:1,  KEY (0001): 014a
        '''
        cmd = "dumpsys display | grep mDefaultViewport"
        msg_info = g_common_obj.adb_cmd_capture_msg(cmd)
        p = re.compile("deviceWidth=(\d+), deviceHeight=(\d+)")
        s = p.search(msg_info)
        width = int(s.group(1))
        height = int(s.group(2))
        p1 = re.compile("0035.+max\s(\d+)")
        p2 = re.compile("0036.+max\s(\d+)")
        event_info_1 = {}
        event_info_2 = {}
        event_info_3 = {}
        touch_info_list = []
        cmd = "getevent -p"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        product_name = self.check_product()
        msg_event_list = msg.split("input props:")
        for i in range(len(msg_event_list)):
            if "bxt" in product_name or "gordon_peak" in product_name \
                    or "icl_presi_kbl" in product_name or "celadon" in product_name:
                if self.official_key in msg_event_list[i] or self.td_key in msg_event_list[i]:
                    touch_info_list.append(msg_event_list[i])
            else:
                if "KEY (0001): 014a" in msg_event_list[i]:
                    touch_info_list.append(msg_event_list[i])
        conn_num = len(touch_info_list)
        for k in range(conn_num):
            for line in touch_info_list[k].splitlines():
                line.strip()
                if line.startswith("add device"):
                    # S.isdigit() False: t1, True:12
                    if k == 0:
                        num_t = line[-2:]
                        if num_t.isdigit() is False:
                            event_info_1["dev_num"] = line[-1]
                        else:
                            event_info_1["dev_num"] = line[-2:]
                    if k == 1:
                        num_t = line[-2:]
                        if num_t.isdigit() is False:
                            event_info_2["dev_num"] = line[-1]
                        else:
                            event_info_2["dev_num"] = line[-2:]
                    if k == 2:
                        num_t = line[-2:]
                        if num_t.isdigit() is False:
                            event_info_3["dev_num"] = line[-1]
                        else:
                            event_info_3["dev_num"] = line[-2:]
                elif line.startswith(" "):
                    s1 = p1.search(line)
                    if s1:
                        max_53 = int(s1.group(1))
                        continue
                    s2 = p2.search(line)
                    if s2:
                        max_54 = int(s2.group(1))
                        break
                else:
                    continue
            if k == 0:
                event_info_1["scale_x"] = 1.0 * max_53 / width
                event_info_1["scale_y"] = 1.0 * max_54 / height
            if k == 1:
                event_info_2["scale_x"] = 1.0 * max_53 / width
                event_info_2["scale_y"] = 1.0 * max_54 / height
            if k == 2:
                event_info_3["scale_x"] = 1.0 * max_53 / width
                event_info_3["scale_y"] = 1.0 * max_54 / height
        if event_info_3 != {}:
            print event_info_1, "\n", event_info_2, "\n", event_info_3
            print "***********************************************"
            return event_info_1, event_info_2, event_info_3
        else:
             if event_info_2 != {}:
                 print event_info_1, "\n",  event_info_2
                 print "***********************************************"
                 return event_info_1, event_info_2
             else:
                 print event_info_1
                 print "***********************************************"
                 return event_info_1

    def push_send_event_script(self, script_name):
        self.adb_root()
        rm_cmd = "rm -rf /mnt/sdcard/%s" % script_name
        #g_common_obj.adb_cmd_capture_msg(rm_cmd)
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        push_to_dir = "/mnt/sdcard/"
        cmd = "push %s %s" % (script_path, push_to_dir)
        g_common_obj.adb_cmd_common(cmd)

    def cmd_draw_pattern_lock(self, dev_num, var1, var2, var3, var4):
        #self.push_send_event_script("draw_pattern_lock.sh")
        dut_path = os.path.join("/mnt/sdcard/", "draw_pattern_lock.sh")
        cmd = "sh {} {} {} {} {} {}".format(dut_path, dev_num, var1, var2, var3, var4)
        return cmd

    def cmd_touch_click_sendevent(self, dev_num, var1, var2):
        #self.push_send_event_script("touch_click_sendevent.sh")
        dut_path = os.path.join("/mnt/sdcard/", "touch_click_sendevent.sh")
        cmd = "sh {} {} {} {} ".format(dut_path, dev_num, var1, var2)
        return cmd

    def cmd_multi_touch_max_support(self, dev_num, var1, var2, var3, var4, max_num):
        #self.push_send_event_script("multi_touch_max_support.sh")
        dut_path = os.path.join("/mnt/sdcard/", "multi_touch_max_support.sh")
        cmd = "sh {} {} {} {} {} {} {}".format(dut_path, dev_num, var1, var2, var3, var4, max_num)
        return cmd

    def cmd_touch_gesture_swipe_horizontal_sendevent(self, dev_num, var1, var2, var3, var4):
        #self.push_send_event_script("touch_gesture_swipe_horizontal_sendevent.sh")
        dut_path = os.path.join("/mnt/sdcard/", "touch_gesture_swipe_horizontal_sendevent.sh")
        cmd = "sh {} {} {} {} {} {}".format(dut_path, dev_num, var1, var2, var3, var4)
        return cmd

    def cmd_touch_gesture_swipe_vertical_down(self, dev_num, var1, var2, var3, var4):
        #self.push_send_event_script("touch_gesture_swipe_vertical_down.sh")
        dut_path = os.path.join("/mnt/sdcard/", "touch_gesture_swipe_vertical_down.sh")
        cmd = "sh {} {} {} {} {} {}".format(dut_path, dev_num, var1, var2, var3, var4)
        return cmd

    def cmd_touch_gesture_swipe_vertical_up(self, dev_num, var1, var2, var3, var4):
        #self.push_send_event_script("touch_gesture_swipe_vertical_up.sh")
        dut_path = os.path.join("/mnt/sdcard/", "touch_gesture_swipe_vertical_up.sh")
        cmd = "sh {} {} {} {} {} {}".format(dut_path, dev_num, var1, var2, var3, var4)
        return cmd

    def draw_pattern_lock(self):
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if d(resourceId="com.android.settings:id/lockPattern").exists:
            bounds = d(resourceId="com.android.settings:id/lockPattern").bounds
        elif d(resourceId="com.android.systemui:id/lockPatternView").exists:
            bounds = d(resourceId="com.android.systemui:id/lockPatternView").bounds
        else:
            return
        margin = (bounds["right"] - bounds["left"]) / 6
        left = int((bounds["left"] + margin) * self.event_info["scale_x"])
        top = int((bounds["top"] + margin) * self.event_info["scale_y"])
        right = int((bounds["right"] - margin) * self.event_info["scale_x"])
        bottom = int((bounds["bottom"] - margin) * self.event_info["scale_y"])
        cmd = self.cmd_draw_pattern_lock(self.event_info["dev_num"], left, top, right, bottom)
        g_common_obj.adb_cmd(cmd)

    def set_screen_lock_pattern(self):
        self.choose_screen_lock("Pattern")
        print "[info]--- Set screen lock pattern"
        self.draw_pattern_lock()
        time.sleep(2)
        d(resourceId="com.android.settings:id/footerRightButton").click()
        time.sleep(2)
        self.draw_pattern_lock()
        time.sleep(2)
        d(resourceId="com.android.settings:id/footerRightButton").click()
        # Text: Done
        product = self.check_product()
        if "gordon_peak" in product:
            if not d(text="DONE").exists:
                return False
            d(resourceId="com.android.settings:id/redaction_done_button").click.wait()
            print "[Completed]------ Set screen lock password completed"
            return True
        else:
            if not d(text="Done").exists:
                return False
            d(resourceId="com.android.settings:id/next_button").click.wait()
            print "[Completed]------ Set screen lock Pattern completed"
            return True

    def check_set_screen_lock_success(self):
        print "[Verify]------Verify Set up screen lock---"
        set_passwd_pin_reId = "com.android.settings:id/password_entry"
        set_pattern_reId = "com.android.settings:id/lockPattern"
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.scrollable_select_text("Security")
        d(textContains="Security").click.wait()
        time.sleep(2)
        d(text="Screen lock").click()
        time.sleep(2)
        if d(resourceId=set_passwd_pin_reId).exists or d(resourceId=set_pattern_reId).exists:
            print "[Verify]------Verify Set up screen lock < Success >"
            return True
        else:
            return False

    def unlock_screen_pattern(self):
        print "[info]--- Unlock screen pattern"
        product = self.check_product()
        if "s3gr" in product:
            d(resourceId="com.android.systemui:id/lock_icon").drag.to(
                resourceId="com.android.systemui:id/clock_view")
        else:
            d.press("menu")
        self.draw_pattern_lock()
        time.sleep(2)

    def remove_screen_lock_pattern(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.scrollable_select_text("Security")
        d(textContains="Security").click.wait()
        time.sleep(2)
        print "[info]--- Remove screen lock pattern"
        d(text="Screen lock").click()
        time.sleep(5)
        self.draw_pattern_lock()
        time.sleep(2)
        if d(text="Swipe").exists:
            d(text="Swipe").click()
        time.sleep(1)
        if d(resourceId="android:id/button1").exists:
            d(resourceId="android:id/button1").click()
        time.sleep(3)
        if d(text="Screen lock").exists and d(text="Swipe").exists:
            print "[Completed]------ Remove screen lock pattern completed"
            return True
        else:
            return False

    def check_remove_screen_lock_success(self):
        print "[Verify]------Verify Remove screen lock---"
        if d(text="Device security").exists and d(text="Screen lock").exists:
            time.sleep(2)
        else:
            g_common_obj.launch_app_am("com.android.settings", ".Settings")
            self.scrollable_select_text("Security")
            d(textContains="Security").click.wait()
            time.sleep(2)
        if d(text="Screen lock").exists and d(textContains="Swipe").exists:
            print "[Verify]------Verify Remove screen lock < Success >"
            return True
        else:
            print "[Verify]------Verify Remove screen lock < Fail >, Need redo remove action"
            return False

    def rotate(self, orientation):
        d.orientaton = orientation
        time.sleep(2)
        assert d.orientaton == orientation

    def check_home_ui_user_safely_O(self):
        if d(text="Drive safely").exists or d(text="Owner").exists:
            if d(text="Owner").exists:
                d(text="Owner").click.wait()
            if d(textContains="START DRIVING").exists:
                d(textContains="START DRIVING").click.wait()
            #assert not d(text="Owner").exists
        else:
            if d(textContains="START DRIVING").exists:
                d(textContains="START DRIVING").click.wait()
            d.press.home()
            g_common_obj.adb_cmd_capture_msg("input tap 310 1007")
            g_common_obj.adb_cmd_capture_msg("input tap 959 1007")
            g_common_obj.adb_cmd_capture_msg("input tap 1280 1296")
            #assert not d(text="Owner").exists

    def root(self):
        g_common_obj.adb_cmd_common("root")
        for i in range(5):
            message = g_common_obj.adb_cmd_common("root")
            if "adbd is already running as root" in message:
                print "[info]--- Root success"
                return
            time.sleep(2)
        assert False, "Root failed!"

    def check_boot_completed(self):
        g_common_obj.adb_cmd_capture_msg("getprop sys.boot_completed")
        for i in range(30):
            time.sleep(5)
            message = g_common_obj.adb_cmd_capture_msg("getprop sys.boot_completed")
            if '1' == message:
                return True
        else:
            return False

    def reboot_devices(self):
        for i in range(5):
            g_common_obj.adb_cmd_common("reboot")
            time.sleep(30)
            if self.check_boot_completed() is True:
                print "[info]--- reboot completed"
                time.sleep(3)
                # self.unlock_screen() #rpc server always failed
                self.boot_up_completed_skip_boot_ui()
                self.adb_root()
                return
        assert False, "reboot failed!"

    def press_power_button(self):
        power_key_cmd = "input keyevent 26"
        g_common_obj.adb_cmd_capture_msg(power_key_cmd)

    def long_press_power_buttou(self):
        power_key_cmd = "input keyevent --longpress 26"
        for i in range(10):
            time.sleep(2)
            g_common_obj.adb_cmd_capture_msg(power_key_cmd)
            if d(text="Power off").exists:
                d(text="Power off").click.wait()
                print "[info]--- power off devices"
                time.sleep(30)
                break

    def power_on(self):
        for i in range(5):
            g_common_obj.adb_cmd_common("reboot")
            time.sleep(20)
            if self.check_boot_completed() is True:
                print "[info]--- power on completed"
                time.sleep(5)
                # self.unlock_screen()
                self.unlock_screen_comm()
                self.adb_root()
                return
        assert False, "power on failed!"

    def enter_screen_off(self):
        g_common_obj.adb_cmd_common("logcat -c")
        self.press_power_button()
        time.sleep(3)
        log_str = "dumpsys display |grep mScreenState"
        state_info = g_common_obj.adb_cmd_capture_msg(log_str)
        print state_info
        if "ON" in state_info:
            self.press_power_button()
        if "OFF" in state_info:
            print "Devices screen is Off"

    def check_screen_off(self):
        log_str = "dumpsys display |grep mScreenState"
        state_info = g_common_obj.adb_cmd_capture_msg(log_str)
        print state_info
        if "ON" in state_info:
            assert False, "Devices screen is ON"

    def screen_turn_off_option_AIA(self, target="Sleep"):
        # For AIA 2.0, androidia and celadon
        # press power key--> power off, restart, sleep UI
        self.press_power_button()
        if target == "Power off":
            if d(text="Power off").exists:
                d(text="Power off").click.wait()
                time.sleep(30)
            else:
                d(text="Power off").click.wait()
                time.sleep(30)
        elif target == "Restart":
            if d(text="Restart").exists:
                d(text="Restart").click.wait()
                time.sleep(30)
            else:
                d(text="Restart").click.wait()
                time.sleep(30)
        elif target == "Sleep":
            if d(text="Sleep").exists:
                d(text="Sleep").click.wait()
                time.sleep(10)
            else:
                d(text="Sleep").click.wait()
                time.sleep(10)
        else:
            print 123
        self.check_screen_off()
        self.set_screen_status("on")
        self.unlock_screen_comm()

    def screen_turn_on_off_comm(self):
        product_name = self.check_product()
        if "bxt" in product_name or "gordon_peak" in product_name or "icl_presi_kbl" in product_name:
            self.check_enter_s0i3_state_for_ivi()
        elif "androidia" in product_name or "celadon" in product_name:
            self.screen_turn_off_option_AIA("Sleep")
        else:
            self.enter_screen_off()
            self.check_screen_off()
            self.set_screen_status("on")
            self.unlock_screen_comm()

    def boot_up_completed_skip_boot_ui(self):
        self.check_boot_completed()
        self.start_RPCServer()
        build_release = self.check_build_release()
        print "[INFO]Build release: %s" % build_release
        if "6." in build_release or "7." in build_release:
            self.unlock_screen_comm()
        elif "8." in build_release:
            time.sleep(20)
            self.unlock_screen_comm()
            self.check_home_ui_user_safely_O()
        else:
            self.unlock_screen_comm()

    def get_relay08_obj(self):
        self.relay = relay08.get_relay_obj()

    def press_power_key(self, duration=4):
        if not self.relay:
            self.relay = relay08.get_relay_obj()
        self.relay.press_power_key(duration)

    def get_s0i3_suspend_status(self):
        cmd_str = "cat /d/suspend_stats  | grep success"
        msg = g_common_obj.adb_cmd_capture_msg(cmd_str)
        success = msg.split()[1]
        print "[INFO]--- S0i3 suspend stat:", success
        return int(success)

    def check_screen_off_or_power_off_for_ivi(self):
        # no devices, adb connection failed for ivi
        time.sleep(80)
        if self.check_adb_connection() is False:
            print "[INFO]---press ignition key success"
        else:
            #skip first enter to S3 to long time, develop take for this not issue
            self.press_power_key(0.5)
            time.sleep(15)
            self.press_power_key(0.5)
            time.sleep(100)
            if self.check_adb_connection() is False:
                print "[INFO]---press ignition key success"
            else:
                assert False, "[INFO]---press ignition key Failed, Screen is ON"

    def check_enter_s0i3_state_for_ivi(self, sleep_time=20):
        self.adb_root()
        s3_pre = self.get_s0i3_suspend_status()
        self.press_power_key(0.5)
        time.sleep(15)
        try:
            self.check_screen_off_or_power_off_for_ivi()
        finally:
            self.press_power_key(0.5)
            time.sleep(3)
            self.boot_up_completed_skip_boot_ui()
        s3_post = self.get_s0i3_suspend_status()
        assert s3_pre < s3_post
        if s3_pre < s3_post:
            return True
        return False

    def power_off_on_from_os_by_ignition(self):
        self.press_power_key(10)
        time.sleep(10)
        try:
            self.check_screen_off_or_power_off_for_ivi()
        finally:
            self.press_power_key(0.5)
            time.sleep(10)
            self.boot_up_completed_skip_boot_ui()
        self.adb_root()
        s3_post = self.get_s0i3_suspend_status()
        assert s3_post == 0

    def check_home_ui_skip_got_it(self):
        if d(textContains="Welcome").exists:
            if d(textContains="GOT IT").exists:
                d(textContains="GOT IT").click.wait()
        if d(text="OK").exists:
            d(text="OK").click.wait()
