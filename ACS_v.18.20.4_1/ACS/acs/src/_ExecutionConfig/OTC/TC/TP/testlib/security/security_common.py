# -*- coding: utf-8 -*-
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
import relay08
import commands
import time
import os
import re

d = g_common_obj.get_device()
dsn = d.server.adb.device_serial()

class SecurityCommon(object):

    def __init__(self):
        formatter = "[%(asctime)s - %(levelname)s] %(message)s"
        self.logger = Logger("Security", formatter)
        self.relay = None
        self.cfg_file = "tests.tablet.security.conf"
        self.conf_path = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), self.cfg_file)
        self.official_key = 'name:     "eGalax Inc. eGalaxTouch EXC7200-7368v1.010          "'
        self.td_key = 'name:     "Weida Hi-Tech'

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

    def check_memery_2G_4G_8G_board(self):
        cmd = 'free -h'
        info_free = g_common_obj.adb_cmd_capture_msg(cmd)
        info = info_free.split()
        mem_index = info.index('Mem:')
        mem = float(info[mem_index+1].strip("G"))
        if 1 < mem < 2.1:
            print "[Mem_Info]------This board is 2G Memery"
            return "2G"
        elif 2.1 < mem < 4.1:
            print "[Mem_Info]------This board is 4G Memery"
            return "4G"
        elif mem > 4.1:
            print "[Mem_Info]------This board is 8G Memery"
            return "8G"
        else:
            print "[Info]------This board is {} Memery".format(mem)

    def check_user_userdebug_image_type(self, build_type):
        # getprop ro.build.type --> user or userdebug
        cmd = "getprop ro.build.type"
        image_flavor = g_common_obj.adb_cmd_capture_msg(cmd)
        if image_flavor == "user" == build_type:
            print "[Info]------This build is <{}> build".format(image_flavor)
            return image_flavor
        elif image_flavor == "userdebug" == build_type:
            print "[Info]------This build is <{}> build".format(image_flavor)
            return image_flavor
        else:
            raise EnvironmentError, "[Info]------This build is <{}> build, pls flash correct type's build".format(image_flavor)

    def check_pre_build_test_or_ebimage(self):
        dut_file = "adb -s {} shell ls /mnt/user/0/primary/draw_pattern_lock.sh".format(dsn)
        print dut_file
        file_info = commands.getoutput(dut_file)
        #file_info = g_common_obj.adb_cmd_capture_msg(dut_file)
        if "No such file or directory" in file_info:
            print "[Info]------This build is test build, please continue..."
        else:
            raise EnvironmentError, "[Error]------build not test image, pls switch to eb image test case...\n {}".format(dut_file)

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

    def home_to_app_list(self):
        d.press.home()
        if d(description="Apps").exists:
            d(description="Apps").click.wait()
        else:
            self.single_swipe_to_app_list()

    def adb_root(self):
        g_common_obj.root_on_device()

    def root(self):
        g_common_obj.adb_cmd_common("root")
        for i in range(5):
            message = g_common_obj.adb_cmd_common("root")
            if "adbd is already running as root" in message:
                print "[info]--- Root success"
                return
            time.sleep(2)
        assert False, "Root failed!"

    def setSleepMode(self, Mode):
        modelist = Mode.split()
        t = int(modelist[0]) * 1000
        if modelist[1].startswith('minute'):
            t *= 60
        cmd = "dumpsys power | grep 'mScreenOffTimeoutSetting='"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        msglist =msg.split('=')
        if int(msglist[1]) == t:
            return
        self.launch_settings("Display")
        for i in range(5):
            if d(text="Sleep").exists:
                d(text = "Sleep").click()
                break
            time.sleep(2)
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.to(text=Mode)
        for i in range(5):
            if d(text = Mode).exists:
                d(text = Mode).click()
                break
            time.sleep(2)

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
        print "[RunTest]: dut start RPC server"
        self.adb_root()
        try:
            self.push_uiautomator_jar_comm()
        except Exception, e:
            print Exception, e
            if self.check_adb_connection() is True:
                print "[info]--- adb connection success"
                time.sleep(8)

    def push_uiautomator_jar_comm(self):
        # put bundle.jar and uiautomator-stub.jar to the right place
        print "Check RPC server ..."
        release_version = g_common_obj.adb_cmd_capture_msg("getprop | grep ro.build.version.release")
        print release_version
        if os.path.exists("/usr/local/lib/python2.7/dist-packages/uiautomator/libs"):
            jar_path = "/usr/local/lib/python2.7/dist-packages/uiautomator/libs"
        else:
            jar_path = "/usr/lib/python2.7/dist-packages/uiautomator/libs"
        bundle_path = jar_path + "/bundle.jar"
        uiautomator_path = jar_path + "/uiautomator-stub.jar"
        push_bundle_jar = " -s " + dsn + " push " + bundle_path + " /data/local/tmp"
        push_uiautomator_jar = " -s " + dsn + " push " + uiautomator_path + " /data/local/tmp"
        for i in range(5):
            time.sleep(5)
            cmd = " 'ls {} 2>/dev/null'| grep {}".format("/data/local/tmp", "uiautomator-stub.jar")
            msg = g_common_obj.adb_cmd_capture_msg(cmd)
            if msg != "":
                break
            print "push jar start ========"
            self.adb_root()
            g_common_obj.adb_cmd_common(push_bundle_jar)
            g_common_obj.adb_cmd_common(push_uiautomator_jar)
            print "push jar end ========="
            time.sleep(5)
        if os.system("adb shell ps |grep uiautomator > /dev/null 2>&1") == 0:
            print "PS check, RPC server started"
            return
        else:
            from uiautomator import device as d
            if d.info:
                print "import uiauto check, RPC server started"
                return
            else:
                raise Exception('RPC server start failed')

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
        if not d(textContains=sub_text).exists:
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

    def open_security_in_settings(self):
        if d(text="Device security").exists and d(text="Screen lock").exists:
            time.sleep(2)
            return
        else:
            g_common_obj.launch_app_am("com.android.settings", ".Settings")
            self.scrollable_select_text("Security")
            d(textContains="Security").click.wait()
            time.sleep(2)
            if d(textContains="Screen lock").exists:
                return
            else:
                g_common_obj.launch_app_am("com.android.settings", ".Settings")
                self.scrollable_select_text("Security")
                d(textContains="Security").click.wait()
                time.sleep(2)
                assert d(textContains="Screen lock").exists

    def choose_screen_lock(self, screen_lock_kind):
        self.open_security_in_settings()
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

    def choose_screen_lock_data_not_encrypt(self, screen_lock_kind):
        self.open_security_in_settings()
        d(text="Screen lock").click()
        d(text=screen_lock_kind).click()
        if d(text="No thanks").exists:
            d(text="No thanks").click()
        if d(text="NO").exists:
            d(text="NO").click()
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
            if "bxt" in product_name or "gordon_peak" in product_name or "icl_presi_kbl" in product_name \
                    or "androidia" in product_name or "celadon" in product_name:
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
        g_common_obj.adb_cmd_capture_msg("mkdir /mnt/user/0/primary/")
        rm_cmd = "rm -rf /mnt/user/0/primary/%s" % script_name
        #g_common_obj.adb_cmd_capture_msg(rm_cmd)
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", script_name)
        push_to_dir = "/mnt/user/0/primary/"
        cmd = "push %s %s" % (script_path, push_to_dir)
        g_common_obj.adb_cmd_common(cmd)

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
        cmd = "sh /mnt/user/0/primary/draw_pattern_lock.sh %s %s %s %s %s" % (
        self.event_info["dev_num"], left, top, right, bottom)
        g_common_obj.adb_cmd(cmd)

    def draw_pattern_lock_incorrect(self, draw_point_top = 3, draw_point_left = 1):
        #(draw_point_top, draw_point_left), (5, 5)==1 ,(3, 5)==2, (1,5)==3
        if not hasattr(self, "event_info"):
            self.event_info = self.get_event_info()
        if d(resourceId="com.android.settings:id/lockPattern").exists:
            bounds = d(resourceId="com.android.settings:id/lockPattern").bounds
        elif d(resourceId="com.android.systemui:id/lockPatternView").exists:
            bounds = d(resourceId="com.android.systemui:id/lockPatternView").bounds
        else:
            return
        margin = (bounds["right"] - bounds["left"]) / 6
        left = int((bounds["left"] + margin*draw_point_left) * self.event_info["scale_x"])
        top = int((bounds["top"] + margin*draw_point_top) * self.event_info["scale_y"])
        right = int((bounds["right"] - margin) * self.event_info["scale_x"])
        bottom = int((bounds["bottom"] - margin) * self.event_info["scale_y"])
        cmd = "sh /mnt/user/0/primary/draw_pattern_lock.sh %s %s %s %s %s" % (
        self.event_info["dev_num"], left, top, right, bottom)
        g_common_obj.adb_cmd(cmd)

    def set_screen_lock_pattern(self):
        self.push_send_event_script("draw_pattern_lock.sh")
        self.choose_screen_lock("Pattern")
        #self.choose_screen_lock_data_not_encrypt("Pattern")
        print "[Info]------ Set screen lock pattern"
        self.draw_pattern_lock()
        time.sleep(2)
        d(resourceId="com.android.settings:id/footerRightButton").click()
        time.sleep(2)
        self.draw_pattern_lock()
        time.sleep(2)
        d(resourceId="com.android.settings:id/footerRightButton").click()
        time.sleep(4)
        #Text: Done
        product = self.check_product()
        if "gordon_peak" in product or "androidia" in product or "celadon" in product:
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

    def unlock_screen_lock_pattern(self, try_input_times):
        boot_ui = "com.android.settings:id/lockPattern"
        lock_ui_in_mos = "com.android.systemui:id/lockPatternView"
        print "[Info]------ Unlock screen pattern"
        self.boot_up_completed_skip_boot_ui()
        if d(resourceId="com.android.systemui:id/notification_stack_scroller").exists:
            self.boot_up_completed_skip_boot_ui()
        time.sleep(2)
        self.adb_root()
        self.push_send_event_script("draw_pattern_lock.sh")
        for i in range(try_input_times):
            if d(resourceId=lock_ui_in_mos).exists:
                self.draw_pattern_lock()
            time.sleep(3)
            if d(resourceId=boot_ui).exists:
                self.draw_pattern_lock()
            time.sleep(3)
        time.sleep(2)
        if d(resourceId=boot_ui).exists or d(resourceId=lock_ui_in_mos).exists:
            return False
        print "[Completed]------ Unlock screen lock pattern completed"
        return True

    def unlock_screen_lock_incorrect_pattern(self, try_input_times):
        boot_ui = "com.android.settings:id/lockPattern"
        lock_ui_in_mos = "com.android.systemui:id/lockPatternView"
        print "[Info]------ Unlock screen incorrect pattern"
        product = self.check_product()
        if "gordon_peak" in product:
            self.check_home_ui_user_safely_O(2)
        else:
            self.unlock_screen()
            self.single_swipe_to_app_list()
        time.sleep(2)
        self.push_send_event_script("draw_pattern_lock.sh")
        for i in range(try_input_times):
            if d(resourceId=lock_ui_in_mos).exists:
                self.draw_pattern_lock_incorrect()
            time.sleep(3)
            if d(resourceId=boot_ui).exists:
                self.draw_pattern_lock_incorrect()
            time.sleep(3)
        time.sleep(2)
        if not (d(resourceId=boot_ui).exists or d(resourceId=lock_ui_in_mos).exists):
            return False
        if d(textContains="To unlock your phone, turn it off and then on").exists:
            print "[Info]------To unlock your phone, turn it off and then on ---exists"
            return True
        else:
            print "[Error]---'To unlock your phone, turn it off and then on'--- not exists "
            return False

    def remove_screen_lock_pattern(self):
        try:
            self.open_security_in_settings()
            print "[Info]------ Remove screen lock pattern"
            d(text="Screen lock").click()
            time.sleep(3)
            self.draw_pattern_lock()
            time.sleep(2)
            if d(text="Swipe").exists:
                d(text="Swipe").click()
            time.sleep(1)
            if d(resourceId="android:id/button1").exists:
                d(resourceId="android:id/button1").click()
        finally:
            time.sleep(3)
            if d(text="Screen lock").exists and d(text="Swipe").exists:
                print "[Completed]------ Remove screen lock pattern completed"
                return True
            else:
                return False

    def set_screen_lock_password(self, password):
        self.choose_screen_lock("Password")
        time.sleep(2)
        print "[Info]------ Set screen lock password: %s" % password
        d(resourceId="com.android.settings:id/password_entry").set_text(password)
        d(resourceId="com.android.settings:id/next_button").click()
        time.sleep(1)
        d(resourceId="com.android.settings:id/password_entry").set_text(password)
        d(resourceId="com.android.settings:id/next_button").click()
        time.sleep(4)
        # Text: Done
        product = self.check_product()
        if "gordon_peak" in product or "androidia" in product or "celadon" in product:
            if not d(text="DONE").exists:
                return False
            d(resourceId="com.android.settings:id/redaction_done_button").click.wait()
            print "[Completed]------ Set screen lock password completed"
            return True
        else:
            if not d(text="Done").exists:
                return False
            d(resourceId="com.android.settings:id/next_button").click.wait()
            print "[Completed]------ Set screen lock password completed"
            return True

    def unlock_screen_lock_password(self, try_input_times, password):
        print "[Info]------ Unlock screen password"
        time.sleep(5)
        product = self.check_product()
        if "gordon_peak" in product:
            self.check_home_ui_user_safely_O()
        else:
            self.unlock_screen()
        time.sleep(2)
        for i in range(try_input_times):
            if d(resourceId="com.android.systemui:id/passwordEntry").exists:
                d(resourceId="com.android.systemui:id/passwordEntry").set_text(password)
                time.sleep(3)
                d.press.enter()
            if d(resourceId="com.android.settings:id/passwordEntry").exists:
                d(resourceId="com.android.settings:id/passwordEntry").set_text(password)
                time.sleep(3)
                d.press.enter()
            time.sleep(3)
        return True

    def remove_screen_lock_password(self, password):
        try:
            self.open_security_in_settings()
            print "[Info]------ Remove screen lock password"
            d(text="Screen lock").click()
            time.sleep(3)
            if d(resourceId="com.android.settings:id/password_entry").exists:
                d(resourceId="com.android.settings:id/password_entry").set_text(password)
                d.press.enter()
            time.sleep(2)
            if d(text="Swipe").exists:
                d(text="Swipe").click()
            time.sleep(1)
            if d(resourceId="android:id/button1").exists:
                d(resourceId="android:id/button1").click()
            time.sleep(3)
        finally:
            if d(text="Screen lock").exists and d(text="Swipe").exists:
                print "[Completed]------ Remove screen lock password completed"
                return True
            else:
                return False

    def reboot_require_unlock_screen_passwd_pin_to_mos(self, password_pin):
        mos_ui_passwd = "com.android.systemui:id/passwordEntry"
        mos_ui_pin = "com.android.systemui:id/pinEntry"
        boot_ui_passwd_pin = "com.android.settings:id/passwordEntry"
        self.reboot_devices()
        time.sleep(10)
        if not (d(resourceId=boot_ui_passwd_pin).exists or d(resourceId=mos_ui_passwd).exists or d(resourceId=mos_ui_pin).exists):
            self.reboot_devices()
            if d(resourceId=boot_ui_passwd_pin).exists:
                self.unlock_screen_lock_password(3, password_pin)
                time.sleep(20)
                self.unlock_screen_lock_password(3, password_pin)
                time.sleep(3)
                self.remove_resume_screen_lock_all(password_pin)
            if d(resourceId=mos_ui_passwd).exists:
                self.unlock_screen_lock_password(3, password_pin)
                time.sleep(3)
                self.remove_resume_screen_lock_all(password_pin)
            if d(resourceId=mos_ui_pin).exists:
                self.unlock_screen_lock_pin(3, password_pin)
                time.sleep(3)
                self.remove_resume_screen_lock_all(password_pin)
            assert False, "[Debug]------passwordEntry not exists"
        for i in range(10):
            if d(resourceId=mos_ui_passwd).exists:
                d(resourceId=mos_ui_passwd).set_text(password_pin)
                time.sleep(3)
                d.press.enter()
            if d(resourceId=mos_ui_pin).exists:
                d(resourceId=mos_ui_pin).set_text(password_pin)
                d.press.enter()
                time.sleep(3)
            if d(resourceId=boot_ui_passwd_pin).exists:
                d(resourceId=boot_ui_passwd_pin).set_text(password_pin)
                d.press.enter()
                time.sleep(3)
            if d(resourceId="android:id/message").exists:
                self.check_screen_unlock_need_unlock_at_resume()
        if d(resourceId=boot_ui_passwd_pin).exists or d(resourceId=mos_ui_passwd).exists or d(resourceId=mos_ui_pin).exists:
            assert False, "[Debug]------passwordEntry exists"
        time.sleep(40)

    def check_screen_unlock_need_reboot_at_boot(self, lock_type, try_input_times, password, pin=None):
        if lock_type == "Password":
            self.unlock_screen_lock_password(try_input_times, password)
        elif lock_type == "PIN":
            self.unlock_screen_lock_pin(try_input_times, pin)
        else:
            return
        mos_ui_passwd = "com.android.systemui:id/passwordEntry"
        boot_ui_passwd = "com.android.settings:id/passwordEntry"
        time.sleep(3)
        if d(resourceId=boot_ui_passwd).exists or d(resourceId=mos_ui_passwd).exists:
            if d(textContains="To unlock your phone, turn it off and then on").exists:
                print "[Info]------To unlock your phone, turn it off and then on ---exists"
                return True
            else:
                print "[Error]---'To unlock your phone, turn it off and then on'--- not exists "
                return False
        else:
            time.sleep(40)
            self.adb_root()
            time.sleep(10)
            self.start_RPCServer()
            print "[Completed]------ Unlock screen lock password or PIN completed"
            return True

    def check_unlock_incorrect_message_time(self):
        input_num_time = []
        time.sleep(2)
        if d(className="android.widget.FrameLayout").exists:
            if d(resourceId="android:id/message").exists:
                message_text = d(resourceId="android:id/message").text
                #You have incorrectly typed your PIN 5 times. Try again in 30 seconds.
                m = re.compile("your.+\s(\d+)\stimes")
                n = re.compile("again.+\s(\d+)\sseconds")
                aa = m.search(message_text)
                bb = n.search(message_text)
                last_num = int(aa.group(1))
                wait_time = int(bb.group(1))
                input_num_time = [last_num, wait_time]
                print input_num_time
                return input_num_time
            else:
                return False
        else:
            return False

    def check_screen_unlock_need_unlock_at_resume(self):
        time.sleep(2)
        if d(className="android.widget.FrameLayout").exists:
            if d(resourceId="android:id/button3").exists:
                d(resourceId="android:id/button3").click.wait()
            else:
                if d(text="OK").exists:
                    d(text="OK").click.wait()
            return True
        else:
            return False

    def unlock_screen_lock_passwd_pin_mos(self, try_input_times, password_pin):
        mos_ui_passwd = "com.android.systemui:id/passwordEntry"
        mos_ui_pin = "com.android.systemui:id/pinEntry"
        print "[Info]---Unlock screen password or pin"
        time.sleep(2)
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            if d(text="Owner").exists:
                d(text="Owner").click.wait()
            time.sleep(2)
            if d(textContains="START DRIVING").exists:
                d(textContains="START DRIVING").click.wait()
            else:
                g_common_obj.adb_cmd_capture_msg("input tap 959 1007")
        else:
            self.unlock_screen()
            if d(resourceId = "com.android.systemui:id/notification_stack_scroller").exists:
                self.single_swipe_to_app_list()
        for i in range(try_input_times):
            if d(resourceId=mos_ui_passwd).exists:
                print "[Info]---MOS password exists, Unlock Passwd"
                d(resourceId=mos_ui_passwd).set_text(password_pin)
                time.sleep(3)
                d.press.enter()
            if d(resourceId=mos_ui_pin).exists:
                print "[Info]---MOS PIN exists, Unlock PIN"
                d(resourceId=mos_ui_pin).set_text(password_pin)
                time.sleep(3)
                d.press.enter()
            time.sleep(3)

    def check_RPC_server_not_start(self, password_pin=None, pattern=None):
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            print "[Info]------product_name: {}".format(product_name)
        else:
            g_common_obj.adb_cmd_capture_msg("input swipe 800 1000 800 0")
        cmd_text = "input text {}".format(password_pin)
        cmd_enter = "input keyevent 66"
        g_common_obj.adb_cmd_capture_msg(cmd_text)
        time.sleep(1)
        g_common_obj.adb_cmd_capture_msg(cmd_enter)
        time.sleep(3)
        if pattern:
            self.draw_pattern_lock()
            time.sleep(3)
        time.sleep(10)

    def remove_resume_screen_lock_all(self, password_pin=None):
        set_rm_passwd_pin_reId = "com.android.settings:id/password_entry"
        set_rm_pattern_reId = "com.android.settings:id/lockPattern"
        try:
            self.open_security_in_settings()
            print "[Info]------ Remove screen lock password_pin_Pattern"
            d(text="Screen lock").click()
            time.sleep(35)
            if d(resourceId=set_rm_passwd_pin_reId).exists:
                d(resourceId=set_rm_passwd_pin_reId).set_text(password_pin)
                d.press.enter()
            if d(resourceId=set_rm_pattern_reId).exists:
                time.sleep(3)
                self.draw_pattern_lock()
            time.sleep(2)
            if d(text="Swipe").exists:
                d(text="Swipe").click()
            time.sleep(1)
            if d(resourceId="android:id/button1").exists:
                d(resourceId="android:id/button1").click()
            time.sleep(5)
        finally:
            if d(text="Screen lock").exists and d(text="Swipe").exists:
                print "[Completed]------ Remove screen lock password_pin_Pattern completed"
                return True
            else:
                return False

    def check_resume_screen_unlock_need_unlock_type(self, lock_type, try_input_times, password=None, pin=None):
        mos_ui_passwd = "com.android.systemui:id/passwordEntry"
        mos_ui_pin = "com.android.systemui:id/pinEntry"
        mos_ui_pattern = "com.android.systemui:id/lockPatternView"
        time.sleep(2)
        self.boot_up_completed_no_root()
        time.sleep(2)
        for loop in range(1, try_input_times + 1):
            print "[Info]------Unlock screen loop input times %d" %loop
            if lock_type == "Password":
                self.unlock_screen_lock_passwd_pin_mos(1, password)
            elif lock_type == "PIN":
                self.unlock_screen_lock_passwd_pin_mos(1, pin)
            elif lock_type == "Pattern":
                self.unlock_screen_lock_incorrect_pattern(1)
            else:
                return
            if loop < 5:
                if d(resourceId="android:id/message").exists:
                    input_num_time = self.check_unlock_incorrect_message_time()
                    input_times = input_num_time[0]
                    wait_input = input_num_time[1]
                    self.check_screen_unlock_need_unlock_at_resume()
                    print "[Info]---waitting for time %d second" % wait_input
                    time.sleep(wait_input)
                    if input_times != 5 or wait_input != 30:
                        return False
            if loop == 5:
                if self.check_unlock_incorrect_message_time() is False:
                    return False
                input_num_time = self.check_unlock_incorrect_message_time()
                input_times = input_num_time[0]
                wait_input = input_num_time[1]
                self.check_screen_unlock_need_unlock_at_resume()
                print "[Info]---waitting for time %d second" %wait_input
                time.sleep(wait_input)
                if input_times != 5 or wait_input != 30:
                    return False
            if loop == 10:
                if self.check_unlock_incorrect_message_time() is False:
                    return False
                input_num_time = self.check_unlock_incorrect_message_time()
                input_times = input_num_time[0]
                wait_input = input_num_time[1]
                self.check_screen_unlock_need_unlock_at_resume()
                print "[Info]---waitting for time %d second" % wait_input
                time.sleep(wait_input)
                if input_times != 10 or wait_input != 30:
                    return False
            if 11 <= loop < 30:
                if self.check_unlock_incorrect_message_time() is False:
                    return False
                input_num_time = self.check_unlock_incorrect_message_time()
                input_times = input_num_time[0]
                wait_input = input_num_time[1]
                self.check_screen_unlock_need_unlock_at_resume()
                print "[Info]---waitting for time %d second" % wait_input
                time.sleep(wait_input)
                if input_times != loop or wait_input != 30:
                    return False
            if loop == 30:
                if self.check_unlock_incorrect_message_time() is False:
                    return False
                input_num_time = self.check_unlock_incorrect_message_time()
                input_times = input_num_time[0]
                wait_input = input_num_time[1]
                self.check_screen_unlock_need_unlock_at_resume()
                print "[Info]---waitting for time %d second" % wait_input
                time.sleep(wait_input)
                if input_times != 30 or wait_input != 30:
                    return False
            if 31 <= loop < 140:
                input_num_time = self.check_unlock_incorrect_message_time()
                input_times = input_num_time[0]
                wait_input = input_num_time[1]
                self.check_screen_unlock_need_unlock_at_resume()
                print "[Info]---waitting for time %d second" % wait_input
                time.sleep(wait_input)
                if input_times != loop or wait_input != 30:
                    return False
        if self.check_screen_unlock_need_unlock_at_resume() is False:
            return False
        if d(resourceId=mos_ui_passwd).exists or d(resourceId=mos_ui_pin).exists or d(resourceId=mos_ui_pattern).exists:
            print "[Info]------com.android.systemui:id/passwordEntry or lockPatternView ---exists"
            return True
        else:
            print "[Error]---'com.android.systemui:id/passwordEntry or lockPatternView'--- not exists "
            return False

    def check_resume_reboot_last_input_5_times_reset(self, lock_type, try_input_num, incorrect_passwd=None, incorrect_pin=None):
        self.press_power_button()
        time.sleep(1)
        self.press_power_button()
        time.sleep(30)
        if self.check_resume_screen_unlock_need_unlock_type(lock_type, try_input_num, incorrect_passwd, incorrect_pin) is True:
            return True
        else:
            return False

    def set_screen_lock_pin(self, PIN):
        self.choose_screen_lock("PIN")
        time.sleep(2)
        print "[Info]------ Set screen lock PIN: %s" % PIN
        d(resourceId="com.android.settings:id/password_entry").set_text(PIN)
        d(resourceId="com.android.settings:id/next_button").click()
        time.sleep(1)
        d(resourceId="com.android.settings:id/password_entry").set_text(PIN)
        d(resourceId="com.android.settings:id/next_button").click()
        time.sleep(4)
        # Text: Done
        product = self.check_product()
        if "gordon_peak" in product or "androidia" in product or "celadon" in product:
            if not d(text="DONE").exists:
                return False
            d(resourceId="com.android.settings:id/redaction_done_button").click.wait()
            print "[Completed]------ Set screen lock PIN completed"
            return True
        else:
            if not d(text="Done").exists:
                return False
            d(resourceId="com.android.settings:id/next_button").click.wait()
            print "[Completed]------ Set screen lock PIN completed"
            return True

    def unlock_screen_lock_pin(self, try_input_times, PIN):
        print "[Info]------ Unlock screen PIN"
        time.sleep(5)
        product = self.check_product()
        if "gordon_peak" in product:
            self.check_home_ui_user_safely_O()
        else:
            self.unlock_screen()
        time.sleep(2)
        for i in range(try_input_times):
            if d(resourceId="com.android.systemui:id/passwordEntry").exists:
                d(resourceId="com.android.systemui:id/passwordEntry").set_text(PIN)
                time.sleep(3)
                d.press.enter()
            if d(resourceId="com.android.settings:id/passwordEntry").exists:
                d(resourceId="com.android.settings:id/passwordEntry").set_text(PIN)
                time.sleep(3)
                d.press.enter()
            if d(resourceId="com.android.systemui:id/pinEntry").exists:
                d(resourceId="com.android.systemui:id/pinEntry").set_text(PIN)
                time.sleep(3)
                d.press.enter()
            time.sleep(3)
        return True

    def remove_screen_lock_pin(self, PIN):
        try:
            self.open_security_in_settings()
            print "[Info]------ Remove screen lock PIN"
            d(text="Screen lock").click()
            time.sleep(3)
            if d(resourceId="com.android.settings:id/password_entry").exists:
                d(resourceId="com.android.settings:id/password_entry").set_text(PIN)
                d.press.enter()
            time.sleep(2)
            if d(text="Swipe").exists:
                d(text="Swipe").click()
            time.sleep(1)
            if d(resourceId="android:id/button1").exists:
                d(resourceId="android:id/button1").click()
            time.sleep(3)
        finally:
            if d(text="Screen lock").exists and d(text="Swipe").exists:
                print "[Completed]------ Remove screen lock password completed"
                return True
            else:
                return False

    def rotate(self, orientation):
        d.orientaton = orientation
        time.sleep(2)
        assert d.orientaton == orientation

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

    def check_home_ui_user_safely_O(self, wait_time = 25):
        time.sleep(wait_time)
        self.start_RPCServer()
        if d(textContains="Drive safely").exists or d(textContains="Owner").exists:
            if d(textContains="Owner").exists:
                owner_coordinates =self.bounds_icon_coordinates("Owner")
                x = owner_coordinates[0]
                y = owner_coordinates[1]
                g_common_obj.adb_cmd_capture_msg("input tap %d %d" %(x, y))
            time.sleep(2)
            if d(textContains="START DRIVING").exists:
                d(textContains="START DRIVING").click.wait()
            #assert not d(text="Owner").exists
        else:
            if d(textContains="Owner").exists:
                d(textContains="Owner").click.wait()
            if d(textContains="START DRIVING").exists:
                d(textContains="START DRIVING").click.wait()
            #d.press.home()
            g_common_obj.adb_cmd_capture_msg("input tap 310 1007")
            g_common_obj.adb_cmd_capture_msg("input tap 959 1007")
            time.sleep(2)
            #assert not d(text="Owner").exists

    def check_boot_completed(self):
        g_common_obj.adb_cmd_capture_msg("getprop sys.boot_completed")
        for i in range(30):
            time.sleep(5)
            message = g_common_obj.adb_cmd_capture_msg("getprop sys.boot_completed")
            if '1' == message:
                return True
        return False

    def reboot_devices(self):
        for i in range(5):
            g_common_obj.adb_cmd_common("reboot")
            time.sleep(30)
            if self.check_boot_completed() is True:
                print "[info]--- reboot completed"
                time.sleep(3)
                # self.unlock_screen() #rpc server always failed
                self.adb_root()
                self.boot_up_completed_skip_boot_ui()
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
                self.single_swipe_to_app_list()
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
                time.sleep(2)
            else:
                d(text="Sleep").click.wait()
                time.sleep(2)
        else:
            print 123
        self.check_screen_off()
        #self.set_screen_status("on")
        #self.unlock_screen_comm()

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
            self.unlock_screen()

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

    def boot_up_completed_no_root(self, timeout = 20):
        self.check_boot_completed()
        time.sleep(timeout)
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            if d(textContains="Owner").exists or d(textContains="Drive safely").exists:
                if d(textContains="Owner").exists:
                    owner_coordinates = self.bounds_icon_coordinates("Owner")
                    x = owner_coordinates[0]
                    y = owner_coordinates[1]
                    g_common_obj.adb_cmd_capture_msg("input tap %d %d" % (x, y))
                time.sleep(2)
                if d(textContains="START DRIVING").exists:
                    d(textContains="START DRIVING").click.wait()
                    time.sleep(10)
            else:
                print "[Debug]Error--- not found Drive safely UI"
                if d(textContains="START DRIVING").exists:
                    d(textContains="START DRIVING").click.wait()
                    time.sleep(10)
                else:
                    g_common_obj.adb_cmd_capture_msg("input tap 310 1007")
                    g_common_obj.adb_cmd_capture_msg("input tap 959 1007")
                    time.sleep(10)
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
        time.sleep(30)
        if self.check_adb_connection() is False:
            print "[INFO]---press ignition key success"
        else:
            time.sleep(80)
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

    def reset_devices(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        product_name = self.check_product()
        if "gordon_peak" in product_name or "androidia" in product_name or "celadon" in product_name:
            self.scrollable_select_text("System")
            d(textContains="System").click.wait()
            d(textContains="Reset").click.wait()
            if d(textContains="Erase all data").exists:
                d(textContains="Erase all data").click.wait()
            if d(textContains="Factory").exists:
                d(textContains="Factory").click.wait()
            d(textContains="RESET PHONE").click.wait()
            d(textContains="ERASE EVERYTHING").click.wait()
            print "[Info]------Wait for 150 seconds"
            time.sleep(150)
            self.boot_up_completed_skip_boot_ui()
        else:
            if d(textContains="Reset").exists:
                d(textContains="Reset").click.wait()
            if d(textContains="Factory").exists:
                d(textContains="Factory").click.wait()
            d(textContains="RESET PHONE").click.wait()
            d(textContains="ERASE EVERYTHING").click.wait()
            print "[Info]------Wait for 150 seconds"
            time.sleep(150)
            self.single_swipe_to_app_list()

    def check_security_encryption_type(self):
        self.adb_root()
        cmd_data = "cat /fstab.* | grep encrypt"
        cmd_data_O = "cat /vendor/etc/fstab.* | grep encrypt"
        msg_data = g_common_obj.adb_cmd_capture_msg(cmd_data)
        if msg_data == '':
            msg_data = g_common_obj.adb_cmd_capture_msg(cmd_data_O)
        if "fileencrypt" in msg_data:
            print "[Check]------crypto data type: File encrypt"
            return 'file'
        elif "forceencrypt" in msg_data:
            print "[Check]------crypto data type: Disk encrypt"
            return 'disk'
        else:
            print "[Check]------crypto data type: other encrypt type"
            raise EnvironmentError, "Not support Disk or File Encrypt"

    def run_check_disk_encryption_type(self):
        type = self.check_security_encryption_type()
        if type == 'disk':
            print "[Info]------Support Disk Encrypt"
        else:
            raise EnvironmentError, "Not support Disk Encrypt, type is: {} Encrypt".format(type)

    def run_check_file_encryption_type(self):
        type = self.check_security_encryption_type()
        if type == 'file':
            print "[Info]------Support File Encrypt"
        else:
            raise EnvironmentError, "Not support File Encrypt, type is: {} Encrypt".format(type)