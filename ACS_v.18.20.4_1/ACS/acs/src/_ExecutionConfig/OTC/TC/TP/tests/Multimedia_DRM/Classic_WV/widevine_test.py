# coding: utf-8
import os
import time
import re
import subprocess
import sys
import thread
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle
from testlib.multimedia.multimedia_drm_helper import MultiMediaDRMHelper
from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj as adb

from testlib.util.log import Logger
logger = Logger.getlogger()


class WidevineTest(UIATestBase):
    """
    @summary: Test Widevine
    """
    
    config = TestConfig()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(WidevineTest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        self.video_button_widget_dict={"play":"android:id/pause",
                                       "pause":"android:id/pause",
                                       "volume_up":"volume_up",
                                       "volume_down":"volume_down",
                                       "back":"back"}
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(WidevineTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()

    def appPrepare(self, model=1):
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_drm.conf')
        self.cfg = self.config.read(self.cfg_file, self.case_name)
        
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(self.cfg_file)
        self.multimedia_drm_helper = MultiMediaDRMHelper()
        
        self.multimedia_setting.install_apk("widevine_apk")#install widevine apk
        self.widevine_package_name, self.widevine_activity_name = self.multimedia_setting.get_package_and_activity_name("widevine_apk")
        
        if model == 1:# play local video mode
            self.dst_path = self.cfg.get("dst_path")
            self.clearResourcefolder()
            self.dst_path = self.multimedia_setting.push_file_new(self.cfg.get("src_path"), self.dst_path)#push video file to device
        elif model == 2:# play network video mode
            self.src_path = self.cfg.get("src_path")

#         g_common_obj.set_vertical_screen()
        g_common_obj.stop_app_am(self.widevine_package_name)
        g_common_obj.adb_cmd_capture_msg("input keyevent 82") #Unlock screen

    def disableVerityDUT(self):
        serial = self.d.server.adb.device_serial()
        result = os.popen("adb -s %s disable-verity" % serial).read()
        logger.debug("result=%s" % result)
        assert "version" not in result and "error" not in result, "Please update sdk version!"
        assert "fail" not in result and "error" not in result, "The DUT can't disable-verity, please reboot and check it"

    def remountDUT(self):
        serial = self.d.server.adb.device_serial()
        os.system("adb -s %s remount" % serial)
        time.sleep(3)
        result = os.popen("adb -s %s remount" % serial).read()
        logger.debug("result=%s" % result)
        assert "fail" not in result and "error" not in result, "The DUT can't remount, please reboot and check it"

    def getDeviceTypeMapCfg(self):
        if "device_type_map" not in dir(self):
            self.device_type_map_cfg = self.config.read(self.cfg_file, "device_type_map")
        return self.device_type_map_cfg

    def executeCommandWithPopen(self, cmd):
        logger.debug(cmd)
#         return subprocess.Popen(cmd, shell=True)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

    def clearResourcefolder(self):
        g_common_obj.adb_cmd_capture_msg("rm -rf %s" % os.path.split(self.dst_path)[0])

    def launchWidevineApp(self):
        SystemUI().unlock_screen()
#         g_common_obj.stop_app_am(self.widevine_package_name)
        g_common_obj.launch_app_am(self.widevine_package_name, self.widevine_activity_name)
        time.sleep(5)
        for _ in range(5):
            if self.d(textContains="Downloads").exists:
                return
            time.sleep(5)
        assert self.d(textContains="Downloads").exists, "launch Winevine App failed!"

    def findVideoExistInWidevineApp(self, video_name):
        logger.debug("findVideoExistInWidevineApp video_name=%s" % video_name)
        time.sleep(5)
        for _ in range(10):
            if self.d(textContains=video_name).exists:
                break
            self.d(text=">>").click()
            time.sleep(20)
        assert self.d(textContains=video_name).exists, "Video not exist! video_name=%s" % video_name
        bounds = self.d(textContains=video_name).info["bounds"]
        x = bounds["left"] + (bounds["right"] - bounds["left"])/2
        y = bounds["top"] - (bounds["bottom"] - bounds["top"])/2
        self.d.click(x,y)

    def clear_edit_text_layout(self,layout):
        while layout.text != "":
            layout.clear_text()

    def setContentPage(self, path):
        self.d(text="Settings").click()
        time.sleep(2)
        edit_text_layout = self.d(className="android.widget.EditText", resourceId="com.widevine.demo:id/content_page")
        if edit_text_layout.text != path:
            self.clear_edit_text_layout(edit_text_layout)
            edit_text_layout.set_text(path)
            self.d.press.back()
            time.sleep(1)
            self.d(text="Update Button").click()
            self.d(text="Ok").click()

    def wait_boot_completed(self, timeout=1000):
        ''' wait Android boot_completed
    
        args: timeout -- optional timeout in second, default 1000s
        '''
        count = 0
        sleep_time = 5
        while count < timeout:
            prop_val = adb.adb_cmd_capture_msg('getprop sys.boot_completed')
            if '1' in prop_val:
                print 'boot_completed'
                return
            count += sleep_time
            time.sleep(sleep_time)
        raise Exception('%ds timeout waiting for boot_completed' % timeout)

    def widevineVideoPlayBack(self):
        self.d(text="Downloads").click()
        time.sleep(5)
        dst_name = os.path.split(self.dst_path)[1]
        assert self.d(textContains=dst_name).exists, "resource file not exist! file_name=%s" % dst_name
        self.d(className="android.widget.ImageView", index="0").click()
        time.sleep(5)
        self.d(text="Acquire Rights").click()
        time.sleep(3)
        acquire_rights_value = -1
        for _ in range(3):
            widevine_value = self.getWidevineValue()
            logger.debug("widevine_value=%s" % widevine_value)
            if "acquireRights" in widevine_value and "Rights installed" in widevine_value:
                acquire_rights_value = self.getWidevineValue_acquireRights()
                logger.debug("acquire_rights_value=%s" % str(acquire_rights_value))
                break
            time.sleep(3)
        assert "error" not in widevine_value and "fail" not in widevine_value, "error widevine_value!! widevine_value=%s" % widevine_value
        assert acquire_rights_value == 0, "error acquire_rights_value!!! acquire_rights_value=%d" % acquire_rights_value
        self.d(text="Play").click()

    def widevineStreamingVideoPlayBack(self):
        self.d(text="Streaming").click()
        time.sleep(5)
        video_index = self.findVideoExistInWidevineApp(self.src_path)
        self.d(className="android.widget.ImageView", index=video_index).click()
        time.sleep(5)
        self.d(text="Acquire Rights").click()
        time.sleep(3)
        acquire_rights_value = -1
        for _ in range(10):
            widevine_value = self.getWidevineValue()
            logger.debug("widevine_value=%s" % widevine_value)
            if "acquireRights" in widevine_value and "Rights installed" in widevine_value:
                acquire_rights_value = self.getWidevineValue_acquireRights()
                logger.debug("acquire_rights_value=%s" % str(acquire_rights_value))
                break
            time.sleep(3)
        assert "error" not in widevine_value and "fail" not in widevine_value, "error widevine_value!! widevine_value=%s" % widevine_value
        assert acquire_rights_value == 0, "error acquire_rights_value!!! acquire_rights_value=%d" % acquire_rights_value
        self.d(text="Play").click()

    def getWidevineValue(self):
        self.widevine_value_text = self.d(className="android.widget.TextView").text
        return self.widevine_value_text
    
    def get_acquire_rights_pattern(self):
        if "acquire_rights_pattern" not in dir(self):
            GET_ACQUIRERIGHTS_PATTERN = ".*acquireRights = (.*)\n.*"
            self.acquire_rights_pattern = re.compile(GET_ACQUIRERIGHTS_PATTERN)
        return self.acquire_rights_pattern
    
    def getWidevineValue_acquireRights(self):
        try:
            acquire_rights_pattern = self.get_acquire_rights_pattern()
            widevine_value = self.getWidevineValue()
            acquire_rights_value = acquire_rights_pattern.findall(widevine_value)
            acquire_rights_value = int(acquire_rights_value[0])
        except Exception as e:
            logger.debug("widevine_value=%s" % widevine_value)
            logger.debug("acquire_rights_value=%s" % str(acquire_rights_value))
            assert 0, e
        return acquire_rights_value

    def getPlatform(self):
        cmd = "adb shell getprop ro.board.platform"
        platform = os.popen(cmd).read()
        platform = platform.replace("\n", "", 1)
        return platform.strip()

    def getDeviceTypeMapNum(self):
        device_platform = self.getPlatform()
        device_platform = device_platform.lower()
        if "sofia_lte" in device_platform:
            num = 2
        else:
            num = 1
#         num = self.getDeviceTypeMapCfg().get(device_platform)
        return num

    def clickScreen(self):
        self.multimedia_setting.clickScreen()

    def click_widget(self, t_widget):
        try:
            t_widget.click()
            self.s_t_text = 0
        except Exception as e:
            logger.debug("click_widget error:%s" % e)
            self.s_t_text = -2
            assert 0, "click_widget error:%s" % e

    def get_widget(self, t_str):
        logger.debug("get_widget---t_str=%s" % t_str)
        assert t_str in self.video_button_widget_dict, "%s not in video_button_widget_dict!" % t_str
        t_resource_id = self.video_button_widget_dict[t_str]
        if t_resource_id == t_str:
            return t_resource_id
        else:
            return self.d(resourceId=t_resource_id)

    def widget_operation_with_thread(self, t_str, t_operation):
        self.s_t_text = -1
        t_widget = self.get_widget(t_str)
        if t_operation == "click":
            thread.start_new_thread(self.click_widget, (t_widget, ))
        else:
            assert 0, "Error operation!"
        while self.s_t_text == -1:
            if not t_widget.exists:
                self.clickScreen()
            time.sleep(1)
        return self.s_t_text

    def click_button(self, t_str):
        logger.debug("click_button---t_str=%s" % t_str)
        t_widget = self.video_button_widget_dict[t_str]
        if t_str != t_widget:
            result = self.widget_operation_with_thread(t_str, "click")
            assert result == 0, "click failed! reselt=%d" % result
        else:
            getattr(self.d.press, t_widget)()

    def checkVideoPlayBackWithWidevine(self):
        self.launchWidevineApp()# launch widevine app
        self.widevineVideoPlayBack()# play video file
        time.sleep(10)
        assert not self.d(textContains="can't").exists, "video playback failed!"#check play status of video
        time.sleep(140)
        assert not self.d(textContains="can't").exists, "video playback failed!"#check play status of video

    def checkStreamingVideoPlayBackWithWidevine(self):
        self.launchWidevineApp()
        self.setContentPage(self.cfg.get("web_page"))
        self.widevineStreamingVideoPlayBack()
        time.sleep(10)
        assert not self.d(textContains="can't").exists, "video playback failed!"
        time.sleep(140)
        assert not self.d(textContains="can't").exists, "video playback failed!"

    def checkVideoPlayBackWithClickButton(self, click_list):
        self.launchWidevineApp()# launch widevine app
        self.widevineVideoPlayBack()# play video file
        time.sleep(10)
        assert not self.d(textContains="can't").exists, "video playback failed!"#check play status of video
        for t_str in click_list:
                if t_str == "sleep":
                    time.sleep(5)
                else:
                    self.click_button(t_str)
                    time.sleep(2)
        time.sleep(60)
        assert not self.d(textContains="can't").exists, "video playback failed!"#check play status of video

    def winevine_main_test(self, sub_func_name="", model=1, *arg, **keywords):
        """
        This test used to test WinevineApp App
        The test case spec is following:
        1. Start record video
        2. do sub_func()
        3. Stop record video
        """
        self.case_name = sys._getframe().f_back.f_code.co_name
        if sub_func_name == "":
            sub_func_name = "%s_sub_func" % self.case_name
        logger.debug("case_name=%s" % self.case_name)
        logger.debug("exoplayer_main_test---sub_func_name=%s" % sub_func_name)
        self.appPrepare(model)
        try:
            self.multimedia_drm_helper.startRecord(self.case_name)#start record
            
            
            logger.debug("Arbitrary parameter is %s" % str(arg))
            logger.debug("keywords parameter is %s" % str(keywords))
            getattr(self, sub_func_name)(*arg, **keywords)
            
            self.d.press.back()
            self.d.press.back()
            self.d.press.home()
            if model == 1:
                self.clearResourcefolder()
            time.sleep(1)
            self.multimedia_drm_helper.stopRecord()#stop record
        except Exception as e:
            self.multimedia_drm_helper.stopRecord()
            assert 0, e
        
        assert 0, "Playback complete! Please check video!"
        logger.debug("Case %s is pass!" % self.case_name)

    def test_video_playback_with_check_S0I3(self):
        self.case_name = sys._getframe().f_back.f_code.co_name
        self.appPrepare()
        after_S0I3_str = self.multimedia_setting.getS0I3()
        self.launchWidevineApp()
        self.widevineVideoPlayBack()
        time.sleep(10)
        assert not self.d(textContains="can't").exists, "video playback failed!"
        self.d.press.power()
        logger.debug("Step: sleep 800s, please wait...")
        time.sleep(800)
        logger.debug("Step: sleep 800s, complete!")
        self.d.press.power()
        SystemUI().unlock_screen()
        self.d.press.back()
        self.d.press.home()
        before_S0I3_str = self.multimedia_setting.getS0I3()
        assert after_S0I3_str != before_S0I3_str, "after_S0I3_str=%s, before_S0I3_str=%s" % (after_S0I3_str, before_S0I3_str)

    def checkVideoPlayBackWithWidevineWithReboot(self, t_time):
        self.case_name = sys._getframe().f_back.f_code.co_name
        self.appPrepare()
        self.launchWidevineApp()
        self.widevineVideoPlayBack()
        time.sleep(20)
        assert not self.d(textContains="can't").exists, "video playback failed!"
        for _ in range(t_time):
            adb.reboot_device()
            self.wait_boot_completed()
        time.sleep(20)
        self.launchWidevineApp()
        self.widevineVideoPlayBack()
        time.sleep(20)
        assert not self.d(textContains="can't").exists, "video playback failed!"

    def checkProvision(self):
        self.case_name = sys._getframe().f_back.f_code.co_name
        logger.debug("case_name=%s" % self.case_name)
        self.appPrepare(0)
        if "false" == self.cfg.get("test"):
            logger.debug("Case %s is skip auto test!" % self.case_name)
            return
        num = self.getDeviceTypeMapNum()
        if num == 1:
            g_common_obj.root_on_device()
            self.disableVerityDUT()
            adb.reboot_device()
            self.wait_boot_completed()
        g_common_obj.root_on_device()
        self.remountDUT()
        time.sleep(3)
        bin_src_path = self.getDeviceTypeMapCfg().get("%s_bin_src_path" % (num))
        bin_dst_path = self.getDeviceTypeMapCfg().get("%s_bin_src_path" % (num))
        bin_src_path = self.multimedia_setting.download_file_to_host(bin_src_path)
        bin_src_folder = os.path.split(bin_src_path)[0]
        bin_dst_filename = os.path.split(bin_dst_path)[1]
        unzip_cmd = self.getDeviceTypeMapCfg().get("unzip_command")
        self.executeCommandWithPopen(unzip_cmd % (bin_src_path, bin_src_folder))
        bin_src_path = os.path.join(bin_src_folder, bin_dst_filename)
        bin_dst_path = self.multimedia_setting.push_file_to_dut(bin_src_path, bin_dst_path)
        
        tools_src_path = self.getDeviceTypeMapCfg().get("%s_tool_src_path" % (num))
        tools_dst_path = self.getDeviceTypeMapCfg().get("%s_tool_dst_path" % (num))
        tools_dst_path = self.multimedia_setting.push_file_new(tools_src_path, tools_dst_path)
        g_common_obj.adb_cmd_capture_msg("chmod 777 %s" % tools_dst_path)
        time.sleep(5)
        i = 1
        while "%s_execute_command_%s" %(num, i) in self.getDeviceTypeMapCfg():
            cmd = self.getDeviceTypeMapCfg().get("%s_execute_command_%s" %(num, i))
            fdp = self.executeCommandWithPopen(cmd)
            time.sleep(3)
            stdout_log = fdp.stdout.read()
            assert "cannot" not in stdout_log and "error" not in stdout_log and "fail" not in stdout_log and "not found" not in stdout_log, "(%s) cmd error: %s" % (cmd, stdout_log)
            i += 1
        logger.debug("Case %s is pass!" % self.case_name)

    def testWVClassicPlayback_WVDemo_Local_720p(self):
        self.winevine_main_test("checkVideoPlayBackWithWidevine")

    def testWVClassic_WVDemo_Local_1080p(self):
        self.winevine_main_test("checkVideoPlayBackWithWidevine")

    def testWVClassic_WVDemo_Local_360(self):
        self.winevine_main_test("checkVideoPlayBackWithWidevine")

    def testWVClassic_WVDemo_SD(self):
        self.winevine_main_test("checkVideoPlayBackWithWidevine")

    def testWVClassic_WVDemo_Local_480p(self):
        self.winevine_main_test("checkVideoPlayBackWithWidevine")

    def testWVClassic_WVDemo_HD_Local(self):
        self.winevine_main_test("checkVideoPlayBackWithWidevine")

    def testWVClassic_WVDemo_WiFi_360p(self):
        self.winevine_main_test("checkStreamingVideoPlayBackWithWidevine", 2)

    def testWVClassic_WVDemo_WiFi_480P(self):
        self.winevine_main_test("checkStreamingVideoPlayBackWithWidevine", 2)

    def testWVClassic_WVDemo_WiFi_720p(self):
        self.winevine_main_test("checkStreamingVideoPlayBackWithWidevine", 2)

    def testWVClassic_WVDemo_WiFi_1080p(self):
        self.winevine_main_test("checkStreamingVideoPlayBackWithWidevine", 2)

    def testWVClassic_WVDemo_Menu_Interaction(self):
        self.winevine_main_test("checkVideoPlayBackWithClickButton", 1, ["volume_up"]*4 + ["sleep"] + ["volume_down"]*4)

    def testWVClassic_WVDemo_Volume_Adjustment_Paused(self):
        self.winevine_main_test("checkVideoPlayBackWithClickButton", 1, ["pause"] + ["volume_up"]*10 + ["sleep"] + ["volume_down"]*10 + ["play"] + ["volume_down"]*10 + ["pause"])

    def testWVClassic_Reboot_Multi_Times(self):
        self.checkVideoPlayBackWithWidevineWithReboot(20)

    def testWVClassic_Resume_S0i3(self):
        self.test_video_playback_with_check_S0I3()

    def testProvisionKeyBox_Reboot(self):
        self.checkProvision()

    def testWVClassic_Provision(self):
        self.checkProvision()