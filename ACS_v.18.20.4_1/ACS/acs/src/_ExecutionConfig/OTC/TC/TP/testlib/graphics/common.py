# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

"""
@summary: Graphics Common class
@since: 03/13/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import os
import sys
import time
import tempfile
import random
import math
import re
import string
import functools
from PIL import Image
from testlib.util.log import Logger
from testlib.util.process import shell_command, shell_command_ext
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.androidframework.common import EnvironmentUtils
from testlib.graphics.compare_pic_impl import compare_pic

try:
    import pyqrcode
    import qrtools
except ImportError:
    pyqrcode = None
    qrtools = None

LOG = Logger.getlogger(__name__)

config_handle = ConfigHandle()
artifactory_href = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
artifactory = Artifactory(artifactory_href)

# Sub settings activity list
ACCESSIBILITY = 'AccessibilitySettingsActivity'


class Logcat(object):

    """Device Logcat"""

    def get_device_time_mark(self):
        """get device time"""
        cmd = 'date +"%m-%d %H:%M:%S"'
        date = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        time.sleep(1)
        return date

    def get_device_log(self, time_mark=None, filters="", pid=None):
        """get device log"""
        cmd = 'logcat -d"'
        filtered_out_cmd = "PDelay Response Receipt Timeout"
        if time_mark:
            cmd = 'logcat -v threadtime -t "%s.000" %s' % (time_mark, filters)
        cmd = repr(cmd) + r"|grep -Ev '\-\- beginning|%s'" % filtered_out_cmd
        log = g_common_obj.get_test_device().adb_cmd_capture_msg_ext(cmd, time_out=90)

        if pid:
            pid_filter = re.findall(r".+ %s  \d+ .+\n" % (pid), log)
            log = ''.join(pid_filter)

        return log

    def check_dmesg_info(self, matchCase=True, keyword=None, assertword=None):
        g_common_obj.root_on_device()
        para = 'I' if matchCase else 'i'
        cmd = "dmesg | grep -%sE %s" % (para, keyword)
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        print "-----Get dmesg info-----:\n %s" % result
        return True if assertword in result else False

    def check_dumpsys_SurfaceFlinger_info(self, matchCase=False, keyword=None, assertword=None):
        para = 'I' if matchCase else 'i'
        cmd = "dumpsys SurfaceFlinger | grep -%s '%s'" % (para, keyword)
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        print "-----Get SurfaceFlinger info-----:\n %s" % result
        return True if assertword in result else False

    def check_dumpsys_SurfaceFlinger_info_with_multiple_keys(self, matchCase=False, keyword=None, assertword=[]):
        para = 'I' if matchCase else 'i'
        cmd = "dumpsys SurfaceFlinger | grep -%sE %s" % (para, keyword)
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        print "-----Get SurfaceFlinger info-----:\n %s" % result
        return any(i in result for i in assertword)

    def get_timestamp_dmesg(self, keyword=None):
        g_common_obj.root_on_device()
        cmd = "dmesg | grep -wIE %s | grep -oP '\d+\.\d+'" % keyword
        timestamps = g_common_obj.adb_cmd_capture_msg(cmd).splitlines()
        print "-----Get timestamps list-----:\n %s" % timestamps
        return timestamps

    def get_layer_number(self):
        cmd = "dumpsys SurfaceFlinger | grep -io num.*Layers\=[0-9]* | cut -d '=' -f2"
        return g_common_obj.adb_cmd_capture_msg(cmd).splitlines()

    def get_refresh_rate(self):
        cmd = "dumpsys SurfaceFlinger | grep  refresh\-rate | cut -d':' -f2 | cut -d' ' -f2"
        return float(g_common_obj.adb_cmd_capture_msg(cmd))

logcat = Logcat()


class Allow_Permission(object):

    def __init__(self):
        self.d = g_common_obj.get_device()

    def permission_allower(self):
        for _ in range(0, 9):
            if self.d(resourceId="com.android.packageinstaller:id/permission_allow_button").exists:
                self.d(resourceId="com.android.packageinstaller:id/permission_allow_button").click.wait()
                time.sleep(1)
            if self.d(text="ALLOW").exists:
                self.d(text="ALLOW").click()
                time.sleep(1)
            if self.d(text="NEXT").exists:
                self.d(text="NEXT").click()
                time.sleep(1)


class Busybox(object):

    """BusyBox"""

    CONFIG_FILE = 'tests.common.busybox.conf'

    def __init__(self):
        self.device = g_common_obj.get_device()

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "Busybox")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.dut_bin = self.config.get("dut_bin")

    def setup(self):
        """busybox setup"""
        self.clean()
        rfile = self.arti.get(self.config.get("file"))
        ret = g_common_obj.push_file(rfile, self.dut_bin)
        assert ret, 'Failed push %s' % (self.dut_bin)
        cmd = "chmod 777 %s; %s --help| %s head -1"\
            % (self.dut_bin, self.dut_bin, self.dut_bin)
        output = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print "[Debug] %s" % (output)

    def adb_busybox_cmd(self, cmd, time_out=90):
        """adb shell busybox cmd"""
        cmd = cmd.replace('busybox', self.dut_bin)
        return g_common_obj.adb_cmd_capture_msg(cmd, time_out)

    def adb_busybox_cmd_nomsg(self, cmd, time_out=90):
        """adb shell busybox cmd"""
        cmd = cmd.replace('busybox', self.dut_bin)
        return g_common_obj.adb_cmd(cmd, time_out)

    def clean(self):
        """clean busybox"""
        cmd = "rm -rf %s; sync;" % (self.dut_bin)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

busybox_obj = Busybox()


class AdbExtension(object):
    """
    This class is aim to do some special preconditions based on adb ver.1.0.32
    """
    def __init__(self):
        self.SET_TIMEOUT = 45
        self.ANDROID_VERSION = EnvironmentUtils().get_android_version()

    def adb_shell_stop(self):
        g_common_obj.root_on_device()
        if self.ANDROID_VERSION == "M":
            g_common_obj.adb_cmd("stop")
            time.sleep(3)
            self._check_display_status("off", self.SET_TIMEOUT)
        else:
            g_common_obj.adb_cmd("stop")
            time.sleep(6)
            g_common_obj.adb_cmd("stop hwcomposer-2-1")
            time.sleep(6)
            self._check_display_status("off", self.SET_TIMEOUT)

    def adb_shell_start(self):
        g_common_obj.root_on_device()
        if self.ANDROID_VERSION == "M":
            g_common_obj.adb_cmd("start")
            time.sleep(3)
            self._check_display_status("on", self.SET_TIMEOUT)
        else:
            g_common_obj.adb_cmd("start")
            time.sleep(6)
            g_common_obj.adb_cmd("start hwcomposer-2-1")
            time.sleep(6)
            self._check_display_status("on", self.SET_TIMEOUT)

    def refresh_ui(self):
        self.adb_shell_stop()
        self.adb_shell_start()

    def adb_disable_verity(self):
        """
        M: remount /system instead of disable-verity.
        O: Need to flash vbmeta_img in fastboot mode.
        """
        if self.ANDROID_VERSION == "M":
            g_common_obj.root_on_device()
            cmd="busybox mount -o rw, remount /system"
            busybox_obj.setup()
            busybox_obj.adb_busybox_cmd(cmd)
            self._adb_reboot()
            g_common_obj.root_on_device()
            time.sleep(3)
            g_common_obj.remount_device()
            time.sleep(3)
        else:
            if 'remount succeeded' not in self._remount_result() or \
                            len(self._remount_result()) != 1:
                self._adb_reboot_fastboot()
                remote_vbmeta_path = TestConfig().read("tests.tablet.artifactory.conf",
                                                       "framebuffer_dependency_o").get("vbmeta")
                arti = Artifactory(TestConfig().read(section='artifactory').get('location'))
                vbmetaimg = arti.get(remote_vbmeta_path)
                self.fastboot_flash_vbmeta(vbmetaimg)
                time.sleep(3)
                assert 'remount succeeded' in self._remount_result(), "Disable-verity failed."
                time.sleep(3)

    def _adb_reboot_fastboot(self):
        adbcmdstr = "adb reboot fastboot"
        LOG.debug('Execute adb reboot fastboot')
        ret, _ = shell_command(adbcmdstr)
        fastbootcmdstr = "fastboot wait-for-device"
        LOG.debug('Waiting for device online')
        ret, _ = shell_command(fastbootcmdstr, timeout_second=90)
        return ret

    def _remount_result(self):
        # return exit code, 0 for succeed, 1 for error
        g_common_obj.root_on_device()
        time.sleep(3)
        LOG.debug('Execute adb remount')
        ret = [i.strip() for i in shell_command('adb remount')[1]]
        LOG.debug('Got remount result: %s' % ret)
        return ret

    def fastboot_flash_vbmeta(self, vbmeta_img=""):
        if "vbmeta_disable_verity.img" not in vbmeta_img:
            return False
        else:
            fastbootcmdstr = "fastboot flashing unlock"
            LOG.debug('Execute fastboot flashing unlock')
            ret, _ = shell_command(fastbootcmdstr)
            fastbootcmdstr = "fastboot flash vbmeta %s" % vbmeta_img
            LOG.debug('Execute fastboot flash vbmeta')
            ret, _ = shell_command(fastbootcmdstr)
            fastbootcmdstr = "fastboot flashing lock"
            LOG.debug('Execute fastboot flashing lock')
            ret, _ = shell_command(fastbootcmdstr)
            fastbootcmdstr = "fastboot reboot"
            LOG.debug('Execute fastboot reboot')
            ret, _ = shell_command(fastbootcmdstr)
            adbcmdstr = "adb wait-for-device"
            LOG.debug('Waiting for device online')
            ret, _ = shell_command(adbcmdstr, timeout_second=90)
            return ret

    def _adb_reboot(self):
        g_common_obj.adb_cmd("reboot")
        time.sleep(3)
        self._check_display_status("on", self.SET_TIMEOUT)
        g_common_obj.root_on_device()

    def _check_display_status(self, status="on", timeout=60):
        chk_cmd = "dumpsys SurfaceFlinger | grep -iE 'StatusBar|NavigationBar'"
        s_time = time.time()
        while time.time() - s_time < timeout:
            rst = g_common_obj.adb_cmd_capture_msg(chk_cmd)
            time.sleep(1)
            if status == "on":
                if rst != "":
                    LOG.debug("Now display status is on.")
                    time.sleep(3)
                    return True
            elif status == "off":
                if rst == "":
                    LOG.debug("Now display status is off.")
                    time.sleep(3)
                    return True
        raise Exception("Timeout! Fail to get display status.")

    def change_automatic_rotation(self, value=0):
        '''
        :param value: 0 to turn off auto rotation, 1 for turn it on.
        :return: None
        '''
        cmd = "content insert --uri content://settings/system\
         --bind name:s:accelerometer_rotation --bind value:i:%d" % value
        return g_common_obj.adb_cmd_capture_msg(cmd)

    def screen_rotation(self, value=0):
        '''
        :param value: 0 is default; 1 is 90 rotate; 2 is 180; 3 is -90.
        :return: None
        '''
        cmd = "content insert --uri content://settings/system\
         --bind name:s:user_rotation --bind value:i:%d" % value
        return g_common_obj.adb_cmd_capture_msg(cmd)

    def run_instrument_test(self, clsName, pkgName, runner=None):
        if runner is None:
            cmd = "am instrument -w  -r -e class %s %s/android.support.test.runner.AndroidJUnitRunner"\
                % (clsName, pkgName)
        else:
            cmd = "am instrument -w  -r -e class %s %s/%s"\
                % (clsName, pkgName, runner)
        return True if "OK" in g_common_obj.adb_cmd_capture_msg("%s | grep -w 'OK'" % cmd) else False

adb32 = AdbExtension()


class QRcode(object):

    """QRcode"""

    def __init__(self):
        self.device = g_common_obj.get_device()

    def mark_image_qrcode(self, org_img, code='9876543210', count=2, margin=5):
        """mark qrcode to image"""
        mark_png = tempfile.mktemp(suffix='.png', prefix='qrcode_', dir='/tmp')
        out_img = tempfile.mktemp(suffix=os.path.splitext(org_img)[1],
                                  prefix='qrcode_', dir='/tmp')

        def get_scale(img_w, img_h):
            image_wide = img_w if img_w > img_h else img_h
            scales = int(math.ceil(image_wide / 220))
            return scales

        base_img = Image.open(org_img)
        base_w, base_h = base_img.size

        qr = pyqrcode.create(code)
        qr.png(mark_png, scale=get_scale(base_w, base_h))
        mark_img = Image.open(mark_png)
        mark_w, mark_h = mark_img.size

        def in_area(x, y):
            return x >= 0 and y >= 0 \
                and x + mark_w <= base_w and y + mark_w <= base_h

        position_map = []
        row = int(math.ceil(math.sqrt(count)))
        for i in range(row):
            for j in range(row):
                position_map.append((j, i))
        random.shuffle(position_map)

        relative_x = ((base_w - mark_w) / 2) - ((mark_w + margin) * int(math.ceil(row / 2)))
        relative_y = ((base_h - mark_h) / 2) - ((mark_h + margin) * int(math.ceil(row / 2)))

        marked_count = 0
        for each in position_map:
            draw_x = relative_x + (mark_w + margin) * each[0]
            draw_y = relative_y + (mark_h + margin) * each[1]
            if in_area(draw_x, draw_y):
                base_img.paste(mark_img, (draw_x, draw_y))
                marked_count += 1
                if marked_count >= count:
                    break
        assert marked_count, \
            "[FAILURE] Failed mark qrcode"

        base_img.save(out_img)
        remove_temp_file(mark_png)
        return out_img

    def decode_image_qrcode(self, img_file):
        """decode image qrcode"""
        qr = qrtools.QR()
        ret = qr.decode(img_file)
        print "[Debug] qrdecode ret: %s" % (ret)
        return ret, qr.data

qrcode_obj = QRcode()


class WifiController(object):

    """WifiController"""

    def turn_on(self):
        """wifi turn on"""
        cmd = 'svc wifi enable'
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        # for _ in range(10):
        #     state, ssid = self.get_status()
        #     print "[Debug] ssid:%s state:%s" % (ssid, state)
        #     if state == 'COMPLETED':
        #         return
        #     time.sleep(2)
        #
        # assert False, \
        #     "[FAILURE] Wait Wifi connection status time over"

    def turn_off(self):
        """wifi turn off"""
        cmd = 'svc wifi disable'
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

    def get_status(self):
        """get wlan status"""
        cmd = "wpa_cli status"
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        lines = msg.splitlines()
        state, ssid = '', ''
        for line in lines:
            if line.startswith('wpa_state'):
                state = line.split('=')[1].strip("\r\n")

            if line.startswith("ssid"):
                ssid = line.split('=')[1].strip("\r\n")

        return (state, ssid)

wifi_ctrl = WifiController()


class WindowsDisplayInfo(object):

    """
    """
    GET_WM_DENSITY = "wm density"
    GET_WM_SIZE = "wm size"
    DUMPSYS_CMD = "dumpsys display|grep DisplayDeviceInfo"
    FULL_SCREEN_HINT = 'ImmersiveModeConfirmation'

    def __init__(self):
        self.device = g_common_obj.get_device()

    def get_dumpsys_density(self):
        """
        run commands 'adb shell dumpsys display| grep DisplayDeviceInfo' to get dumpsys density,such as density 160
        """
        msg = g_common_obj.adb_cmd_capture_msg(repr(self.DUMPSYS_CMD))
        m = re.search(r'density\s*\w+', msg)
        if m != None:
            dumpsys_density = int(m.group().strip().split(" ")[1])
        return dumpsys_density

    def get_wm_size(self):
        """
        run commands 'adb shell wm size' to get real size,e.g.:Physical size: 1080x1920
        """
        for _ in range(0, 3):
            density = g_common_obj.adb_cmd_capture_msg(repr(self.GET_WM_SIZE))
            if density.find("Override") == -1:
                break
            else:
                g_common_obj.adb_cmd_capture_msg("wm size reset")
        output = g_common_obj.adb_cmd_capture_msg(repr(self.GET_WM_SIZE))
        m = re.search(r'Physical size:\s*\d+x\d+', output)
        if m != None:
            size = m.group().split(":")[1].strip()
            print size
        return size

    def get_wm_density(self):
        """
        run commands 'adb shell wm density' to get real density
        """
        for _ in range(0, 3):
            density = g_common_obj.adb_cmd_capture_msg(repr(self.GET_WM_DENSITY))
            if density.find("Override") == -1:
                break
            else:
                g_common_obj.adb_cmd_capture_msg("wm density reset")
        output = g_common_obj.adb_cmd_capture_msg(repr(self.GET_WM_DENSITY))
        m = re.search(r'Physical density:\s*\w+', output)
        if m != None:
            density = int(m.group().split(":")[1].strip())
        return density

    def get_wm_width(self):
        size = self.get_wm_size()
        width = int(size.split("x")[0].strip())
        print width
        return width

    def get_wm_hight(self):
        size = self.get_wm_size()
        hight = int(size.split("x")[1].strip())
        print hight
        return hight

    def get_dumpsys_dpi(self):
        """
        run commands 'adb shell dumpsys display| grep DisplayDeviceInfo' to get dpi,such as 149.824 x 149.411 dpi
        """
        msg = g_common_obj.adb_cmd_capture_msg(repr(self.DUMPSYS_CMD))
        m = re.search(r'\w+.\w*\s*x\s*\w+.\w*\s*dpi', msg)
        if m != None:
            dumpsys_dpi = m.group()
        return dumpsys_dpi

    def get_dumpsys_backlight(self):
        msg = g_common_obj.adb_cmd_capture_msg("dumpsys display |grep mActualBacklight")
        if msg != None:
            actualBacklight = int(msg.split("=")[1].strip())
            print "[Debug] actualBacklight is %s" % actualBacklight
            return actualBacklight

    def disable_fullscreen_hint(self):
        cmd = "dumpsys window|grep mCurrentFocus"
        strings = g_common_obj.adb_cmd_capture_msg(cmd)
        if self.FULL_SCREEN_HINT in strings:
            time.sleep(1)
            self.device.swipe(self.device.info['displayWidth'] / 2,
                                0,
                                self.device.info['displayWidth'] / 2,
                                self.device.info['displayHeight'] / 2,
                                steps=20)

windows_info = WindowsDisplayInfo()


class SettingsCLI(object):

    def set_key_value(self, namespace, key, value):
        assert namespace in ['system', 'secure', 'global'],\
            "namespace is one of {system, secure, global}"

        cmd = 'settings get %s %s' % (namespace, key)
        output = g_common_obj.adb_cmd_capture_msg(cmd)
        assert output != 'null', 'Invalid key %s' % key

        cmd = 'settings put %s %s %s' % (namespace, key, value)
        output = g_common_obj.adb_cmd_capture_msg(cmd)

    def get_key_value(self, namespace, key):
        cmd = 'settings get %s %s' % (namespace, key)
        output = g_common_obj.adb_cmd_capture_msg(cmd)
        assert output != 'null', 'Invalid key %s' % key
        return output


class DBSettingsSetGet(object):

    def __init__(self):
        self.device = g_common_obj.get_device()

    def set_screen_off_timeout(self, screen_off_timeout=60000):
        screen_off_timeout = int(screen_off_timeout)
        for _ in range(0, 3):
            g_common_obj.adb_cmd_capture_msg("settings put system screen_off_timeout %d" % screen_off_timeout)
            self.device.screen.on()
            time.sleep(1)
            if self.get_screen_off_timeout() == screen_off_timeout:
                break

    def get_screen_off_timeout(self):
        output = int(g_common_obj.adb_cmd_capture_msg("settings get system screen_off_timeout"))
        print "[Debug] get screen off timeout is %s" % output
        time.sleep(1)
        return output

    def set_status_stay_awake(self, status):
        """
        parameter status is True or False
        """
        for _ in range(0, 3):
            if status == True:
                g_common_obj.adb_cmd_capture_msg("settings put global stay_on_while_plugged_in 3")
                if self.get_status_stay_awake() == True:
                    break
            elif status == False:
                g_common_obj.adb_cmd_capture_msg("settings put global stay_on_while_plugged_in 0")
                if self.get_status_stay_awake() == False:
                    break
        self.device.screen.on()

    def get_status_stay_awake(self):
        output = int(g_common_obj.adb_cmd_capture_msg("settings get global stay_on_while_plugged_in"))
        if output == 3:
            print"[Debug] device stay awake is True"
            return True
        elif output == 0:
            print "[Debug] device staty awake is False"
            return False

    def set_brightness_level(self, level):
        if level < 0:
            print "[WARNING] The brightness should be in [0, 255]!"
            print "[INFO] Set brightness to 0!"
            level = 0
        elif level > 255:
            print "[WARNING] The brightness should be in [0, 255]!"
            print "[INFO] Set brightness to 255!"
            level = 255
        level = int(level)
        for _ in range(0, 3):
            output = g_common_obj.adb_cmd_capture_msg("settings put system screen_brightness %s" % level)
            if self.get_brightness_level() == level:
                break

    def get_brightness_level(self):
        output = int(g_common_obj.adb_cmd_capture_msg("settings get system screen_brightness"))
        print "[Debug]get brightness level is %s" % output
        return output

    def set_color_inversion(self, enabled=False):
        cmdstr = "settings put secure accessibility_display_inversion_enabled %d" % enabled
        g_common_obj.adb_cmd_capture_msg(cmdstr)
        adb32.adb_shell_stop()
        adb32.adb_shell_start()

    def get_color_inversion(self):
        cmdstr = "settings get secure accessibility_display_inversion_enabled"
        val = int(g_common_obj.adb_cmd_capture_msg(cmdstr))
        LOG.debug("Got color inversion status: %d" % val)
        return val

    def set_color_corretion(self, enabled=False):
        cmdstr = "settings put secure accessibility_display_daltonizer_enabled %d" % enabled
        g_common_obj.adb_cmd_capture_msg(cmdstr)
        adb32.adb_shell_stop()
        adb32.adb_shell_start()

    def get_color_corretion(self):
        cmdstr = "settings get secure accessibility_display_daltonizer_enabled"
        val = int(g_common_obj.adb_cmd_capture_msg(cmdstr))
        LOG.debug("Got color corretion status: %d" % val)
        return val

    def set_color_corretion_mode(self, mode=11):
        cmdstr = "settings put secure accessibility_display_daltonizer %d" % mode
        return g_common_obj.adb_cmd_capture_msg(cmdstr)

    def get_color_corretion_mode(self):
        cmdstr = "settings get secure accessibility_display_daltonizer"
        val = int(g_common_obj.adb_cmd_capture_msg(cmdstr))
        LOG.debug("Got color corretion mode: %d" % val)
        return val

    def set_force_resizable_activities(self, enabled=False):
        cmdstr = "settings put global force_resizable_activities %d" % enabled
        return g_common_obj.adb_cmd_capture_msg(cmdstr)

    def get_force_resizable_activities(self):
        cmdstr = "settings get global force_resizable_activities"
        val = int(g_common_obj.adb_cmd_capture_msg(cmdstr))
        LOG.debug("Got force resizable activities status: %d" % val)
        return val

    def get_skip_first_use_hints(self):
        cmdstr = "settings get secure skip_first_use_hints"
        val = g_common_obj.adb_cmd_capture_msg(cmdstr)
        LOG.debug("Got skip_first_use_hints status: %s" % val)
        return val

    def set_skip_first_use_hints(self, enabled=False):
        cmdstr = "settings put secure skip_first_use_hints %d" % enabled
        return g_common_obj.adb_cmd_capture_msg(cmdstr)

dbsetting = DBSettingsSetGet()


def get_current_focus_window():
    """get current activity focus window"""
    cmd = "dumpsys window|grep mCurrentFocus"
    pattern = r"(?P<PKG_NAME>[\w.]+)/(?P<ACT_NAME>[\w.]+)}"
    packagename, activityname = '', ''
    for _ in range(3):
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        match = re.search(pattern, msg)
        if not match:
            time.sleep(1)
            continue
        packagename = match.group('PKG_NAME')
        activityname = match.group('ACT_NAME')
        break
    return packagename, activityname


def rotate_screen(mode='wide'):
    """rotate screen to wide or narrow mode"""
    device = g_common_obj.get_device()

    def is_wide():
        return device.info['displayHeight'] < device.info['displayWidth']
    for way in ['l', 'r', 'n']:
        if mode == 'wide' and is_wide():
            return
        elif mode == 'narrow' and not is_wide():
            return
        device.orientation = way


def close_all_tasks(sleep_after_close=1):
    LOG.debug("Close all tasks")

    d = g_common_obj.get_device()
    d.press.home()
    d.press("recent")
    nr_of_dismissed_tasks = 0
    try:
        while d(resourceId='com.android.systemui:id/dismiss_task').wait.exists(timeout=5000):
            d(resourceId='com.android.systemui:id/dismiss_task').click()
            time.sleep(sleep_after_close)
            nr_of_dismissed_tasks += 1
        LOG.debug("Closed %s" % (nr_of_dismissed_tasks))
    except Exception as e:
        LOG.warning(str(e))
        LOG.warning("Failed UI close all tasks")


def get_screenshot_region(bounds=g_common_obj.get_device()().bounds):
    """get screen's region image"""
    screenshot_img = tempfile.mktemp(suffix='.png', prefix='temp_screen_', dir='/tmp')
    region_img = tempfile.mktemp(suffix='.png', prefix='region_', dir='/tmp')
    g_common_obj.take_screenshot(screenshot_img)
    assert os.path.isfile(screenshot_img)
    image = Image.open(screenshot_img)
    box = (bounds["left"], bounds["top"], bounds["right"], bounds["bottom"])
    region = image.crop(box)
    region.save(region_img)
    remove_temp_file(screenshot_img)
    return region_img

def is_device_screen_on():
    """check device screen is bright"""
    cmd = 'dumpsys power'
    power_status = g_common_obj.adb_cmd_capture_msg(repr(cmd))
    return 'mScreenOn=true' in power_status\
        or'mScreenOn=SCREEN_ON_BIT' in power_status\
        or 'Display Power: state=ON' in power_status


def id_generator(size=10, chars=string.ascii_uppercase + string.digits):
    """generate random id"""
    return ''.join(random.choice(chars) for _ in range(size))


def remove_temp_file(fullname):
    """remove temp file"""
    try:
        if os.path.exists(fullname):
            os.remove(fullname)
    except os.error:
        print "[Warning] %s" % (sys.exc_info()[1])


def launch_settings_am(subsettings=''):
    activityName = "com.android.settings.Settings" \
        if subsettings == '' else "com.android.settings.Settings\$%s" % subsettings
    cmdstr = 'am start -S -n com.android.settings/%s' % activityName
    g_common_obj.adb_cmd('\'' + cmdstr + '\'')
    time.sleep(1)


def launch_aosp_home():

    pkg_name = "com.google.android.apps.nexuslauncher"
    activity_name = "com.android.launcher3.Launcher"

    def get_device():
        return g_common_obj.get_device()

    def install_launcher():
        if not pkg_name in pkgmgr.get_packages():
            apk_path = get_resource_from_atifactory("tests.tablet.artifactory.conf", "aosp_launcher", "apk")
            pkgmgr.apk_install(apk_path)

    device = get_device()
    cmdstr = "am start -c android.intent.category.HOME -a android.intent.action.MAIN -n %s/%s" \
            %(pkg_name, activity_name)
    install_launcher()
    g_common_obj.adb_cmd_capture_msg(cmdstr)
    device.press.home()
    time.sleep(2)
    if device(text="Notification access").exists:
        if device(text="Launcher3").right(resourceId="android:id/switch_widget", text="OFF").exists:
            device(text="Launcher3").right(resourceId="android:id/switch_widget").click.wait()
            time.sleep(1)
        if device(text="ALLOW").exists:
            device(text="ALLOW").click.wait()
            time.sleep(1)
            device.press.back()
    if device(text="Launcher3"):
        device(text="Launcher3").click.wait()
        time.sleep(1)
    device.press.back() * 3
    assert pkg_name in get_current_focus_window(), "Fail to show aosp launcher."


def remove_aosp_launcher():
    return g_common_obj.adb_cmd_capture_msg("pm uninstall com.google.android.apps.nexuslauncher")


def stop_settings_am():
    """ Stop Settings via adb am command
    """
    print "Stop Settings by adb am"
    g_common_obj.stop_app_am("com.android.settings")


def unlock_screen():
    device = g_common_obj.get_device()
    for _ in range(0, 3):
        if device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.down()
        if device(description="Unlock").exists:
            device(description="Unlock").drag.to(resourceId="com.android.systemui:id/clock_view")
        time.sleep(3)
        if device(description="Unlock") is None:
            break


def get_resource_from_atifactory(conf_name, section, resource_name):
    # usage: et_resource_from_atifactory("tests.tablet.artifactory.conf", "content_picture", "wbmp")
    config = TestConfig()
    cfg_file = conf_name
    cfg_arti = config.read(cfg_file, 'artifactory')
    config_handle = ConfigHandle()
    cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
    cfg = config.read(cfg_file, section)
    arti = Artifactory(cfg_arti.get('location'))
    pic_name = cfg.get(resource_name)
    file_path = arti.get(pic_name)
    return file_path


class OsVersion(object):

    """GetAndroidVersion"""

    CONFIG_FILE = 'tests.common.busybox.conf'

    def __init__(self):
        self.device = g_common_obj.get_device()

    def get_product_name(self):
        cmd = 'getprop ro.product.name'
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        return msg

    def get_android_version(self):
        """adb shell getprop | grep ro.build.version.release"""
        array = []
        cmd = 'getprop | grep ro.build.version.release | cut -d ":" -f2 | cut -d "[" -f2 | cut -d "]" -f1'
        cmd1 = 'getprop | grep ro.build.version.release | cut -d ":" -f2 | cut -d "[" -f2 | cut -d "]" -f1 | cut -d "." -f1'
        cmd2 = 'getprop | grep ro.build.version.release | cut -d ":" -f2 | cut -d "[" -f2 | cut -d "]" -f1 | cut -d "." -f2'
        cmd3 = 'getprop | grep ro.build.version.release | cut -d ":" -f2 | cut -d "[" -f2 | cut -d "]" -f1 | cut -d "." -f3'
#         androidversion, androidversion_mini, androidversion_micro = 0, 0, 0
        androidversion = g_common_obj.adb_cmd_capture_msg(cmd)
        androidversion_large = g_common_obj.adb_cmd_capture_msg(cmd1)
        if androidversion_large.isdigit():
            androidversion_mini = g_common_obj.adb_cmd_capture_msg(cmd2)
            androidversion_micro = g_common_obj.adb_cmd_capture_msg(cmd3)
            androidversion_large = 0 if len(androidversion_large) == 0 else int(androidversion_large)
            androidversion_mini = 0 if len(androidversion_mini) == 0 else int(androidversion_mini)
            androidversion_micro = 0 if len(androidversion_micro) == 0 else int(androidversion_mini)
        else:
            androidversion_large = androidversion_large.upper()
            print androidversion_large
            androidversion_large = ord(androidversion_large) - 71
            androidversion_mini = 0
            androidversion_micro = 0

        array.append(androidversion_large)
        array.append(androidversion_mini)
        array.append(androidversion_micro)
        print array
        return array

osversion = OsVersion()


class PackageManager(object):

    def __init__(self):
        self.cfg_file = 'tests.common.packages.conf'
        self.config = TestConfig()

        self.cli_settings = SettingsCLI()

    def get_packages(self):
        cmd = "pm list packages"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        packages = re.findall(r"package:([\w.]+)", msg)
        return packages

    def get_package_path(self, pkg_name):
        packages = self.get_packages()
        assert pkg_name in packages, "%s not in 'pm list packages'" % (pkg_name)

        cmd = 'pm path %s' % (pkg_name)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        match = re.match(r"package:(.+)", msg)
        assert match, "unavailable package path: %s" % (msg)

        return match.group(1)

    def clear_package_data(self, pkg_name):
        cmd = 'pm clear %s' % (pkg_name)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        assert 'success' in msg.lower(), "package data clean failed"

    def pull_package(self, apk_path, target_path=None):
        if not target_path:
            target_path = tempfile.mktemp(suffix='.apk', prefix='apk_', dir='/tmp')
        LOG.debug('adb pull %s' % apk_path)
        assert g_common_obj.pull_file(target_path, apk_path), 'Failed pull file'
        return target_path

    def get_latest_resource(self, package, api_level=None):
        def parser(name):
            pattern = r"v#(?P<version>[\w.]+),m#(?P<api_min>[\w.]+),t#(?P<api_max>[\w.]+)"
            match = re.match(pattern, name)
            return match.groupdict()
        links = self.config.read(self.cfg_file, package)
        packages = {k: v for k, v in links.iteritems()}

        filter_keys = []
        for row in links.iterkeys():
            each = parser(row)
            if api_level:
                if api_level >= each['api_min'] and api_level <= each['api_max']:
                    filter_keys.append(each)
            else:
                filter_keys.append(each)
        latest = sorted(filter_keys, key=lambda row: row['version'], reverse=True)[0]
        return packages.get("v#%s,m#%s,t#%s" % (latest['version'], latest['api_min'], latest['api_max']))

    def grant_permissions(self, package):
        g_common_obj.adb_cmd_capture_msg("pm grant %s android.permission.READ_EXTERNAL_STORAGE" % (package))
        g_common_obj.adb_cmd_capture_msg("pm grant %s android.permission.WRITE_EXTERNAL_STORAGE" % (package))

        g_common_obj.adb_cmd_capture_msg("pm grant %s android.permission.CAMERA" % (package))
        g_common_obj.adb_cmd_capture_msg("pm grant %s android.permission.RECORD_AUDIO" % (package))

        g_common_obj.adb_cmd_capture_msg("pm grant %s android.permission.READ_CALENDAR" % (package))
        g_common_obj.adb_cmd_capture_msg("pm grant %s android.permission.WRITE_CALENDAR" % (package))

    def dowanload_apk(self, package, api_level=None):

        resource_path = self.get_latest_resource(package, api_level)
        cached_path = artifactory.get(resource_path)
        return cached_path

    def apk_install(self, file_path):
        self.cli_settings.set_key_value('global', 'package_verifier_enable', '0')
        self.cli_settings.set_key_value('secure', 'install_non_market_apps', '1')
        # pkginfo = self.dump_package(file_path)
        g_common_obj.adb_cmd_common('install -r -g %s' % file_path)
        time.sleep(10)
        # assert pkgmgr._package_installed(pkginfo.packageName), "Fail to install apk."
        # self.grant_permissions(pkginfo.packageName)

    def dump_package(self, apk_path):
        assert os.path.isfile(apk_path), "%s is not a file" % (apk_path)

        class InfoParser(object):

            def __init__(self, msg):
                match = re.search(r"package:.* name='(.+?)'", msg, re.IGNORECASE)
                assert match, "Can't parse package name"
                self.packageName = match.group(1)

                match = re.search(r"package:.* versionCode='(.+?)'", msg, re.IGNORECASE)
                assert match, "Can't parse package versionCode"
                self.versionCode = match.group(1)

                match = re.search(r"package:.* versionName='(.+?)'", msg, re.IGNORECASE)
                assert match, "Can't parse package versionName"
                self.versionName = match.group(1)

                match = re.search(r"\nsdkversion:'(.+?)'", msg, re.IGNORECASE)
                assert match, "Can't parse package sdkversion"
                self.sdkVersion = match.group(1)

                match = re.search(r"targetSdkVersion:'(.+?)'", msg, re.IGNORECASE)
                assert match, "Can't parse package targetSdkVersion"
                self.targetSdkVersion = match.group(1)

            def __str__(self):
                attrs = vars(self)
                return ', '.join("%s: %s" % (k, v) for k, v in attrs.items())

        cmd = "aapt dump badging %s" % (apk_path)
        LOG.debug('Execute command: %s' % cmd)

        ret, msg, errmsg = shell_command_ext(cmd)
        assert ret == 0, 'Failed dump package reason: %s' % (errmsg)
        return InfoParser(msg)

    def _package_installed(self, pkgName=None):
        cmd = "pm list package | grep -I %s" % pkgName
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        return True if pkgName in msg else False

    def hide_package(self, packageName):
        if packageName in self.get_packages():
            try:
                g_common_obj.adb_cmd_capture_msg('pm hide %s' % packageName)
                adb32._adb_reboot()
            except:
                raise Exception("Fail to hide given package.")

pkgmgr = PackageManager()


class FileSystem(object):

    hwc_display_file = '/vendor/etc/hwc_display.ini'
    CONFIG_FILE = 'tests.tablet.artifactory.conf'

    def __init__(self):
        g_common_obj.root_on_device()
        busybox_obj.setup()  # using busybox for full-args find

    def get_file_list(self, folder=None):
        cmd = "/data/busybox find %s" % folder
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        if "No such file or directory" in result:
            return None
        else:
            _list = [i for i in result.splitlines() if '.' in i]
            LOG.debug("Get file as listed below:\n %s" % _list)
            return _list

    def get_security_context(self, target=None):
        pass

    def is_HDMI_connected(self):
        root_path = "/sys/class/drm/"
        cmdstr = "find %s ! -path %s" % (root_path, root_path)
        folders = [i for i in g_common_obj.adb_cmd_capture_msg(cmdstr).splitlines()]
        LOG.info("Get list as below:\n %s" % folders)
        hdmi_status = []
        for _ in folders:
            if "HDMI" in _:
                result = g_common_obj.adb_cmd_capture_msg("cat %s/status" % _)
                hdmi_status.append(result)
        print hdmi_status
        assert hdmi_status.count('connected') >= 1, "Device is not connected with HDMI display."

    def get_file_context(self, matchCase=True, file_path='', keyword='', extend=False, ext_cmd=''):
        para = 'I' if matchCase else 'i'
        if extend:
            cmd = "cat %s | grep -%s %s | %s" % (file_path, para, keyword, ext_cmd)
        else:
            cmd = "cat %s | grep -%s %s" % (file_path, para, keyword)
        ret = g_common_obj.adb_cmd_capture_msg(cmd)
        print "----------Got context from %s:  %s " % (file_path, ret)
        return ret

    def switch_hwc_display(self, head, value):
        adb32.adb_disable_verity()
        tmp_file = '/vendor/etc/hwc_tmp.ini'
        g_common_obj.adb_cmd_capture_msg('rm -rf %s' % tmp_file)
        cmdstr = "\'cat %s | sed \'s/^%s=.*$/%s=\\\\\\\"%s\\\\\\\"/g\' | tee %s\'" \
                 % (self.hwc_display_file, head, head, value, tmp_file)
        clean_cmdsrt = "\'rm -rf %s; mv %s %s\'" % (self.hwc_display_file, tmp_file, self.hwc_display_file)
        g_common_obj.adb_cmd_capture_msg(cmdstr)
        g_common_obj.adb_cmd_capture_msg(clean_cmdsrt)
        chk_str = head + '\\=\\\"' + value + '\\\"'
        print chk_str
        assert len(self.get_file_context(True, self.hwc_display_file, chk_str)) > 0, 'Fail to switch hwc display.'

    def init_hwc_display(self):
        adb32.adb_disable_verity()
        hwc_display_file_local = get_resource_from_atifactory(self.CONFIG_FILE, 'hwc_display', 'file_path')
        LOG.debug('Push init hwc_display.ini to /vendor/etc/')
        g_common_obj.shell_cmd('adb push %s %s' % (hwc_display_file_local, self.hwc_display_file))
        adb32.adb_shell_stop()
        adb32.adb_shell_start()

    def rescan_file(self, file):
        cmdstr = "am broadcast -a android.intent.action.MEDIA_SCANNER_SCAN_FILE -d file:///mnt%s" % file
        return g_common_obj.adb_cmd_capture_msg(cmdstr)

file_sys = FileSystem()


class MultiDisplay(object):

    def __init__(self):
        self.d = g_common_obj.get_device()
        self.PRIMARY_DISPLAY_ID = 0
        self.EXTERNAL_DISPLAY_ID = 1
        g_common_obj.adb_cmd_capture_msg('pm disable android.car.cluster.sample')

    def is_multi_displayed(self):
        assert not logcat.check_dumpsys_SurfaceFlinger_info(keyword="Display.*entries.*",
                                                        assertword=("0" and "1")), \
            "Not detected external display."

    def get_display_number(self):
        cmd = "dumpsys SurfaceFlinger | grep Display.*entries.* | grep -oP '\d+'"
        dp_num = g_common_obj.adb_cmd_capture_msg(cmd)
        return int(dp_num)

    def is_clone_mode(self):
        # Capture 1 / 4 area for both display and compare with each other to find if
        # they're the same picture.
        LOG.debug('Check device if is in clone mode.')
        sx, sy, ex, ey = 0, 0, self.d.info['displayWidth'] / 2, self.d.info['displayHeight'] / 2
        img1 = self.get_screenshot_multidisplay_croped(self.PRIMARY_DISPLAY_ID, sx, sy, ex, ey)
        img2 = self.get_screenshot_multidisplay_croped(self.EXTERNAL_DISPLAY_ID, sx, sy, ex, ey)
        assert compare_pic.compare_pic(img1, img2) < 2, "Not in clone mode."
        return True

    def is_show_different_picture(self):
        # Capture 1 / 4 area for both display and compare with each other to find if
        # they're the same picture.
        LOG.debug('Check device if shows different pictures.')
        sx, sy, ex, ey = 0, 0, self.d.info['displayWidth'] / 2, self.d.info['displayHeight'] / 2
        img1 = self.get_screenshot_multidisplay_croped(self.PRIMARY_DISPLAY_ID, sx, sy, ex, ey)
        img2 = self.get_screenshot_multidisplay_croped(self.EXTERNAL_DISPLAY_ID, sx, sy, ex, ey)
        assert compare_pic.compare_pic(img1, img2) > 20, "Got same picture on those displays."
        return True

    def is_mosaic_mode(self):
        pass

    def is_presentation(self, retry=3):
        # Capture middle area for both display and compare with each other to find if
        # they're in the presentation mode.
        LOG.debug('Check device if is in presentation mode.')
        cx, cy = self.d.info['displayWidth'] / 2, self.d.info['displayHeight'] / 2
        sx, sy, ex, ey = cx - 300, cy - 50, cx + 100, cy + 50
        primary_count, external_count = 0, 0
        for i in range(1, retry + 1):
            str_primary = compare_pic.extract_strings_from_croped_screen_shot(self.PRIMARY_DISPLAY_ID, sx, sy, ex, ey)
            if '' in str_primary:
                break
            else:
                primary_count += 1
        assert primary_count != retry, "Unexpected picture displayed in primary side."
        target_external_strs = ['photo', '1', '#0', 'sp']
        for i in range(1, retry + 1):
            str_external = compare_pic.extract_strings_from_croped_screen_shot(self.EXTERNAL_DISPLAY_ID, sx, sy, ex, ey)
            if any(i in str_external for i in target_external_strs):
                break
            else:
                external_count += 1
        assert external_count != retry, "Unexpected picture displayed in external side."

    def is_presentation_with_mediarouter(self):
        primary_text = compare_pic.extract_strings_from_screen_shot(self.PRIMARY_DISPLAY_ID)
        external_text = compare_pic.extract_strings_from_screen_shot(self.EXTERNAL_DISPLAY_ID)
        assert len(primary_text) > 0, "Media is displaying on primary screen."
        assert len(external_text) == 0, "Media is not displaying on external screen."
        return True

    def get_screenshot_forMultiDisplay(self, displayId=0):
        tmp_img = tempfile.mktemp(suffix='.png', prefix='temp_screen_', dir='/tmp')
        screenshot_img = "/sdcard/screenshot.png"
        g_common_obj.adb_cmd_capture_msg("screencap -d %d %s" % (displayId, screenshot_img))
        g_common_obj.pull_file(tmp_img, screenshot_img)
        g_common_obj.adb_cmd_capture_msg("rm %s" % screenshot_img)
        LOG.info("Save screenshot: %s" % tmp_img)
        return tmp_img

    def get_screenshot_multidisplay_croped(self, displayId=0, sx=0, sy=0, ex=100, ey=100):
        tmp_img = self.get_screenshot_forMultiDisplay(displayId)
        img = Image.open(tmp_img)
        img2 = img.crop((sx, sy, ex, ey))
        img2.save(tmp_img)
        return tmp_img

    def run_activities_multidisplay(self, displayId, pkg_name, activity_name):
        cmdstr = "am start -S -n %s/%s --display %d" % (pkg_name, activity_name, displayId)
        return g_common_obj.adb_cmd_capture_msg(cmdstr)

multi_display = MultiDisplay()


if __name__ == "__main__":
    print PackageManager().dowanload_apk('com.google.android.apps.photos')
