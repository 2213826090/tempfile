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
@summary: ChromeExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import time
import re

from testlib.util.config import TestConfig
from testlib.util.process import shell_command
from testlib.util.common import g_common_obj
from testlib.util.repo import Artifactory
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl


class ChromeCastImpl(object):

    """ChromeCast uiautomator """

    config_file = 'tests.common.chromecast.conf'

    class MenuUI(object):

        """Main frame UI"""

        def __init__(self, device):
            self.device = device

        @property
        def left_drawer(self):
            return self.device(resourceId="com.google.android.apps.chromecast.app:id/left_drawer")

        def show_menu(self):
            y = self.device.info["displayHeight"]
            x = self.device.info["displayWidth"]
            self.device.swipe(x / 2, y - 20, 0, y - 20, steps=5)
            if not self.left_drawer.exists:
                self.device.swipe(0, y / 2, x / 2, y / 2, steps=5)
                time.sleep(1)

        @property
        def cast_screen(self):
            self.show_menu()
            return self.device(text="Cast screen / audio")

        @property
        def devices(self):
            self.show_menu()
            return self.device(text="Devices")

    class CastScreen(object):

        """Page Cast Screen"""

        def __init__(self, device):
            self.device = device

        @property
        def scan_castscreen(self):
            return self.device(resourceId="com.google.android.apps.chromecast.app:id/mirror_button")

        @property
        def list(self):
            return self.device(resourceId="com.google.android.apps.chromecast.app:id/mr_chooser_route_name")

        @property
        def disconnect(self):
            return self.device(resourceId="com.google.android.apps.chromecast.app:id/disconnect_button")

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.menu = ChromeCastImpl.MenuUI(self.device)
        self.castscreen = ChromeCastImpl.CastScreen(self.device)
        self.systemui = SystemUiExtendImpl()
        self.config = TestConfig()

    def skip_welcome(self):
        self.device.watcher("AUTO_FC_WHEN_ANR")\
            .when(text='Accept').click(text='Accept')
        if self.device(text="OK, got it").exists:
            self.device(text="OK, got it").click()
        if self.device(textContains='No Thanks').exists:
            self.device(textContains='No Thanks').click()
        if self.device.watcher("AUTO_FC_WHEN_ANR").triggered:
            print "[Debug] Skip Welcome"

    def setup(self):
        cfg = self.config.read(self.config_file, "ChromeCast")
        arti = Artifactory(cfg.get('artifactory_location'))
        apk_path = arti.get(cfg.get("chromecast_apk"))

        cmd = "install -r %s" % (apk_path)
        print g_common_obj.adb_cmd_common(cmd)

    def launch(self):
        """Launch app"""
        g_common_obj.launch_app_am('com.google.android.apps.chromecast.app',
                                   'com.google.android.apps.chromecast.app.DiscoveryActivity')
        self.skip_welcome()

    def resume(self):
        """Resume app"""
        cmdstr = "am start %s/%s" % ('com.google.android.apps.chromecast.app',
                                     'com.google.android.apps.chromecast.app.DiscoveryActivity')
        g_common_obj.adb_cmd(cmdstr)
        self.skip_welcome()

    def scan_cast_screen(self):
        devices = []
        self.menu.cast_screen.click()
        if self.device(text="OK").exists:
            self.device(text="OK").click.wait()
        time.sleep(2)
        if self.castscreen.scan_castscreen.exists:
            self.castscreen.scan_castscreen.click()
        time.sleep(10)

        for idx in range(len(self.castscreen.list)):
            title = self.castscreen.list[idx].text
            devices.append({'title': title, })
        print "[Debug] cast_screen  %s" % (devices)

        return devices

    def choose_castscreen(self, model):
        devices = self.scan_cast_screen()
        assert len(devices), \
            "[FAILURE] There is no any cast device"

        return devices[0]['title']

    def _connect_cast_screen(self, name):
        if self.device(text=name).exists:
            self.device(text=name).click()

        for _ in range(1, 21):
            time.sleep(1)
            if self.castscreen.disconnect.exists:
                return
        assert False, \
            "[FAILURE] Wait cast connection time over"

    def disconnect_cast_screen(self):
        self.menu.cast_screen.click()
        if self.castscreen.disconnect.exists:
            self.castscreen.disconnect.click()

    def verify_connection_from_quick_settings(self, name):
        self.systemui.unlock_screen()
        self.device.screen.on()
        self.device.press.home()
        self.device.open.quick_settings()
        self.device.wait.update()
        status = self.device(textContains="Connected to %s" % (name)).exists
        assert status, \
            "[FAILURE] Not found Connected inf in Quick Settings"
        self.device.press.back()
        self.device.press.back()


class ChromeCastNexusPlayerImpl(ChromeCastImpl):

    def __init__(self, model):
        super(ChromeCastNexusPlayerImpl, self).__init__()

        self.serial = self.detect_device(model)
        self.model = model

        cfg = self.config.read(self.config_file, "NexusPlayer")
        self.logcat_connected_pattern = cfg.get('logcat_connected_pattern')

    def adb_shell(self, cmd):
        _cmd = "adb -s %s shell %s" % (self.serial, cmd)
        msgs = shell_command(_cmd)[1]
        return ''.join(msgs).strip('\r\n')

    def detect_device(self, model):
        re_device = re.compile('^([a-zA-Z0-9_:.-]+)\tdevice$', re.MULTILINE)
        result = g_common_obj.adb_cmd_common('devices')
        devices = re_device.findall(result)
        for serial in sorted(devices):
            _cmd = "adb -s %s shell getprop ro.product.model" % (serial)
            msgs = shell_command(_cmd)[1]
            ret = ''.join(msgs).strip('\r\n')
            if ret == model:
                return serial
        assert False, 'There is no %s device' % model

    def get_ip_address(self):
        cmd = "getprop dhcp.wlan0.ipaddress"
        return self.adb_shell(cmd)

    def get_device_time_mark(self):
        cmd = 'date +"%m-%d %H:%M:%S"'
        date = self.adb_shell(cmd)
        time.sleep(1)
        return date

    def get_device_log(self, time_mark=None):
        cmd = 'logcat -v time -t "%s.000"' % (time_mark)
        log = self.adb_shell(cmd)
        return log

    def connect_cast_screen(self, name):
        mark_time = self.get_device_time_mark()
        super(ChromeCastNexusPlayerImpl, self)\
            ._connect_cast_screen(name)
        cast_log = self.get_device_log(mark_time)
        print "[Debug] Find log pattern %s" % (self.logcat_connected_pattern)
        catch_log = re.findall(self.logcat_connected_pattern, cast_log)
        print '-' * 60 + '\n'
        print catch_log
        print '-' * 60 + '\n'
        assert catch_log, \
            "[FAILURE] Connect failed"

    def watch_connection_status(self, count=20, step=60):
        cast_device_ip = self.get_ip_address()
        print "[Debug] CAST Device ip: %s" % (cast_device_ip)
        cmd_watch = "netstat|grep %s|grep -v ::ffff --color=no" % (cast_device_ip)
        for i in range(1, count + 1):
            print "[Debug] Idle %s %s secs" % (i, step)
            time.sleep(step)
            msg = g_common_obj.adb_cmd_capture_msg(repr(cmd_watch))
            print msg
            assert 'ESTABLISHED' in msg, \
                "[FAILURE] Lost connections ! in the %s secs" % (i * step)
