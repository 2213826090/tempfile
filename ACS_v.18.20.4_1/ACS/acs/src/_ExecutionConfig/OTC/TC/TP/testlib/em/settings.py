# -*- coding: utf-8 -*-
import os
import time
from constants_def import *
from tools import *

class Settings(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.action = "android.settings.SETTINGS"

    def launch(self):
        self.testDevice.stop_app_am("com.android.settings")
        self.testDevice.adb_cmd("am start -a %s" % self.action)
        time.sleep(2)

    def get_status_bar_rect(self):
        self.launch()
        bounds = self.d(resourceId="android:id/statusBarBackground").bounds
        rect = (bounds["right"] * 3 / 4, 0, bounds["right"], bounds["bottom"])
        print rect
        return rect

class AirplaneModeSetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.WIRELESS_SETTINGS"

    def switch_airplane(self, status):
        '''
        status: 'ON' or 'OFF'
        '''
        self.launch()
        if self.d(text="Airplane mode").right(className="android.widget.Switch").text.upper() != status:
            self.d(text="Airplane mode").right(className="android.widget.Switch").click()

    def check_airplane_status(self, status):
        if not self.d(text="Airplane mode").exists:
            self.launch()
        print "[info]--- Check airplane %s" % status
        assert self.d(text="Airplane mode").right(className="android.widget.Switch").text.upper() == status

class WifiSetting(Settings):

    def __init__(self, serial = None, ssid = None, passwd = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.WIFI_SETTINGS"
        self.ssid = ssid
        self.passwd = passwd

    def switch_wifi(self, status):
        param = "enable" if status.upper() == "ON" else "disable"
        cmd = "svc wifi %s" % param
        self.testDevice.adb_cmd(cmd)

    def get_wifi_status(self):
        self.launch()
        return self.d(className="android.widget.Switch").checked

    def read_wifi_conf(self, conf_tag = "wifi_adb"):
        cfg = get_config_by_tag(conf_tag)
        self.ssid = cfg.get("ssid")
        self.passwd = cfg.get("passwd")

    def check_wifi_connect(self):
        cmd = "dumpsys connectivity | grep 'CONNECTED/CONNECTED.*%s'" % self.ssid
        if self.testDevice.adb_cmd_capture_msg(cmd):
            return True
        return False

    def add_network(self):
        version = self.get_build_version()
        if version.startswith("7") or version.startswith("8") or version == "O":
            if version == "O":
                if self.d(resourceId="com.android.settings:id/list").exists:
                    self.d(resourceId="com.android.settings:id/list").scroll.to(text="See all networks")
                self.d(text="See all networks").click()
            if self.d(scrollable=True).exists:
                self.d(scrollable=True).scroll.vert.toEnd()
        else:
            self.d.press("menu")
        self.d(text="Add network").click()
        self.d(resourceId="com.android.settings:id/ssid").set_text(self.ssid)
        self.d(className="android.widget.Spinner").click()
        self.d(text="WPA/WPA2 PSK").click()
        self.d(resourceId="com.android.settings:id/password").set_text(self.passwd)
        self.d(resourceId="android:id/button1").click()
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.vert.toBeginning()

    def wait_wifi_connected(self):
        for i in range(10):
            if self.d(text=self.ssid).exists:
                if self.d(text=self.ssid).sibling(textStartsWith="Connected").exists:
                    return
                else:
                    self.d(text=self.ssid).click()
                    if self.d(text="Connect").exists:
                        self.d(text="Connect").click()
                    elif self.d(text="CONNECT").exists:
                        self.d(text="CONNECT").click()
                    else:
                        self.d.press.back()
            time.sleep(5)

    def connect_none_secured_wifi(self):
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text=self.ssid)
        self.d(text=self.ssid).click()

    def connect_wifi(self):
        if self.check_wifi_connect():
            return
        self.launch()
        if not self.d(className="android.widget.Switch").checked:
            self.d(className = "android.widget.Switch").click()
            time.sleep(10)
            assert self.d(className="android.widget.Switch").checked
            if self.check_wifi_connect():
                return
        if not self.passwd:
            self.connect_none_secured_wifi()
        else:
            self.add_network()
        self.wait_wifi_connected()

    def connect_wifi_by_conf(self, wifi_conf):
        self.read_wifi_conf(wifi_conf)
        for i in range(3):
            try:
                self.connect_wifi()
                return True
            except Exception as e:
                print e.message
        else:
            return False

    def ping_remote_host(self, host, count = 1):
        cmd = "ping -c %s %s | grep -E icmp_seq.+time.+ms" % (count, host)
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        if msg != "":
            return True
        return False

    def get_wifi_ip(self):
        wifi_ip = self.testDevice.adb_cmd_capture_msg("getprop dhcp.wlan0.ipaddress")
        if not wifi_ip:
            cmd = "dumpsys connectivity | grep LinkAddresses"
            result = self.testDevice.adb_cmd_capture_msg(cmd)
            import re
            p=re.compile(".*LinkAddresses: .*[^\d](\d+\.\d+\.\d+\.\d+)/24,.*\]")
            m = p.match(result)
            wifi_ip = m.group(1)
        return wifi_ip

    def enable_wifi_adb(self):
        wifi_ip = self.get_wifi_ip()
        port = 5555
        serial_wifi = "%s:%s" % (wifi_ip, port)
        for i in range(5):
            self.testDevice.adb_cmd_common("tcpip %s" % port, 30)
            msg = self.testDevice.adb_cmd_common("connect %s" % serial_wifi, 30)
            if msg.startswith("connected"):
                break
            time.sleep(3)
        else:
            assert False
        print "[info]--- device wifi serial: %s" % serial_wifi
        return serial_wifi


class BTSetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.BLUETOOTH_SETTINGS"
        self.send_file_process = None
        self.agent_register = False

    def host_setup(self):
        from testaid.bluez5.process import Logger
        from testaid.bluez5.agent import Agent
        Logger.set_enable(True)
        self.agent = Agent.init_from_remote(capability="NoInputNoOutput")
        self.agent.register()
        self.agent_register = True

    def host_teardown(self):
        if self.agent_register:
            self.agent.unregister()
            self.agent.destroy()
        if self.send_file_process != None:
            self.send_file_process.kill()
            self.send_file_process = None

    def get_host_alias(self):
        from testaid.bluez5.manager import BTManager
        self.adapter = BTManager().default_adapter()
        return self.adapter.Alias

    def get_device_bdaddr(self):
        product = self.get_product()
        if product in [BXT_M]:
            cmd = "cat /sys/class/bluetooth/hci0/address"
        elif product in [BXT_O]:
            cmd = "dumpsys bluetooth_manager | grep address | awk '{print $2}'"
        else:
            cmd = "getprop persist.service.bdroid.bdaddr"
        bdaddr = self.testDevice.adb_cmd_capture_msg(cmd)
        print "[Info] Device bdaddr", bdaddr
        return bdaddr

    def switch_bt(self, status):
        self.launch()
        if self.d(className="android.widget.Switch").text.upper() != status:
            self.d(className = "android.widget.Switch").click()

    def get_bt_status(self):
        self.launch()
        text = self.d(className="android.widget.Switch").text
        return text.upper() == "ON"

    def bt_search(self, name):
        if self.get_build_version() == O_MR1:
            if self.d(text=name).exists:
                return True
            self.d(text="Pair new device").click()
        elif not self.d(resourceId="com.android.settings:id/scanning_progress").exists:
            self.d.press("menu")
            self.d(text="Refresh").click()
        for _ in range(10):
            time.sleep(10)
            if not self.d(resourceId="com.android.settings:id/scanning_progress").exists:
                break
            if self.d(text=name).exists:
                return True
        return self.d(text=name).exists

    def bt_pair(self, name):
        self.d(text=name).click()
        for _ in range(10):
            time.sleep(5)
            if self.d(text=name).right(resourceId="com.android.settings:id/deviceDetails"):
                break
            elif self.d(text=name).right(resourceId="com.android.settings:id/settings_button"):
                break
        else:
            assert False

    def get_paired_status(self, name):
        if self.d(text=name).right(resourceId="com.android.settings:id/deviceDetails"):
            return True
        elif self.d(text=name).right(resourceId="com.android.settings:id/settings_button"):
            return True
        return False

    def host_send_file_to_device(self):
        import subprocess
        file_path = download_artifactory_content("long_music")
        file_name = os.path.basename(file_path)

        bdaddr = self.get_device_bdaddr()
        cmd = ["ussp-push", bdaddr + "@", file_path, file_name]
        print ' '.join(cmd)
        self.send_file_process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def device_receive_file(self):
        self.d.open.notification()
        for _ in range(10):
            time.sleep(2)
            if self.d(text="ACCEPT").exists:
                self.d(text="ACCEPT").click()
                break
            if self.d(text="Bluetooth share: Incoming file").exists:
                self.d(text="Bluetooth share: Incoming file").click()
                time.sleep(2)
                self.d(text="Accept").click()
                return
        else:
            assert False
        self.d.press("back")

    def get_file_transfer_status(self):
        if None == self.send_file_process.poll():
            return True
        return False


class DisplaySetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.DISPLAY_SETTINGS"

    def set_sleep_mode(self, mode):
        modelist = mode.split()
        t = int(modelist[0]) * 1000
        if modelist[1].startswith('minute'):
            t *= 60
        cmd = "dumpsys power | grep 'mScreenOffTimeoutSetting='"
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        msglist =msg.split('=')
        if int(msglist[1]) == t:
            return
        self.launch()
        for i in range(5):
            if self.d(text="Sleep").exists:
                self.d(text = "Sleep").click()
                break
            time.sleep(2)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text=mode)
        for i in range(5):
            if self.d(text = mode).exists:
                self.d(text = mode).click()
                break
            time.sleep(2)

    def set_brightness_level(self, level):
        self.launch()
        self.d(text="Brightness level").click.wait()
        rect = self.d(description="Display brightness").info["bounds"]
        y = (int(rect["top"]) + int(rect["bottom"])) / 2
        xr = int(rect["right"])
        xl = int(rect["left"])
        x = xl + (xr - xl) * int(level) / 255
        xc = (xl + xr)/2
        self.d.swipe(xc, y, x, y, steps = 100)

    def set_brightness_auto(self, status = "OFF"):
        self.launch()
        time.sleep(5)
        if self.d(text="Adaptive brightness").right(className="android.widget.Switch").text.upper() != status:
            self.d(text="Adaptive brightness").right(className="android.widget.Switch").click()
        time.sleep(2)

    def set_keep_awake(self, status):
        DeveloperSetting().set_keep_awake(status)

    def set_auto_rotate(self, status):
        AccessibilitySetting().set_auto_rotate(status)


class BatterySetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.intent.action.POWER_USAGE_SUMMARY"

    def get_battery_level(self):
        self.launch()
        time.sleep(5)
        level = self.d(resourceId = "com.android.settings:id/charge").text
        level = level.split("%")[0]
        return int(level)

    def get_battery_status(self):
        self.launch()
        time.sleep(5)
        status = self.d(resourceId = "com.android.settings:id/estimation").text
        return status

    def check_battery_usage_data(self):
        self.launch()
        time.sleep(5)
        print "[info]--- Check battery usage data in settings"
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.vert.toEnd()
        if self.d(text="Use since last full charge").exists:
            if self.d(text="Battery usage data isn’t available.").exists:
                self.d(description="Refresh").click()
                return False
            return True
        return False


class BatterySaverSetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.BATTERY_SAVER_SETTINGS"


class AccessibilitySetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.ACCESSIBILITY_SETTINGS"

    def set_auto_rotate(self, status):
        self.launch()
        self.d(scrollable=True).scroll.to(text="Auto-rotate screen")
        if self.d(text="Auto-rotate screen").right(className="android.widget.Switch").checked != status:
            self.d(text="Auto-rotate screen").right(className="android.widget.Switch").click()


class LocationSetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.LOCATION_SOURCE_SETTINGS"

    def switch_GPS(self, status):
        self.launch()
        time.sleep(2)
        if self.d(text="Agree").exists:
            self.d(text="Agree").click()
        if self.d(className="android.widget.Switch").text.upper() != status:
            self.d(className = "android.widget.Switch").click()
            if status == "ON" and self.d(text = "Agree").exists:
                self.d(text = "Agree").click.wait()

    def set_mode(self, mode = "High accuracy"):
        if self.d(text=mode, resourceId="android:id/summary").exists:
            return
        self.d(text="Mode").click()
        self.d(text=mode).click()
        if mode == "High accuracy":
            time.sleep(5)
            while self.d(resourceId="android:id/button1").exists:
                self.d(resourceId="android:id/button1").click()
                time.sleep(2)


class SecuritySetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.SECURITY_SETTINGS"
        self.lock = "Swipe"
        self.encrypt = None

    def change_lock_screen_status(self, status):
        self.launch()
        if self.d(text="Screen lock").down(text=status) != None:
            return
        self.d(text="Screen lock").click()
        for i in range(5):
            if self.d(text=status).exists:
                self.d(text=status).click()
                break
            time.sleep(1)
        assert self.d(text="Screen lock").down(text=status) != None


class DateSetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.DATE_SETTINGS"

    def set_property(self, auto = False, use_24 = True):
        self.launch()
        if self.d(text="Use network-provided time").right(className="android.widget.Switch").checked != auto:
            self.d(text="Use network-provided time").click()
        if self.d(text="Use 24-hour format").right(className="android.widget.Switch").checked != use_24:
            self.d(text="Use 24-hour format").click()

    def set_time(self, h_min = "23:59"):
        self.launch()
        self.d(text="Set time").click()
        h, m = h_min.split(":")
        self.d(resourceId="android:id/radial_picker").child(index=int(h)).click()
        self.set_minute(m)
        self.d(text="OK").click()

    def set_minute(self, minute_str):
        from math import sin, cos, pi
        bounds = self.d(description="0").bounds
        x_0 = (bounds["left"] + bounds["right"]) / 2.0
        y_0 = (bounds["top"] + bounds["bottom"]) / 2.0
        bounds = self.d(description="30").bounds
        x_30 = x_0
        y_30 = (bounds["top"] + bounds["bottom"]) / 2.0
        oy = (y_0 + y_30) / 2.0
        r = oy - y_0
        minute = int(minute_str)
        for m in [minute, minute - 0.5, minute + 0.5, minute - 1, minute + 1]:
            angle = m * pi / 30
            x = x_0 + r * sin(angle)
            y= oy - r * cos(angle)
            self.d.click(x, y)
            if self.d(resourceId="android:id/minutes").text == minute_str:
                return
        assert False, "set minute failed"


class DeveloperSetting(Settings):

    def __init__(self, serial = None):
        Settings.__init__(self, serial)
        self.action = "android.settings.APPLICATION_DEVELOPMENT_SETTINGS"

    def select_usb_option(self, option):
        """
        O MR1
        """
        self.launch()
        self.d(scrollable=True).scroll.to(text="Select USB Configuration")
        self.d(text="Select USB Configuration").click()
        bounds = self.d(textStartsWith=option).bounds
        x = (bounds["left"] + bounds["right"]) / 2
        y = (bounds["top"] + bounds["bottom"]) / 2
        cmd = "input tap %s %s" % (x, y)
        self.testDevice.adb_cmd(cmd)

    def set_keep_awake(self, status):
        self.launch()
        status = status.upper()
        if self.d(text="Stay awake").right(className="android.widget.Switch").text != status:
            self.d(text="Stay awake").right(className="android.widget.Switch").click.wait()
            assert self.d(text="Stay awake").right(className="android.widget.Switch").text == status

