# -*- coding:utf-8 -*-

'''
@summary: GPS test common module.
@since: 06/21/2016
@author: Lijin Xiong
'''

import time
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.process import shell_command
from testlib.dut_init.dut_init_impl import Function


class GPS_Common(object):

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.func=Function()
        self.conf = TestConfig().read(section='wifisetting')
        self.ssid = self.conf.get("ssid")
        self.passwd = self.conf.get("passwd")
        self.security = self.conf.get("security")

    def location_not_get(self):
        g_common_obj.launch_app_am("com.intel.cws.gps", "com.intel.cws.gps.CwsGpsActivityMain")
        self.device(resourceId="com.intel.cws.gps:id/GpsScrollView").swipe.left()
        time.sleep(2)
        self.device(resourceId="com.intel.cws.gps:id/NetworkStartStop").click()
        time.sleep(15)
        self.device(resourceId="com.intel.cws.gps:id/NetworkStartStop").click()
        assert self.device(resourceId="com.intel.cws.gps:id/NetworkLatitudeValue").info[u'text'][0:7] == "000.000"
        assert self.device(resourceId="com.intel.cws.gps:id/NetworkLongitudeValue").info[u'text'][0:7] == "000.000"

    def get_location_and_check_accuracy(self, _latitude, _longitude):
        print _latitude
        print _longitude
        g_common_obj.launch_app_am("com.intel.cws.gps", "com.intel.cws.gps.CwsGpsActivityMain")
        self.device(resourceId="com.intel.cws.gps:id/GpsScrollView").swipe.left()
        time.sleep(2)
        self.device(resourceId="com.intel.cws.gps:id/NetworkStartStop").click()
        for i in range(20):
            if self.device(resourceId="com.intel.cws.gps:id/NetworkStateValue").info['text'] == 'Acquisition':
                time.sleep(i)
            else:
                break
        self.device(resourceId="com.intel.cws.gps:id/NetworkStartStop").click()
        located_latitude = self.device(resourceId="com.intel.cws.gps:id/NetworkLatitudeValue").info[u'text'][0:6]
        located_longitude = self.device(resourceId="com.intel.cws.gps:id/NetworkLongitudeValue").info[u'text'][0:6]
        assert located_latitude in _latitude, "Located latitude is %s not %s, which means location is not accurate or failed!" % (located_latitude, _latitude)
        assert located_longitude in _longitude, "Located longitude is %s not %s, which means location is not accurate or failed!" % (located_longitude, _longitude)

    def Turn_On_Location(self):
        g_common_obj.adb_cmd("am start -a android.settings.LOCATION_SOURCE_SETTINGS")
        if not self.device(className="android.widget.Switch").checked:
            self.device(className="android.widget.Switch").click()
        time.sleep(3)

    def Turn_Off_Location(self):
        g_common_obj.adb_cmd("am start -a android.settings.LOCATION_SOURCE_SETTINGS")
        if self.device(className="android.widget.Switch").checked:
            self.device(className="android.widget.Switch").click()
        time.sleep(3)

    def check_if_wifi_connected(self):
        _cmd = "adb shell dumpsys connectivity | grep 'CONNECTED/CONNECTED'"
        n=3
        try:
            while shell_command(_cmd)[0] and n > 0:
                self.func.connect_AP(self.ssid, self.passwd, self.security)
                n=n-1
                time.sleep(10)
        except:
            print "WiFi reconnection ERROR!"
        assert not shell_command(_cmd)[0], "WiFi is not connected!"

    def check_android_version(self):
        """
        Get Android version:
        adb shell getprop | grep ro.build.version.sdk
        """
        cmd = 'getprop | grep ro.build.version.sdk'
        sdk_string = g_common_obj.adb_cmd_capture_msg(cmd)
        if '23' in sdk_string:
            return "M"
        elif '24' in sdk_string or '25' in sdk_string:
            return "N"

    def fetch_latitude_longitude(self):
        _location = []
        config = TestConfig()
        cfg_file = 'tests.tablet.gps.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'latitude_longitude')
        arti = Artifactory(cfg_arti.get('location'))
        for i in ["loacation_latitude", "loacation_longitude"]:
            txt_name = cfg.get(i)
            txt_path = arti.get(txt_name)
            with open(txt_path, 'r') as _loct:
                _loc_ = _loct.read()
                _location.append(_loc_.split()[0])
        return _location

    def Install_Test_Apps(self, *args):
        _cmdstr_disable = "settings put global package_verifier_enable 0"
        _cmdstr_get_status = "settings get global package_verifier_enable"
        _status = shell_command("adb shell settings get  global package_verifier_enable")[1][0].strip()
        if int(_status):
            g_common_obj.adb_cmd(_cmdstr_disable)
        config = TestConfig()
        cfg_file = 'tests.tablet.gps.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'CWS_GPS')
        arti = Artifactory(cfg_arti.get('location'))
        for i in args:
            apk_name = cfg.get(i)
            apk_path = arti.get(apk_name)
            g_common_obj.adb_cmd_common("install -r %s" % apk_path)

    def init_apk(self):
        M_apps = ["uitests_m", "uitests_m_android", "gps_test_app"]
        N_apps = ["uitests_n", "uitests_n_android", "gps_test_app"]
        if self.check_android_version() == "N":
            print "Android version is N"
            self.Install_Test_Apps(self, *N_apps)
        elif self.check_android_version() == "M":
            print "Android version is M"
            self.Install_Test_Apps(self, *M_apps)

    def init_location(self):
        g_common_obj.adb_cmd("am start -a android.settings.LOCATION_SOURCE_SETTINGS")
        if self.device(text="High accuracy").exists:
            g_common_obj.stop_app_am("com.android.settings")
            return
        elif self.device(text="Mode").exists:
            self.device(text="Mode").click.wait()
        if self.device(text="High accuracy").exists:
            self.device(text="High accuracy").click()
        time.sleep(5)
        if self.device(resourceId="android:id/alertTitle").exists:
            self.device(resourceId="android:id/button1").click()
            time.sleep(3)
        g_common_obj.stop_app_am("com.android.settings")

    def kill_uiautomator(self):
        _uiautomator_pid = shell_command("adb shell ps | grep -i uiautomator | awk '{print $2}'")[1][0]
        g_common_obj.adb_cmd("kill %s" % _uiautomator_pid)
