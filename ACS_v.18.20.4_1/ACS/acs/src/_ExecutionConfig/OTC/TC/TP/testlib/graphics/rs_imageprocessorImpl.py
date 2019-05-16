# -*- coding: utf-8 -*-
'''
Created on Jan 7, 2015

@author: yusux
'''
import os

from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import osversion


class RS_ImageProcessor_Impl(object):

    '''
        Yet another ImageProcessingImpl runs benchmark and check\
        hw_accelerated Methods
        function
    '''
    pkg_name = "com.android.rs.image"
    activity_name = "com.android.rs.image.ImageProcessingActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
#         self._locator = Locator()

    def installImageProcessor(self):
        '''
            install from Artifactory
        '''
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_imageprocessing')
        arti = Artifactory(cfg_arti.get('location'))
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            apk_name = cfg.get("name")
        elif androidversion == 6:
            print "osversion is M"
            apk_name = cfg.get("name")
        elif androidversion == 5:
            print "osversion is L"
            apk_name = cfg.get("name_l")
        else:
            print "osversion is %s" % (androidversion)
            apk_name = cfg.get("name_l")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps(self.pkg_name)
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + str(file_path), 300)

    def run_benchmark(self):
        '''
            run_benchmark
        '''
        SystemUI().unlock_screen()
        g_common_obj.launch_app_am(self.pkg_name, self.activity_name)
        self._device().scroll.vert.to(
            text="Benchmark All")
        if self._device(resourceId="com.android.rs.image:id/benchmarkText").text.endswith("not run"):
            self._device(
                textContains="Benchmark", className="android.widget.Button").click()
            self._device.wait.update()
            print "[Info:] benchmark has a " + self._device(resourceId="com.android.rs.image:id/benchmarkText").text

    def hw_accelerated(self, message):
        """
        @summary: search the message from log
        @parameter:
            message : the message searched
        @return: True or False
        """
        SystemUI().unlock_screen()
        g_common_obj.launch_app_am(self.pkg_name, self.activity_name)
        cmd = "adb logcat -d|grep \"%s\"" % message
        pipe = os.popen(cmd).read().strip()
        if len(pipe) == 0:
            print("Search %s is None" % message)
            return False
        print("INFO:%s" + pipe)
        return True
