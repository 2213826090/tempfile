#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: Class for longitude and latitude
@since: 10/21/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

import os
import time
import ConfigParser
from testlib.util.common import g_common_obj

class LLTUDE(object):

    pkg_name = "com.kaiwidment.latlon"
    activity_name = "com.kaiwidment.latlon.MainActivity"
    app_name = "Latitude & Longitude"

    class Locator(object):

        def __init__(self, device):
            self.d = device

        @property
        def btn_lltitle(self):
            """ the title """
            return self.d(resourceId="android:id/action_bar_title", \
                textMatches="Latitude & Longitude")

        @property
        def welcome(self):
            """ the title """
            return self.d(resourceId="android:id/action_bar_title", \
                textMatches="Welcome!")

        @property
        def accept(self):
            """ the title """
            return self.d(textMatches="Accept!")

        @property
        def latitude(self):
            """ the latitude """
            return self.d(textMatches="Latitude")

        @property
        def latitude_info(self):
            """ the latitude """
            return self.latitude.down(\
                resourceId="com.kaiwidment.latlon:id/subtitle").info

        @property
        def longitude(self):
            """ the longitude """
            return self.d(textMatches="Longitude")

        @property
        def longitude_info(self):
            """ the longitude """
            return self.longitude.down(\
                resourceId="com.kaiwidment.latlon:id/subtitle").info

    def __init__(self, cfg):
        self.d = g_common_obj.get_device()
        self.locator = LLTUDE.Locator(self.d)
        self.cfg = cfg
        base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)
        self.apk_file = (os.sep).join\
        (base_path+['com.kaiwidment.latlon-1.apk'])

    def launch_by_am(self):
        """ launch Google Map by adb am command """
        self.d.watcher("SKILL_WELCOME").\
        when(textMatches="Welcome!").press("back")
        g_common_obj.launch_app_am(LLTUDE.pkg_name, LLTUDE.activity_name)
        time.sleep(3)
        if not self.focus_window(self.pkg_name):
            self.install()
            g_common_obj.launch_app_am(LLTUDE.pkg_name, LLTUDE.activity_name)
            time.sleep(3)
        assert self.wait_exist(self.locator.btn_lltitle, 60), "Launch failed!"

    def launch_from_app_gallery(self):
        """ launch Google Map from UI """
        self.d.watcher("SKILL_WELCOME").\
        when(textMatches="Welcome!").press("back")
        g_common_obj.launch_app_from_home_sc(LLTUDE.app_name, "Apps", None)
        time.sleep(3)
        if not self.focus_window(self.pkg_name):
            self.install()
            g_common_obj.launch_app_from_home_sc(LLTUDE.app_name, "Apps", None)
            time.sleep(3)
        assert self.wait_exist(self.locator.btn_lltitle, 60), "Launch failed!"

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=60):
            """ wait """
            return uiobj.wait("exists", timeout*1000)
        return _wait

    def get_latitude(self):
        """
        @summary: get latitude
        """
        self.wait_exist(self.locator.latitude, 60)
        latitude = self.locator.latitude_info.get("text")
        print "The Latitude is %s" % latitude
        latitude = latitude.split(".")
        return int(latitude[0])

    def get_longitude(self):
        """
        @summary: get longitude
        """
        self.wait_exist(self.locator.longitude, 60)
        longitude = self.locator.longitude_info.get("text")
        print "The Longitude is %s" % longitude
        longitude = longitude.split(".")
        return int(longitude[0])

    def install(self):
        """
        @summary: install ll apk
        """
        print "[INFO] Install apk"
        self.d.watcher("Accept").\
        when(textMatches="Accept!").click(textMatches="Accept!")
        cmd = "install " + self.apk_file
        message = g_common_obj.adb_cmd_common(cmd, 120)
        print "Install message:", message
        assert "Success" in message, message

    def uninstall(self):
        """
        @summary: uninstall ll apk
        """
        print "[INFO] Uninstall apk"
        cmd = "uninstall " + self.pkg_name
        message = g_common_obj.adb_cmd_common(cmd, 120)
        print "Uninstall message:", message
        assert "Success" in message, message

    @staticmethod
    def focus_window(package):
        """
        @summary: check focus package
        """
        cmd = "dumpsys window|grep mCurrentFocus"
        message = g_common_obj.adb_cmd_capture_msg(cmd)
        print "[INFO] Current Focus window is %s" % message
        if package in message:
            return True
        return False

    def check_location(self):
        """
        @summary: check location
        """
        long_conf = int(self.cfg.get("longitude"))
        lat_conf = int(self.cfg.get("latitude"))
        deviation = int(self.cfg.get("deviation"))
        longitude = self.get_longitude()
        latitude = self.get_latitude()
        lo_message = "The logitude[%d] is not in range %d, %d" % \
                (longitude, long_conf-deviation ,long_conf+deviation)
        la_message = "The latitude[%d] is not in range %d, %d" % \
        (latitude, lat_conf-deviation ,lat_conf+deviation)
        assert longitude in range (long_conf-deviation, long_conf+deviation), \
        lo_message
        assert latitude in range (lat_conf-deviation, lat_conf+deviation), \
        la_message
        print "The longitude is in range %d, %d" % \
        (long_conf-deviation ,long_conf+deviation)
        print "The latitude is in range %d, %d" % \
        (lat_conf-deviation, lat_conf+deviation)

    def teardown(self):
        """tear down"""
        self.d.watcher("SKILL_WELCOME").remove()
        self.d.watcher("Accept").remove()