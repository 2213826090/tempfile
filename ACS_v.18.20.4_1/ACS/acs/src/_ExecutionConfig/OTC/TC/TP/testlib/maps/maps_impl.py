"""
@summary: module for maps application
@since: 10/2/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""

from testlib.util.common import g_common_obj
import time
import os

class MapsImpl:
    """
    google maps functions.
    """
    googlemap_package = "com.google.android.apps.maps"
    googlemap_activity = "com.google.android.maps.MapsActivity"
    setting_package = "com.android.settings"
    setting_activity = ".Settings"

    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_sign_in(self):
            """ UI button sign in """
            return self.d(text = "Sign in")

        @property
        def btn_yes_im_in(self):
            """ UI button yes im """
            return self.d(text = "Yes, I'm in")

        @property
        def btn_off(self):
            """ UI button off """
            return self.d(text = "Off")

        @property
        def btn_on(self):
            """ UI button on """
            return self.d(text = "On")

        @property
        def btn_OFF(self):
            """ UI button OFF """
            return self.d(text = "OFF")

        @property
        def btn_switch(self):
            """ UI button switch """
            return self.d(resourceId="com.android.settings:id/switch_widget", enabled=True)

        @property
        def btn_checkbox(self):
            """ UI button checkbox """
            return self.d(resourceId="com.google.android.gms:id/checkbox")


        @property
        def btn_status(self):
            """ UI button switch """
            return self.d(
                resourceId="com.android.settings:id/switch_text").\
            info.get("text").decode('gbk')

        @property
        def btn_location(self):
            """ UI button location """
            return self.d(text = "Location")

        @property
        def btn_agree(self):
            """ UI button agree """
            return self.d(text = "Agree")

        @property
        def btn_mode(self):
            """ UI button mode """
            return self.d(text = "Mode")

        @property
        def btn_high_accuracy(self):
            """ UI button high accuracy """
            return self.d(text = "High accuracy")

        @property
        def btn_accept_continue(self):
            """ UI button accept continue """
            return self.d(text = "Accept & continue")

        @property
        def btn_text1(self):
            """ UI resource id text1 """
            return self.d(resourceId = "android:id/text1")

        @property
        def btn_my_location(self):
            """ UI button my location """
            return self.d(resourceId = "com.google.android.apps.gmm:id/mylocation_button")

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = MapsImpl.Locator(self.d)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        g_common_obj.set_vertical_screen()

    def launch_from_am(self,app_name):
        """
            Launch app from am
        """
        print "[INFO] Launch app from am"
        if app_name == "googlemap" :
            g_common_obj.launch_app_am(MapsImpl.googlemap_package, \
                MapsImpl.googlemap_activity)
        elif app_name == "settings" :
            g_common_obj.launch_app_am(MapsImpl.setting_package, \
                MapsImpl.setting_activity)
        time.sleep(2)

    def stop_from_am(self,app_name):
        """
            Stop google maps from am
        """
        print "[INFO] Stop Keep from am"
        if app_name == "googlemap" :
            g_common_obj.stop_app_am(MapsImpl.googlemap_package)
        elif app_name == "settings" :
            g_common_obj.launch_app_am(MapsImpl.setting_package)

    def quit_app(self):
        for i in range(3):
            self.d.press.back()

    def open_location(self):
        self.launch_from_am("settings")
        time.sleep(2)
        self._locator.btn_location.click.wait()
        if self._locator.btn_off.exists:
            self._locator.btn_off.click.wait()
            time.sleep(2)
            self._locator.btn_agree.click.wait()
            time.sleep(2)
        if self._locator.btn_OFF.exists:
            self._locator.btn_OFF.click.wait()
            time.sleep(2)
            self._locator.btn_agree.click.wait()
            time.sleep(2)
        self._locator.btn_mode.click.wait()
        self._locator.btn_high_accuracy.click.wait()
        if self._locator.btn_agree.exists:
            self._locator.btn_agree.click.wait()
        time.sleep(2)
        self.quit_app()

    def gps_location(self):
        """
        This test used to test GPS function.
        The test case spec is following:
        1. Launch the "google map".
        2. Check GPS lock is acquired by finding my location in Maps.
        """

        self.launch_from_am("googlemap")
        time.sleep(5)
        if self._locator.btn_accept_continue.exists:
            self._locator.btn_accept_continue.click.wait()
        time.sleep(10)
        if self._locator.btn_sign_in.exists:
            self._locator.btn_sign_in.click.wait()
            time.sleep(2)
        if self._locator.btn_yes_im_in.exists:
            self._locator.btn_yes_im_in.click.wait()
            time.sleep(3)
        if self._locator.btn_text1.exists:
            self._locator.btn_text1.click.wait()
            time.sleep(10)
        self._locator.btn_my_location.click.wait()
        time.sleep(10)
        self.quit_app()

    def gps_switch(self, repeat=1):
        """
            switch gps on/off
        """
        self.gps_prepare()
        assert self._locator.btn_switch.exists
        status=self._locator.btn_on if self._locator.btn_on.exists else self._locator.btn_off
        for i in range (int(repeat)*2):
            if i%2==0:
                print "click %d" % (i/2+1)
            #self._locator.btn_switch.click()
            if self._locator.btn_on.exists:
                self._locator.btn_switch.click()
                if not self._locator.btn_off.wait.exists(timeout=60000):
                    self._locator.btn_switch.click()
                assert self._locator.btn_off.wait.exists(timeout=60000)
    
            elif self._locator.btn_off.exists:
                self._locator.btn_switch.click()
                if not self._locator.btn_on.wait.exists(timeout=60000):
                    self._locator.btn_switch.click()
                assert self._locator.btn_on.wait.exists(timeout=60000)
            else:
                print "[WARNING] GPS status is [%s]" % self._locator.btn_status
        time.sleep(2)
        self.quit_app()

    def gps_prepare(self):
        """
        prepare
        """
        self.launch_from_am("settings")
        time.sleep(2)
        self._locator.btn_location.click()
        if self._locator.btn_on.exists:
            self._locator.btn_switch.click()
        if self._locator.btn_off.exists:
            self._locator.btn_switch.click()
        if self._locator.btn_agree.wait.exists(timeout=3000):
            self._locator.btn_checkbox.click()
            self._locator.btn_agree.click()
            time.sleep(2)