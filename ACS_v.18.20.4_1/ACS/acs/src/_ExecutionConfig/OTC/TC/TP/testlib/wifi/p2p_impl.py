#!/usr/bin/env python
#-*- encoding: utf-8 -*-
#Copyright (C) 2014D
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
@summary: Class for P2P settings
@since: 07/10/2014
"""

import os
import time
import subprocess

from testlib.util.device import TestDevice


class P2PImpl:
    """
    Implements P2P UI actions.

    """

    class Locator:
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, serial):
            self.device = TestDevice(serial).get_device()
        @property
        def btn_ok(self):
            """ UI button OK """
            return self.device(text="OK")

        @property
        def btn_accept(self):
            """ UI button Accept """
            return self.device(text="Accept")

        @property
        def btn_connected(self):
            """ UI button connected """
            return self.device(text="Connected")

        @property
        def btn_connected_in_apk(self):
            """ UI button connected in apk"""
            return self.device(text="Connected",
                resourceId="com.everwasproductions.demo.wifidirect:id/my_status")

        def btn_group_role_in_apk(self):
            """ UI button group role in apk"""
            return self.device(
                resourceId="com.everwasproductions.demo.wifidirect:id/group_owner").info['text'].decode('gbk')

        @property
        def package_gallery(self):
            """ gallery package """
            return self.device(packageName="com.google.android.apps.plus")

        @property
        def package_viewer(self):
            """ viewer package """
            return self.device(packageName="com.estrongs.android.pop")

        @property
        def group_owner_check(self):
            """ UI button owner check in apk"""
            if "yes" in self.btn_group_role_in_apk():
                return True
            elif "no" in self.btn_group_role_in_apk():
                return False
            else:
                return None

        @property
        def btn_launch_gallery_in_apk(self):
            """ UI button launch gallery """
            return self.device(text="Launch Gallery")

        @property
        def first_picture(self):
            """ UI button first picturn """
            return self.device(
                resourceId="com.android.documentsui:id/icon_thumb", \
                instance="0")

        @property
        def btn_recent(self):
            """ UI button photos """
            return self.device(text="Recent")

        @property
        def wifi_settings(self):
            """ UI 'Wi-Fi' display on Settings, \
            click it enter Wi-Fi settings """
            return self.device(textMatches="Wi.*Fi")

        @property
        def settings_title(self):
            """ UI setting_title"""
            return self.device(resourceId="android:id/action_bar_title", \
                text="Settings")

        @property
        def btn_inactive(self):
            """ UI button inactive """
            return self.device(
                resourceId="com.google.android.googlequicksearchbox:id/inactive")

        @property
        def btn_scroll(self):
            """ scroll icon """
            return self.device(scrollable=True)

        def btn_icon(self, name):
            """ UI icon """
            return self.device(text=name)

        @property
        def btn_apps(self):
            """ UI button apps """
            return self.device(description="Apps")

        def back(self):
            """ back """
            self.device.press.back()

        def back_home(self):
            """back to home screen"""
            self.device.press.back()
            self.device.press.back()
            self.device.press.back()
            self.device.press.home()

        @property
        def btn_more_options(self):
            """ UI button more options """
            return self.device(description="More options")

        @property
        def btn_advanced(self):
            """ UI button advanced setting """
            return self.device(text="Advanced")

        @property
        def btn_wifi_direct(self):
            """ UI button wifi direct """
            return self.device(textMatches="Wi.*Fi Direct")

        @property
        def btn_peer_devices(self):
            """ UI button peer devices """
            return self.device(text="Peer devices")

        @property
        def btn_connected_devices(self):
            """ UI button connected devices """
            if not self.btn_connected.exists:
                return None
            return self.btn_connected.up(
                resourceId="android:id/title")

        def btn_device_in_peers(self, name):
            """ UI button peer devices """
            return self.btn_peer_devices.down(
                textContains=name, resourceId="android:id/title")

        @property
        def btn_peer_devices_signal(self):
            """ UI button peer devices """
            return self.device(
                resourceId="com.android.settings:id/signal")

        def btn_searched_device(self, name):
            """ UI button peer devices """
            return self.btn_peer_devices.down(text=name)

        @property
        def device_name(self):
            """UI device name """
            return self.btn_peer_devices.up(
                resourceId="android:id/title").info["text"].decode('gbk')

        @property
        def btn_group(self):
            """ UI button rembered groups """
            return self.device(text="Remembered groups")

        @property
        def btn_remembered_group(self):
            """ UI button rembered groups """
            if not self.btn_group.exists:
                return None
            return self.btn_group.down(resourceId="android:id/title", index="0")

        @property
        def btn_search(self):
            """ UI button search """
            return self.device(textMatches="Search.*")

        @property
        def btn_rename(self):
            """ UI button rename """
            return self.device(textContains="Rename")

        @property
        def btn_rename_title(self):
            """ UI button rename title """
            return self.device(textContains="Rename device")

        @property
        def btn_rename_bar(self):
            """ UI button rename bar """
            return self.device(className="android.widget.EditText")

        @property
        def btn_disconnect_title(self):
            """ UI button disconnect title """
            return self.device(textMatches="Disconnect.*")

        @property
        def btn_open_with(self):
            return self.device(text="Open with")

        @property
        def btn_photos(self):
            return self.device(text="Photos")

        @property
        def btn_always(self):
            return self.device(resourceId="android:id/button_always")

    def __init__(self, serial, cfg={}):
        """Initialize the environment"""
        self.cfg = cfg
        self.serial = serial
        self._locator = P2PImpl.Locator(self.serial)
        self.g_device = TestDevice(self.serial)

    @staticmethod
    def execute_cmd(command):
        """execute command"""
        lines = os.popen(command).readlines()
        return lines

    @staticmethod
    def execute_no_end_cmd(command, timeout=20):
        """ excute command and kill it while timeout"""
        child = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        time.sleep(timeout)
        child.kill()
        return child.stdout.readlines()

    def set_orientation(self):
        """Set the screen to portrait."""
        self._locator.device.orientation = "n"

    def launch_app_from_home_sc(self, appname):
        """
            restrute for there is no switch widget/apps in app screen
        """
        iffind = False
        self._locator.back_home()
        self._locator.btn_apps.click()
        time.sleep(2)
        count = int(self._locator.btn_inactive.count) + 1
        for i in range(0, count*2):
            time.sleep(2)
            if self._locator.btn_icon(appname).exists:
                self._locator.btn_icon(appname).click()
                iffind = True
                break
            if i < count:
                self._locator.btn_scroll.scroll.horiz()
            else:
                self._locator.btn_scroll.scroll.horiz.backward()
        assert iffind == True

    def launch_wifi_from_settings(self):
        """Launch Wi-Fi from Settings"""
        print "[INFO] Launch Wi-Fi setting from Settings"
        self.launch_app_from_home_sc("Settings")
        self._locator.wifi_settings.click()

    def launch_from_settings(self):
        """Launch Wi-Fi from Settings"""
        print "[INFO] Launch Wi-Fi Direct from Settings"
        msg_fail = "[FAILURE] Launch Wi-Fi Direct fail"
        self.launch_app_from_home_sc("Settings")
        self._locator.wifi_settings.click()
        self._locator.btn_more_options.click()
        self._locator.btn_advanced.click()
        self._locator.btn_wifi_direct.click()
        assert self._locator.btn_group.exists, msg_fail

    def setup(self):
        """ set up test """
        self.set_orientation()
        self._locator.back_home()

    def teardown(self):
        """ back home """
        self.g_device.close_background_apps()
        self._locator.back_home()

    def forget_remembered_group(self):
        """ forget Remembered group """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        while not self._locator.btn_remembered_group is None:
            self._locator.btn_remembered_group.long_click()
            if self._locator.btn_ok.exists:
                self._locator.btn_ok.click()

    def get_device_name(self):
        """ get device name """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        return self._locator.device_name

    def get_group_name(self):
        """ get group name """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        if self._locator.btn_remembered_group is None:
            print "[WARNING] No group remembered!"
            return None
        return self._locator.btn_remembered_group.info['text'].decode('gbk')

    def get_connect_owner(self):
        """ get owner name """
        group_name = self.get_group_name()
        assert not group_name is None
        print "The group name is %s" % group_name
        owner_name = group_name.split("-")[-1]
        print "Group [%s] owner is %s" % (group_name, owner_name)
        return owner_name

    def searching(self):
        """ searching devices """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        self._locator.btn_search.click()

    def check_searching_result(self):
        """ check if the device is found while searching"""
        return self._locator.btn_peer_devices_signal.wait.exists(timeout=30000)

    def connect(self, name):
        """ connect to device """
        print "[INFO] Connect to %s" % name
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        if self._locator.btn_device_in_peers(name) is None:
            print "Not found device %s" % name
            self.searching()
        assert self._locator.btn_device_in_peers(name).exists
        self._locator.btn_device_in_peers(name).click()

    def check_connect(self, name):
        """ check connected to device """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        self._locator.btn_connected.wait.exists(timeout=3000)
        connect = self._locator.btn_connected_devices.info['text'].decode('gbk')
        print connect, name , connect in name
        assert connect in name
        print "[INFO] Device [%s] connected" % name

    def accept_connect(self):
        """ accept connect invited """
        self._locator.btn_accept.click()
        connect = self._locator.btn_connected.wait.exists(timeout=300000)
        assert connect

    def disconnect(self):
        """ disconnect device """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        if not self._locator.btn_connected.exists:
            print "No device connected!"
            return None
        connect_d = self._locator.btn_connected_devices
        connect_d.long_click()
        assert self._locator.btn_disconnect_title.exists
        self._locator.btn_ok.click()
        time.sleep(5)
        assert not self._locator.btn_connected.exists
        print "[INFO] Disconnect"

    def rename(self):
        """ rename the device """
        rename = self.cfg.get("rename")
        if rename is None:
            rename = "Rename_test_" + time.strftime('%H_%M_%S')
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        self._locator.btn_rename.click()
        self._locator.btn_rename_title.wait.exists(timeout=10000)
        self._locator.btn_rename_bar.clear_text()
        self._locator.btn_rename_bar.set_text(rename)
        self._locator.btn_ok.click()
        print "Rename", rename
        print "Original name", self.get_device_name()
        assert rename in self.get_device_name()
        return rename

    def check_searched_device(self, name):
        """ check the device can be searched """
        if not self._locator.btn_peer_devices.exists:
            self.launch_from_settings()
        assert self._locator.btn_searched_device(name)

    def launch_apk(self):
        """ launch wifi direct apk by click """
        self.launch_app_from_home_sc("WiFi Direct")
        if not self._locator.btn_connected_in_apk.exists:
            print "[WARNING]Launch apk failed or no connected devices!"

    def send_file(self):
        """ send file to connect device """
        print "[INFO] Send the file!"
        assert not self._locator.group_owner_check
        self._locator.btn_launch_gallery_in_apk.click()
        if self._locator.btn_recent.wait.exists(timeout=10000):
            self._locator.btn_recent.click()
        self._locator.first_picture.wait.exists(timeout=30000)
        self._locator.first_picture.long_click()

    def recive_file(self):
        """ recive file from connected device """
        if self._locator.btn_open_with.wait.exists(timeout=10000):
            self._locator.btn_photos.click()
            self._locator.btn_always.click()
        assert self._locator.package_gallery.wait.exists(timeout=30000)
        self._locator.back()
        print "[INFO] Recive the file!"

    def view_a_picture(self):
        """ view a screenshot picture """
        print "[INFO] View a picture first!"
        pre_cmd = "adb -s " + self.serial + " shell "
        screen_shot = pre_cmd + "screencap -p /sdcard/Pictures/screen.png"
        view_cmd = pre_cmd + "am start -a android.intent.action.VIEW \
        -d file:///storage/sdcard0/Pictures/screen.png -t image/.png"
        refresh_cmd = pre_cmd + "am broadcast \
        -a android.intent.action.MEDIA_MOUNTED \
        -d file:///mnt/sdcard/Pictures"
        self.execute_no_end_cmd(screen_shot)
        self.execute_no_end_cmd(view_cmd)
        self.execute_no_end_cmd(refresh_cmd)
        assert self._locator.package_viewer.wait.exists(timeout=30000)
        self._locator.back()
