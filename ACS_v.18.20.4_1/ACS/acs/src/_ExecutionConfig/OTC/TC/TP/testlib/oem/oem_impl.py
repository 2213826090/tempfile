#!/usr/bin/env python
#-*- encoding: utf-8 -*-
import os
import time
import random
from nose.tools import assert_equals
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.util.process import shell_command
from testlib.util.device import TestDevice
from tests.IRDA_OEM_Customization.init.tools import ConfigHandle
from atk import Text
from uiautomator import Device
from tests.IRDA_OEM_Customization.init.func import OEMFunc

class OEMImpl:
    """
    Implements OEM Setting UI actions.
    """
    wall_package = "com.google.android.googlequicksearchbox"
    wall_activity = "com.google.android.launcher.GelWallpaperPickerActivity"
    set_package = "com.android.settings"
    set_activity = ".Settings"

    def check_apps(self):
        '''
        Implements the apks are visible in app menu after factory reset.
        '''
        print "[Info] --- launch apps menu."
        self.d(description="Apps", \
            className = "android.widget.TextView").click.wait()
        time.sleep(2)
        assert self.d(text = "CustomChoiceList", description = "CustomChoiceList").exists
        print "[Info] --- Custom Choice List is visible in app menu"


    def launch_oem_app(self):
        '''
        Implements the apks are visible to launch.
        '''
        print "[Info] ---Launch Custom Choice List app."
        g_common_obj.stop_app_am("com.example.android.customchoicelist")
        time.sleep(5)
        g_common_obj.launch_app_am("com.example.android.customchoicelist",".MainActivity")
        time.sleep(5)
        self.d(text="Acorn").click()
        time.sleep(2)
        if self.d(textContains="Acorn").right(className="android.widget.ImageView").checked:
            pass
        g_common_obj.stop_app_am("com.example.android.customchoicelist")
        print "[Info] --- Launch Custom Choice List app success"


    def remove_apps(self):
        '''
        Implements the oem apks cannot be removed.
        '''
        print "[Info] ---Launch settings app to check apks."
        for i in range(10):
            self.d.press.home()
            print "[Info] ---Launch home page success"
            if self.d(description="Apps").exists:
                break
            time.sleep(2)
        assert self.d(description="Apps").exists
        time.sleep(2)
        self.d(description="Apps").click()
        print "[Info] ---Launch app page success"
        displayWidth=self.d.info["displayWidth"]
        displayheight=self.d.info["displayHeight"]
        #print "screen width is %s"% displayWidth
        coordinate_x= displayWidth/2
        coordinate_y=displayheight/10
        
        #coordinate=self.d(displayWidth/2)
        print "coordinate is %s %s"% (coordinate_x,coordinate_y)
        self.d(text = "CustomChoiceList", description="CustomChoiceList").drag.to(coordinate_x,coordinate_y)
        print "[Info] --- Open the CustomChoiceList info."
        assert not self.d(text="Uninstall" , \
            resourceId="com.android.settings:id/right_button").exists
        print "[Info] ---There is no unintall button in Custom Choice List"

    class Locator(object):
        """
        Helper class to locate UI object on screen
        """

        def __init__(self, device):
            """Init environment"""
            self.d = device

        @property
        def set_wallpaper(self):
            """ UI 'Set wallpaper' button on screen"""
            return self.d(text="Set wallpaper")

        @property
        def wall_list(self):
            return self.d(resourceId="com.google.android.googlequicksearchbox:id/wallpaper_image")

        @property
        def wall_container(self):
            return self.d(resourceId="android:id/decor_content_parent")

        @property
        def wall_flag(self):
            return self.d(descriptionContains="Wallpaper")

        def wall_spec(self, index, total):
            return self.d(description="Wallpaper %s of %s" % (index, total))

        @property
        def wall_scroll(self):
            return self.d(className="android.widget.HorizontalScrollView")

        @property
        def reset_btn(self):
            return self.d(text="Backup & reset")

        @property
        def reset_btn2(self):
            return self.d(resourceId="com.android.settings:id/initiate_master_clear")

        @property
        def reset_text(self):
            return self.d(text="Factory data reset")

        @property
        def erase_btn(self):
            return self.d(text="Erase everything")

        @property
        def scroll_btn(self):
            return self.d(scrollable=True)

        @property
        def lock_icon(self):
            return self.d(resourceId="com.android.systemui:id/notification_stack_scroller")

        @property
        def clock_icon(self):
            return self.d(resourceId="com.android.systemui:id/clock_view")

        @property
        def ok_btn(self):
            return self.d(text="OK")

        @property
        def guide_line(self):
            return self.d(packageName="com.google.android.setupwizard")

        @property
        def home_flag(self):
            return self.d(description="Apps")
    def __init__(self, cfg):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = OEMImpl.Locator(self.d)
        self.host_file_path=ConfigHandle().read_host_file_path()
    def setup_connection(self):
        g_common_obj.root_on_device()
        value = False
        for i in range(5):
            if self.d.server.alive:
                value = True
                break
            else:
                self.d.server.stop()
                self.d.server.start()
        if value:
            print "Success to setup connection"
            return
        print "Fail to set up connection"

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation="n"

    def wake_up(self):
        print "[INFO]: Unlock screen"
        
        for i in range(10):

            self.d.wakeup()
            time.sleep(1.5)
            name = "com.android.systemui:id/lock_icon"
            if self.d(resourceId=name).exists:
                self.d(resourceId=name).drag.to(resourceId="com.android.systemui:id/clock_view", steps=100)
            if self.d(resourceId=name).exists == False:
                break
        assert self.d(resourceId=name).exists == False

    def launch_from_am(self, packagename, activityname):
        """launch item in the settings pannel"""
        print "Start to launch"
        g_common_obj.stop_app_am(packagename)
        time.sleep(5)
        g_common_obj.launch_app_am(packagename, activityname)
        time.sleep(5)
        print "Success to launch"

    def launch_wallpaper(self):
        g_common_obj.back_home()
        print "Launch Wall paper"
        self.launch_from_am(OEMImpl.set_package, OEMImpl.set_activity)
        time.sleep(2)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "Display",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        print "**********************************************"
        if self.d(text="Wallpaper").exists:
            self.d(text="Wallpaper").click()
        time.sleep(2)
        if self.d(text="Wallpapers").exists:
            self.d(text="Wallpapers").click()
        time.sleep(2)
        assert self._locator.set_wallpaper.exists

    def check_app(self, appName):
        print "Check app exists in apps"
        g_common_obj.back_home()
        self._locator.home_flag.click()
        time.sleep(3)
        assert self.d(text=appName).exists

    def check_oem_apps(self):
        app_dicts = eval(self.cfg.get("oem_apps"))
        for app_package in app_dicts:
            app_activity, app_name = app_dicts[app_package]
            self.check_app(app_name)
            self.launch_from_am(app_package, app_activity)

    def set_oem_app(self):
        app_dicts = eval(self.cfg.get("set_apps"))
        for app in app_dicts:
            push_cmd = "push %s %s" % (self.host_file_path + '/file/' + app, "/oem/app/")
            g_common_obj.adb_cmd_common(push_cmd)
            #check_cmd = "ls /oem/app/%s" % app
            #msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
            #assert msgs.find("No such file or directory") == -1

    def check_set_apps(self):
        app_dicts = eval(self.cfg.get("set_apps"))
        for app in app_dicts:
            app_package, app_activity, app_name = app_dicts[app]
            self.check_app(app_name)
            self.launch_from_am(app_package, app_activity)

    def delete_app(self, packageName, status=True):
        print "Uninstall app"
        cmd = "uninstall %s" % packageName
        msgs = g_common_obj.adb_cmd_common(cmd)
        if status:
            assert msgs.find("Success") != -1
        else:
            assert msgs.find("Success") == -1

    def delete_oem_apps(self):
        app_dicts = eval(self.cfg.get("oem_apps"))
        for app_package in app_dicts:
            app_activity, app_name = app_dicts[app_package]
            self.delete_app(app_package, False)
            self.launch_from_am(app_package, app_activity)

    def check_oem_partition(self):
        print "Check OEM partition"
        check_cmd = "df | grep oem"
        msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
        assert len(msgs) != 0

    def check_oem_device_model(self):
        print "Check oem device_model"
        check_cmd = "getprop ro.product.model"
        msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
        print msgs
        assert len(msgs) != 0

    def local_chinese(self):
        assert self.d(text="欢迎使用").exists, "Error"

    def get_wallpaper_count(self):
        print "Get number of wall paper"
        value = 1
        if self._locator.wall_flag.count != 0:
            tmpStr = self._locator.wall_flag[0].contentDescription
            value = int(tmpStr.split(" ")[-1])
        return value

    def change_wallpaper(self, value=1, picName=None):
        if not self._locator.wall_list.exists:
            print "wall_list exist"
            time.sleep(2)
        total = self.get_wallpaper_count()
        print "total is :%s"%total
        if value > total:
            print "needn't change"
            time.sleep(2)
            self._locator.set_wallpaper.click()
            time.sleep(5)
        else:
            print "start to change"
            value = str(value)
            print "value is : %s"%value
            self._locator.wall_container.click()
            self._locator.wall_scroll.scroll.toBeginning()
            for i in range(50):
                if self._locator.wall_spec(value, total).exists:
                    self._locator.wall_spec(value, total).click()
                    break
                else:
                    self._locator.wall_scroll.scroll.horiz.forward(steps=100)
                time.sleep(2)
            print "selected wallpaper"
            time.sleep(3)
            self._locator.set_wallpaper.click()
            print "set wallpaper"
            time.sleep(5)
        print "to home"
        self.d.press.home()
        print "get home"
        time.sleep(5)
        assert self._locator.home_flag.exists
        if picName:
            print "Take picture: %s" % (self.host_file_path + '/file/' + picName)
            g_common_obj.take_screenshot(self.host_file_path + '/file/' + picName)
            assert picName in os.listdir(self.host_file_path + '/file/')

    def take_picture(self, picName):
        print "Take picture: %s" % (self.host_file_path + '/file/' + picName)
        g_common_obj.take_screenshot(self.host_file_path + '/file/' + picName)
        assert picName in os.listdir(self.host_file_path + '/file/')

    def reboot_device(self):
        print "Reboot device"
        sn=g_common_obj2.getSerialNumber()
        self.serial=sn
        self.func=OEMFunc(self.serial)
        g_common_obj.reboot_device()
        time.sleep(30)
        self.d = g_common_obj.get_device()
        self.setup_connection()
        self.func.wait_for_android_os()
        self.wake_up()
        assert self._locator.home_flag.exists

    def get_device(self, mes):
        return mes.find("\tdevice\n") != -1 or mes.find("\tunauthorized\n") != -1

    def getDevices(self):
        cmdstr="adb devices"
        print cmdstr
        cmd=os.popen(cmdstr).read()
        print cmd
    def getDevices2(self):
        ret, msgs = shell_command("adb devices")
        devices = filter(self.get_device, msgs)
        alllist = sorted(map(lambda x: x.split()[0], devices), key=lambda e: len(e))
        return alllist

    def restart_adb(self):
        shell_command("adb kill-server")
        shell_command("adb start-server")

    def skip_guideline(self):
        print "Skip guideline"
        for i in range(5):
            if self._locator.ok_btn.exists:
                self._locator.ok_btn.click()
                time.sleep(2)

        if self._locator.guide_line.exists:
            self.d(description="Start").click.wait()
            self.d(text="Skip").click.wait()
            self.d(text="Skip anyway").click.wait()
            self.d(text="Next").click.wait()
            self.d.press.back()
            time.sleep(2)
            self.d(text="Next").click.wait()
            if self.d(text="Next").exists:
                self.d(text="Next").click.wait()
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
                self.d(text="Skip anyway").click.wait()
            self.d(text="More").click.wait()
            self.d(text="Next").click.wait()
        if self.d(text="Finish").exists:
            self.d(text="Finish").click.wait()
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()
        for i in range(5):
            if self._locator.ok_btn.exists:
                self._locator.ok_btn.click.wait()
                time.sleep(2)
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        assert self._locator.guide_line.exists is False

    def reset_device(self):
        start_devices = g_common_obj.getAllSerial()
        sn=g_common_obj2.getSerialNumber()
        self.serial=sn
        self.func=OEMFunc(self.serial)
        print "Factory reset device"
        g_common_obj.back_home()
        self.launch_from_am(OEMImpl.set_package, OEMImpl.set_activity)
        time.sleep(2)

        if not self._locator.reset_btn.exists:
            self._locator.scroll_btn.scroll.to(text="Backup & reset")
        self._locator.reset_btn.click()
        time.sleep(2)
        self.d(className="android.widget.ListView", resourceId="android:id/list") \
            .child_by_text(
            "Factory data reset",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        self._locator.reset_text.click()
        time.sleep(2)
        self._locator.reset_btn2.click()
        time.sleep(2)
        self._locator.erase_btn.click()
        self.func.wait_for_android_os()
        time.sleep(5)
        for i in range(10):
            try:
                g_common_obj.root_on_device()
                self.setup_connection()
                self.wake_up()
                g_common_obj.set_vertical_screen()
                self.func.skip_initial_screen_after_factory_reset()
                if self._locator.home_flag.exists:
                    break
                time.sleep(10)
            except Exception, e:
                print "Error, retry to another time"
                print e
                time.sleep(10)

        time.sleep(2)
        assert self._locator.home_flag.exists


    def write_oem(self):
        g_common_obj.root_on_device()
        print "Make oem able to be written"
        write_cmd = self.cfg.get("write_oem")
        g_common_obj.adb_cmd(write_cmd)

    def set_default_wallpaper(self):
        default_wall_path = self.cfg.get("default_wall_path")
        new_wall_name = self.cfg.get("new_wall_name")
        new_wall_path = self.host_file_path + '/file/' + new_wall_name
        check_cmd = "ls %s" % default_wall_path
        msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
        if msgs.find("No such file or directory") == -1:
            print "Remove %s" % default_wall_path
            rm_cmd = "rm -rf %s" % default_wall_path
            g_common_obj.adb_cmd(rm_cmd)
            msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
            assert msgs.find("No such file or directory") != -1

        print "Reset %s" % default_wall_path
        push_cmd = "push %s %s" % (new_wall_path, default_wall_path)
        g_common_obj.adb_cmd_common(push_cmd)
        msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
        assert msgs.find("No such file or directory") == -1

    def unwrite_oem(self):
        g_common_obj.root_on_device()
        print "Make oem unable to be written"
        read_cmd = self.cfg.get("read_oem")
        g_common_obj.adb_cmd(read_cmd)

    def launch_user(self):
        print "Launch users in settings"
        g_common_obj.back_home()
        self.launch_from_am(OEMImpl.set_package, OEMImpl.set_activity)
        time.sleep(2)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "Users",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        time.sleep(2)
        assert self.d(description="More options").exists

    def switch_to_owner(self):
        self.launch_user()
        print "Switch users to owner"
        value = self.d(text="Owner").sibling(resourceId="android:id/title").text
        if value.startswith("You"):
            print "No need to switch to user"
        else:
            self.d(text="Owner").click()
            time.sleep(5)
        self.wake_up()
#         result = self.d(text="Owner").sibling(resourceId="android:id/title").text
#         assert result.startswith("You")

    def switch_to_user(self, name="Users"):
        '''
        Add and switch to new user.
        '''
        self.launch_user()
        sn=g_common_obj2.getSerialNumber()
        self.serial=sn
        self.func = OEMFunc(self.serial)
        print "Add new user"
        if not self.d(text=name, resourceId="android:id/title").exists:
            self.d(text="Add user or profile").click()
            time.sleep(1)
            self.d(text="User").click.wait()
            self.d(text="OK").click.wait()
            self.d(text="Set up now").click.wait()
            time.sleep(2)
            self.func.wait_for_android_os()
            self.wake_up()
            TestDevice().skip_initial_screen_after_factory_reset()
            time.sleep(3)
            self.launch_user()
            time.sleep(2)
        print "Switch to user"
        #if self.d(text=name, resourceId="android:id/title").exists:
        #    self.d(text=name, resourceId="android:id/title").click()
        #    time.sleep(5)
        #    self.wake_up()
        #assert self.d(text="You (%s)" % name).exists or self.d(text="You (New user)").exists


    def go_through_guideline(self, name="Users"):
        '''
        Go through guideline.
        '''
        print "Go through guideline."
        for i in range(5):
            if self._locator.ok_btn.exists:
                self._locator.ok_btn.click()
                time.sleep(2)
        if self.d(description="Start").exists:
            self.d(description="Start").click.wait()
            self.d(text="Skip").click.wait()
            self.d(text="Skip anyway").click.wait()
            self.d(resourceId="com.google.android.setupwizard:id/first_name_edit").click()
            self.d(resourceId="com.google.android.setupwizard:id/first_name_edit").set_text(name)
            time.sleep(2)
            self.d.press.back()
            time.sleep(2)
            self.d(text="Next").click.wait()
            self.d(text="More").click.wait()
            self.d(text="Next").click.wait()
            time.sleep(2)
            if self.d(text="Finish").exists:
                self.d(text="Finish").click.wait()
            if self.d(text="Allow").exists:
                self.d(text="Allow").click.wait()
            #self.d.sleep()
            time.sleep(10)
            self.d.press.power()
            time.sleep(10)
            self.d.wakeup()
            #time.sleep(5)
            g_common_obj2.unlock()
        for i in range(5):
            if self._locator.ok_btn.exists:
                self._locator.ok_btn.click.wait()
                time.sleep(2)
        if not self.d(text="GOT IT").exists:
            time.sleep(2)
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        assert self._locator.home_flag.exists
