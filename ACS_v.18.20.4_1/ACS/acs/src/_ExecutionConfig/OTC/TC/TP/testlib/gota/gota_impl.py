#-*- coding: utf-8 -*-
#Copyright (C) 2014  Lan, SamX <samx.lan@intel.com>;Xu,yuhui <yuhuix, xu@intel.com>
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

import commands
import os
import subprocess
import time
import string
import cmd
import sys
import random
from nose.tools import assert_equals
from macpath import realpath
from tests.IRDA_OEM_Customization.init.tools import ConfigHandle
from uiautomator import Device
from testlib.clock.clock import Clock
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.audio.audio_impl import AudioImpl
from testlib.util.repo import Artifactory
from testlib.system.system_impl import SystemImpl as fileop
from testlib.util.process import shell_command,killall
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.audio.audio_log import AudioLogger
from testlib.AfW.api_impl import ApiImpl
from testlib.AfW.entity import Remote
from testlib.util.device import TestDevice
from testlib.util.config import TestConfig
from testlib.util.uiatestbase import UIATestBase
from multiprocessing import Process
from multiprocessing import Pipe
from multiprocessing import Value
from atk import Text
from twisted.test.test_tcp_internals import resource
from twisted.python.rebuild import latestClass
from time import sleep
from pydoc import describe

class gotaImpl(UIATestBase):
    """
    Implements System android framework actions.
    """
    gota_record="/tmp/gota/test.log"
    clock=Clock()
    def __init__ (self, cfg = None):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        self.config = TestConfig()
        self.api = ApiImpl()
        self.remote = Remote()
        self.child_pid = 0
        self.cfg=cfg
        self.serial=g_common_obj2.getSerialNumber()
        self.locator=ApkInstallUninstallImpl.Locator(self.d)
        self.fileop=fileop(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
        self.logger = AudioLogger.getLogger()
        self.audio = AudioImpl(self.cfg)
    """
    Android_Framework
    -------------------------------------------------------------------
    """
    def getprop_grep(self,keywords):
        cmdstr="adb -s %s shell getprop | grep  %s" %self.serial %keywords
        getprop_grep=os.popen(cmdstr).read()
        print  "%s is %s"%(keywords,getprop_grep)
        return getprop_grep

    def check_notification(self,para):
        print 'checking  if %s is found in notification'%para
        if not self.d(resourceId="com.android.systemui:id/notification_stack_scroller").exists:
            print 'open notification'
            self.d.open.notification()
        time.sleep(4)
        res=self.d(textContains=para).exists
        print "check result is %s"%res
#         assert self.d(text=para).exists,'[error] %s is not found in notification'%para
#         ' Error%s is found in notification'%para
        self.d.press.back()
        self.d.press.back()
        return res

    def del_all_alarm(self):
        print "delete all alarm"
        self.clock.launchAlarm()
        alarm_number=self.d(resourceId="com.android.deskclock:id/arrow").count
        i=0
        while not self.d(text="No Alarms").exists:
            print 'delete start'
            alarm_number=self.d(resourceId="com.android.deskclock:id/arrow").count
            print 'Alarm count is %s'%alarm_number
            self.d.click(530,227)
            self.d(resourceId="com.android.deskclock:id/delete").click.wait()
            i +=1
            if i>30:
                break
        if self.d(resourceId="com.android.deskclock:id/arrow").exists:
            print "no alarm is found"
            return True
        else :
            print 'alarm is not found'

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

    def del_all_usrs(self):
        i=1
        while self.d(resourceId = "com.android.settings:id/trash_user").exists:
            print "delete user: %s"%i
            self.d(resourceId = "com.android.settings:id/trash_user").click.wait()
            self.d(text = "Delete").click.wait()
            i=i+1
            if i>10:
                break
    """
    Add an user and switch to it
    """
    def add_one_multi_user (self):
        self.launch_settings()
        self.d(text = "Users").click.wait()
        time.sleep(3)
        self.del_all_usrs()
        if self.d(text = "Add user or profile").exists:
            time.sleep(2)
            #self.d.click(100,200)
            self.d(text="Owner").click.wait()
            self.d(index="1", resourceId="com.android.settings:id/user_name").set_text('Owner')
            self.d(text = "OK").click.wait()
            self.d(text = "Add user or profile").click.wait()
            self.d(text = "User").click.wait()
            self.d(text = "OK").click.wait()
            self.d(text = "Set up now").click.wait()
            print 'start sleep 15s'
            time.sleep(15)
            print 'end sleep 15s'
#             self.wake_up()
#         self.d(resourceId="com.google.android.setupwizard:id/start").click()
    """

    common
    -------------------------------------------------------------------
    """
    def reboot_device(self,time_out=90):
        """
        android root on
        """

        adbcmdstr = "adb reboot"
        ret = g_common_obj.shell_cmd(adbcmdstr, time_out)
        print "xxxxxxxxxxxxxxxxxxxxxxxxxxx   adb reboot :%s"%ret
        adbcmdstr = "adb wait-for-device"
        ret = g_common_obj.shell_cmd(adbcmdstr, time_out)
        return ret

    def get_pid(self,pid_name):
        cmd_get_pid = "adb shell ps |grep %s"%pid_name
        cmd_result = os.popen(cmd_get_pid).read().split()
        if len(cmd_result) == 0:
            self.child_pid = None
        else:
            self.child_pid = int(os.popen(cmd_get_pid).read().split()[1])
        print self.child_pid
        return self.child_pid

    def kill_pid(self,pid):
        cmd_kill_pid = "adb shell kill %s"%pid
        cmd_result = os.popen(cmd_kill_pid).read().split()
        print cmd_result

    def launch_settings(self):
        '''
        Launch Settings app.
        '''
        print "[Info] ---Launch Settings app."
        g_common_obj.launch_app_am("com.android.settings", ".Settings")

    def get_process_owner(self,pid_name):
        cmd_get_pid = "adb shell ps |grep %s"%pid_name
        cmd_result = os.popen(cmd_get_pid).read().split()
        if len(cmd_result) == 0:
            self.process_owner = None
        else:
            self.process_owner = os.popen(cmd_get_pid).read().split()[0]
        print self.process_owner
        return self.process_owner

    def make_device_to_low_memeory(self):
        return self.system_impl().make_device_to_low_memeory()

    def make_device_cpu_overload(self):
        print "This is to make cpu overload"
        cmdstr='adb shell "cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null&"'
        proc = subprocess.Popen(cmdstr,
                            shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
        
        print "proc is running"
        time.sleep(30)
        print "go to kill the proc "
        proc.kill()
        res =proc.poll()
        print "proc's status is : %s"%res
        return res
    def execute_no_end_cmd(self, command):
        child = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        time.sleep(20)
        child.kill()
        return child.stdout.readlines()
        '''
        make_device_cpu_overload_release():need to improve
        '''
    def make_device_cpu_overload_release(self):
        print "This is to make cpu overload release: kill the pid of overload"
        pid_name="/system/bin/sh"
        pid=0
        global res
        while pid:
            pid=self.get_pid(pid_name)
            print pid
            res=self.kill_pid(pid)
            print "kill pid result is ",res
        return res
        '''
        Manageability
        -------------------------------------------------------------------------------------------
        '''
    def launch_devtool(self):
        '''
        Launch Settings app.
        '''
        print "[Info] ---Launch Settings app."
        g_common_obj.launch_app_am("com.android.development", ".Development")
        for i in range(10):
            if self.d(text="Dev Tools").exists:
                break
            time.sleep(1)
        assert self.d(text="Dev Tools").exists

    def select_option_badbehavior(self, option):
        '''
        Set select option under BadBehavior
        '''
        print "[Info] ---Set select option under BadBehavior: select %s" % option

        if not self.d(text="Dev Tools").exists:
            self.launch_devtool()
        self.d(text = "Bad Behavior").click.wait()
        self.d(text = option).click.wait()
        if self.d(text="OK").exists:
            self.d(text="OK").click.wait()
#         elif self.d(text="Dev Tools").exists:
#             self.d(text="").click.wait()
    def count_anrfile(self):
        '''
        adb shell ls /data/anr
        '''
        cmdstr='adb shell ls /data/anr'
        check_res=os.popen(cmdstr).read().count('trace')
        print "the number of trace.txt is:",check_res
        return check_res

    def unverify_apps_over_USB(self):
        time.sleep(2)
        while 1:
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
            time.sleep(1)
            if self.d(text="Security").exists:
                break
        time.sleep(1)
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        time.sleep(1)
        self.d(textContains="Developer options").click.wait()
        time.sleep(1)
        self.d(scrollable=True).scroll.vert.to(textContains="Verify apps over USB")
        time.sleep(1)
        self.d(textContains="Verify apps over USB").click.wait()
        self.d.press.home()

    def check_exceptionlog(self):
        '''
        clear (flush) the entire log and exit
        Force finishing activity com.android.development/.BadBehaviorActivity
        '''
        cmdstr='adb shell logcat -d | grep com.android.dev'
        res=os.popen(cmdstr).read()
#         print res
        count=res.count('Force finishing activity')
        assert count>0 , "[Error]exception is not found in logcat"
        print '[info] exception is found in logcat!'

    def install_systemdomain_app(self, apkfolder, apkname):
        """
            @summary: Install apps whose apk under one folder
            @param apkfolder: folder placed apks install file of apps
            @param apkfilter: filter to choose apk,
                if filter turn True, install the apk, otherwise skip install
            @return: install successed apps
        """
        print "[INFO] Install Apps under folder: %s" % apkfolder
        suc_cnt = 0
        fail_cnt = 0
        apk_path = os.path.join(apkfolder, apkname)
        install_cmd = 'adb install -r "%s"' % apk_path
        pipe = os.popen(install_cmd).read()
        if "Success" in pipe:
            suc_cnt += 1
            print "[INFO] Install App success: %s" % apkname
        else:
            fail_cnt += 1
            print "[ERROR] Install App fail: %s" % apkname
        print "[INFO] Try to install %d apps, Success: %d, Fail: %d" % (suc_cnt+fail_cnt, suc_cnt, fail_cnt)



    def check_fastboot_version(self, version, sn=""):
        if sn=="":
            sn=g_common_obj2.getSerialNumber()
        cmd="adb -s %s reboot bootloader" %sn
        result1 =os.popen(cmd)
        time.sleep(20)
        cmd2="fastboot -s %s getvar all 2>&1" %sn
        result2=os.popen(cmd2).read()
        print "hahahhaha %s" %result2
        assert result2.count(version)!=0, "[ERROR]: The fastboot version %s is not correct" %version
        print "[INFO]: The fastboot version %s is correct" %version
        self.reboot_from_fastboot_to_mos(sn)

    def search_user_apps(self,app):
        """
            @summary: Get user installed apps
        """
        cmd = 'pm list packages -3; echo $?'
        apps = g_common_obj.adb_cmd_capture_msg(cmd).count(app)
        assert apps>0 , "[Error]The installed app is not found"
        print '[info] The installed app is found'

    def dismiss_alarm(self):
        start = time.time()
        print "Wait for alarm..."
        while time.time() - start<130:
            if self.d(resourceId="com.android.deskclock:id/alarm").exists:
                print 'swipe to dismiss alarm'
                return self.d.swipe(380,822,624,822)

    def snooze_alarm(self):
        start = time.time()
        print "Wait for alarm..."
        while time.time() - start<130:
            if self.d(resourceId="com.android.deskclock:id/alarm").exists:
                print 'swipe to snooze alarm'
                return self.d.swipe(380,822,148,822)
        time.sleep(10)

    def set_screen_lock(self, option, curpasswd="abcd", newpasswd="abcd"):
        '''
        Set screen lock.
        '''
        print "[Info] ---Set screen lock %s." % option
        if not self.d(text="Screen lock").exists:
            self.d(text = "Security").click.wait()
        self.d(text = "Screen lock").click.wait()
        if self.d(resourceId = "com.android.settings:id/password_entry").exists:
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(curpasswd)
            self.d(text = "Continue").click.wait()
        self.d(text = option).click.wait()
        if option == "Password":
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(newpasswd)
            self.d(text = "Continue").click.wait()
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(newpasswd)
            self.d(text = "OK").click.wait()
            self.d(text = "Done").click.wait()
        assert self.d(text="Screen lock").down(text=option) != None
        self.d.press.back()

    def developr_option_check(self, option_name, enable):
        '''
        check/uncheck options in "Developer options"

        Parameters:
            option_name: name of options in UI
            enable: true -- check, false -- uncheck option
        '''
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings"% self.serial)
        self.d(scrollable=True).scroll.toEnd()  # it's at the end
        if not self.d(textContains="Developer options").exists:
            self.enable_developer_option()
        self.d(textContains="Developer options").click.wait()
        self.d(scrollable=True).scroll.vert.to(textContains=option_name)
        checkbox = self.d(textContains=option_name).right(
            resourceId="android:id/checkbox")
        if checkbox.checked != enable:
            checkbox.click()
        self.d.press.back()  # back to Settings
        self.d.press.back()  # exit Settings Activity

    def enable_developer_option(self):
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(scrollable=True).scroll.vert.to(textContains="About tablet")
        if self.d(textContains="Developer options").exists:
            return
        self.d(textContains="About tablet").click.wait()
        for i in range(8):
            self.d(textContains="Build number").click.wait()
        self.d.press.back()
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        assert self.d(textContains="Developer options").exists

    def restart_jsonrpc_server(self):
    #try to start jsonrpc server
        try:
            d = Device(self.serial)
            d.info
        except Exception as e:
            print "start json rpc server failed"

    def push_uiautomator_jar(self):
    # put bundle.jar and uiautomator-stub.jar to the right place
        if os.path.exists("/usr/lib/python2.7/dist-packages/uiautomator/libs"):
            jar_path="/usr/lib/python2.7/dist-packages/uiautomator/libs"
        else:
            jar_path="/usr/local/lib/python2.7/dist-packages/uiautomator/libs"
        bundle_path = jar_path + "/bundle.jar"
        uiautomator_path = jar_path + "/uiautomator-stub.jar"
        push_bundle_jar = "adb -s " + self.serial + " push " + bundle_path + " /data/local/tmp"
        push_uiautomator_jar = "adb -s " + self.serial + " push " + uiautomator_path + " /data/local/tmp"
        for i in range(20):
            os.system("adb -s %s root > /dev/null 2>&1" % self.serial)
#             os.system(push_bundle_jar)
#             os.system(push_uiautomator_jar)
            self.restart_jsonrpc_server()
            time.sleep(10)
            dd = os.system("adb -s %s shell ps |grep uiautomator" % self.serial)
            if dd == 0:
                break
            else:
                print "push jar start ========"
                os.system(push_bundle_jar)
                os.system(push_uiautomator_jar)
                print "push jar end ========="
                time.sleep(5)
        else:
            raise Exception('RPC server start failed')

    def wifi_connect_after_gota(self):
        time.sleep(2)
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        time.sleep(2)
        self.d(text="Wi‑Fi").click.wait()
        time.sleep(5)
        if self.d(text="Connected").exists:
            print "wifi Connected"
        else:
            assert False,"wifi not Connected"


    def connect_AP(self, ssid, password):
        if os.system("adb -s %s shell dumpsys connectivity | grep 'CONNECTED/CONNECTED.*%s' > /dev/null 2>&1" % (self.serial, ssid)) == 0:
            print "Already connect wifi"
            return
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(text="Wi‑Fi").click.wait()
        if not self.d(className="android.widget.Switch").checked:
            self.d(className="android.widget.Switch").click.wait()
            time.sleep(3)
        self.d.press.menu()
        self.d(text="Add network").click.wait()
        self.d(text="Enter the SSID").set_text(ssid)
        self.d(resourceId="com.android.settings:id/security", index=1).click.wait()
        self.d(text="WPA/WPA2 PSK").click.wait()
        self.d(resourceId="com.android.settings:id/password").set_text(password)
        self.d(text="Save").click.wait()
        time.sleep(1)
        for j in range(100):
            if self.d(text="Connected").exists:
                print "connect wifi pass..."
                break
            else:
                if self.d(text=ssid).exists==True:
                    self.d(text=ssid).click.wait()
                    if self.d(resourceId="com.android.settings:id/password").exists:
                        self.d(resourceId="com.android.settings:id/password").set_text(password)
                    if self.d(text="Connect").exists:
                        self.d(text="Connect").click.wait()
                    else:
                        self.d.press.back()
                    time.sleep(10)
                else:
                    time.sleep(10)
        assert self.d(text=ssid).down(text="Connected").exists, "Connect AP failed"


    def turn_off_wifi(self):
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(textMatches="Wi.*Fi").click.wait()
        if self.d(text="On", resourceId="com.android.settings:id/switch_text").exists:
            self.d(text="On", resourceId="com.android.settings:id/switch_text").click()
        time.sleep(3)
        assert self.d(text="Off", resourceId="com.android.settings:id/switch_text").exists, "[ERROR]: Turn-off wifi failed"

    def add_google_account(self, username, password):
    # add google account
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        time.sleep(2)
        self.d(text="Accounts").click.wait()
        if self.d(text="Google").exists:
            self.d.press.menu()
            if self.d(resourceId="android:id/checkbox").checked:
                self.d(resourceId="android:id/checkbox").click.wait()
                self.d(text="OK").click.wait()
            else:
                self.d.press.back()
            return
        self.d(resourceId="android:id/icon").click.wait()
        self.d(text="Google").click.wait()
        for i in range(20):
            if self.d(description="Add your account").exists:
                break
            assert not self.d(text="Couldn't sign in").exists
            time.sleep(3)
        assert self.d(description="Add your account").exists
        time.sleep(3)
        y= self.d.displayHeight
        x= self.d.displayWidth
        self.d.click(200, y/1.77454545)
        self.d.click(200, y/1.77454545)
        #self.d.click(200, 710)
        os.system("adb -s %s shell input text '%s'" % (self.serial, username))
        time.sleep(1)
        self.d.click(x/1.08108108, y/1.28)
        time.sleep(10)
        self.d.click(x/4.44444444, y/2.16888889)
        os.system("adb -s %s shell input text '%s'" % (self.serial, password))
        time.sleep(1)
        self.d.click(x/1.08108108, y/1.28)
        for i in range(50):
            if self.d(textStartsWith="Back up your phone").exists:
                break
            assert not self.d(text="Couldn't sign in").exists
            time.sleep(3)
        assert self.d(textStartsWith="Back up your phone").exists
        self.d(text="Next").click.wait()
        if self.d(text="SKIP").exists:
            self.d(text="SKIP").click.wait()
        time.sleep(5)
        if self.d(text="Remind me later").exists:
            self.d(text="Remind me later").click.wait()
            self.d(text="Next").click.wait()
        time.sleep(5)
        self.d.press.menu()
        if self.d(resourceId="android:id/checkbox").checked:
            self.d(resourceId="android:id/checkbox").click.wait()
            self.d(text="OK").click.wait()
        os.system("adb -s %s shell am force-stop com.android.settings" % self.serial)

    def check_google_account_exist(self, account):
        #launch systm update to do ota update
        #package_name="com.android.settings"
        #activity_name="com.android.settings.Settings"
        #g_common_obj.launch_app_am(package_name, activity_name)
        self.unlock_screen()
        time.sleep(1)
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        time.sleep(1)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "Accounts",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        #self.d(text="Accounts").click()
        assert self.d(text="Google").exists, "[ERROR]: No google account saved"
        self.d(text="Google").click()
        assert self.d(text=account).exists, "[ERROR]: Google account: %s erased" %account
        print "[INFO]: Google account %s is not erased" %account

    def close_lock_screen(self):
    # delete clock screen
        time.sleep(2)
        while 1:
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
            time.sleep(1)
            if self.d(text="Security").exists:
                self.d(text="Security").click.wait()
                break
        time.sleep(2)
        self.d(text="Screen lock").click()
        time.sleep(2)
        self.d(text="None").click()
        assert self.d(text="Screen lock").down(text="None") != None

    def accept_unknow_resource(self):
    # delete clock screen
        time.sleep(2)
        while 1:
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
            time.sleep(1)
            if self.d(text="Security").exists:
                self.d(text="Security").click.wait()
                break
        time.sleep(3)
        self.d(text="Unknown sources").click.wait()
        time.sleep(1)
        self.d(text="OK").click()
        assert self.d(resourceId="android:id/switchWidget", text="ON", instance=1).exists
        self.d.press.home()

    def keep_awake(self):
    # keep screen awake
        time.sleep(2)
        while 1:
            os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
            time.sleep(1)
            if self.d(text="Security").exists:
                break
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        self.d(textContains="Developer options").click.wait()
#         if not self.d(text="Stay awake").right(resourceId="android:id/checkbox").checked:
#             self.d(text="Stay awake").right(resourceId="android:id/checkbox").click()
        time.sleep(1)
        self.d(text="Stay awake").click.wait()
        time.sleep(1)
        self.d.press.back()
        self.d(scrollable=True).scroll.vert.to(text="Display")
        self.d(text="Display").click.wait()
        time.sleep(1)
        if not self.d(text="After 30 minutes of inactivity").exists:
            self.d(text="Sleep").click.wait()
            time.sleep(1)
            self.d(text="30 minutes").click.wait()
        assert self.d(text="After 30 minutes of inactivity").exists

    def file_download(self, file_website, file_path="/tmp/resource_ota"):
        #verify the website
        if file_website is None:
            print("ERROR: The website is None!")
            raise
        u=self.cfg.get("download_user")
        p=self.cfg.get("download_password")
        cmd = "mkdir %s;cd %s;wget --http-user=%s --http-passwd=%s --no-check-certificate -c %s"\
         % (file_path, file_path, u, p, file_website)
        pipe = os.system(cmd)
        if pipe != 0:
            print("ERROR:Download file error!\
                Please check the website [%s]" % file_website)
            return False
        print("SUCCESS:Download finish!")
        return True

    def phone_flash_tool_build_zip_file(self,filename, auto_clean_download="no"):
        #phone flash tool the device
        sn=g_common_obj2.getSerialNumber()
        cm="adb -s %s shell getprop ro.product.device" %sn
        device=os.popen(cm).read().strip()
        print device

        if device=="st70408_4_coho":
            filename=self.cfg.get("base_build_trekstor")
        elif device=="one7_0_4_coho":
            filename=self.cfg.get("base_build_one704")
        elif device=="one695_1_coho":
            filename=self.cfg.get("base_build_one695")
        elif device=="one8_0_1_coho":
            filename=self.cfg.get("base_build_one801")
        elif device=="ecs210a_0_coho":
            filename=self.cfg.get("base_build_ecs210a")
        #reboot to fastboot to fit all phone flash tool versions
        cc = "adb -s %s reboot bootloader" %sn
        print cc
        rr=os.popen(cc).read()
        print rr
        time.sleep(10)
        #download the flash file from website and flash it
        if filename.count("http")!=0:
            u=self.cfg.get("download_user")
            p=self.cfg.get("download_password")
            cmd="phoneflashtool --cli --os-sn %s --artifact --auth %s:%s --timeout 120 --real-time-download --uri %s --flash-filename flash.xml" %(sn, u,p,filename)
        #pft the flash file in local dir
        else:
            cmd="phoneflashtool --cli --os-sn %s --flash-file %s --timeout 120 --real-time-download --flash-filename flash.xml" % (sn, filename)
        print "[INFO]: %s" %cmd
        result=os.popen(cmd).read()
        print result
        assert result.count("Flash success")==1, "[ERROR]: phone flash tool failed for %s" % filename
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("com.google.android.setupwizard/com.google.android.setupwizard.user.WelcomeActivity")==0:
            if  os.popen(cmd3).read().count("StatusBar")!=0:
                self.wake_up()
                break
            print "[INFO]: wait for android os..."
            time.sleep(10)
        '''
        cmd2="adb -s %s get-state" %sn
        while os.popen(cmd2).read().count("device")==0:
            time.sleep(10)
        print "[INFO]: adb -s %s get-state: %s" %(sn, os.popen(cmd2).read())
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn

        while os.popen(cmd3).read().count("mCurrentFocus=null")!=0 or os.popen(cmd3).read()=="":
            print "[INFO]: wait for android os..."
            time.sleep(10)
        '''
        time.sleep(5)

        print "[INFO]: Phone flash tool successfully for %s" % filename
        # adb root after the phone flash successfully
        cmd4="adb -s %s root" %sn
        result4 = os.popen(cmd4).read()
        print "[INFO]: adb root for %s %s" %(sn,result4)
        #if auto_clean_download is read as "yes, delete the the download pft package
        if self.cfg.get("auto_clean_download")=="yes":
            cmd5="phoneflashtool --cli  --purge-cache 0"
            result5 = os.popen(cmd5).read()
            print "[INFO]: %s" %result5
            assert result5.count("Purge done")!=0, "[ERROR]: Purge cache failed"
        time.sleep(3)
    
    def phone_flash_tool_build_zip_file2(self,filename, base_seq, auto_clean_download="no"):
        #phone flash tool the device
        sn=g_common_obj2.getSerialNumber()
        cm="adb -s %s shell getprop |grep finger" %sn
        if os.popen(cm).read().count("trekstor")!=0:
            filename=self.cfg.get("base_build_ecs"+"_"+base_seq)
        elif os.popen(cm).read().count("ecs27b")!=0:
            filename=self.cfg.get("base_build_ecs27b"+"_"+base_seq)
        elif os.popen(cm).read().count("ecs28a")!=0:
            filename=self.cfg.get("base_build_ecs28a"+"_"+base_seq)
        elif os.popen(cm).read().count("cloudfone/cloudpad/one695_1_coho")!=0:
            filename=self.cfg.get("base_build_one695_1_coho"+"_"+base_seq)
        elif os.popen(cm).read().count("cloudfone/cloudpad/one8_0_1_coho")!=0:
            filename=self.cfg.get("base_build_one8_0_1_coho"+"_"+base_seq)
        #reboot to fastboot to fit all phone flash tool versions
        cc = "adb -s %s reboot bootloader" %sn
        print cc
        rr=os.popen(cc).read()
        print rr
        time.sleep(10)
        #download the flash file from website and flash it
        if filename.count("http")!=0:
            u=self.cfg.get("download_user")
            p=self.cfg.get("download_password")
            cmd="phoneflashtool --cli --os-sn %s --artifact --auth %s:%s --timeout 120 --real-time-download --uri %s --flash-filename flash.xml" %(sn, u,p,filename)
        #pft the flash file in local dir
        else:
            cmd="phoneflashtool --cli --os-sn %s --flash-file %s --timeout 120 --real-time-download --flash-filename flash.xml" % (sn, filename)
        print "[INFO]: %s" %cmd
        result=os.popen(cmd).read()
        print result
        assert result.count("Flash success")==1, "[ERROR]: phone flash tool failed for %s" % filename
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("com.google.android.setupwizard/com.google.android.setupwizard.user.WelcomeActivity")==0:
            print "[INFO]: wait for android os..."
            time.sleep(10)
        '''
        cmd2="adb -s %s get-state" %sn
        while os.popen(cmd2).read().count("device")==0:
            time.sleep(10)
        print "[INFO]: adb -s %s get-state: %s" %(sn, os.popen(cmd2).read())
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn

        while os.popen(cmd3).read().count("mCurrentFocus=null")!=0 or os.popen(cmd3).read()=="":
            print "[INFO]: wait for android os..."
            time.sleep(10)
        '''
        time.sleep(5)

        print "[INFO]: Phone flash tool successfully for %s" % filename
        # adb root after the phone flash successfully
        cmd4="adb -s %s root" %sn
        result4 = os.popen(cmd4).read()
        print "[INFO]: adb root for %s %s" %(sn,result4)
        #if auto_clean_download is read as "yes, delete the the download pft package
        if self.cfg.get("auto_clean_download")=="yes":
            cmd5="phoneflashtool --cli  --purge-cache 0"
            result5 = os.popen(cmd5).read()
            print "[INFO]: %s" %result5
            assert result5.count("Purge done")!=0, "[ERROR]: Purge cache failed"
        
    def phone_flash_tool_build_zip_file_3(self,filename, auto_clean_download="no"):
        sn=g_common_obj2.getSerialNumber()
        self.download_gota_content()
        OTA_path = self.cfg.get("ota_path")
        print "OTA_path=%s"%OTA_path
        cm="adb -s %s shell getprop |grep finger" %sn
        if os.popen(cm).read().count("trekstor/surftab/st70408_4_coho")!=0:
            filename=self.cfg.get("base_build_trekstor")
            dut_name = "Trekstor"
            configuration = "blank_TREKSTOR"
            
        elif os.popen(cm).read().count("cloudfone/cloudpad/one7_0_4_coho")!=0:
            filename=self.cfg.get("base_build_one704")
            dut_name = "One704"
        elif os.popen(cm).read().count("ecs27b")!=0:
            filename=self.cfg.get("base_build_ecs27b")
            dut_name = "ecs27b"
            
        elif os.popen(cm).read().count("ecs28a")!=0:
            filename=self.cfg.get("base_build_ecs28a")
            dut_name = "ecs28a"
        elif os.popen(cm).read().count("cloudfone/cloudpad/one695_1_coho")!=0:
            filename=self.cfg.get("base_build_one695")
            dut_name = "one695"
            #zipname=self.cfg.get("base_image_one695")
        elif os.popen(cm).read().count("cloudfone/cloudpad/one8_0_1_coho")!=0:
            filename=self.cfg.get("base_build_one801")
            dut_name = "one801"
        elif os.popen(cm).read().count("ecs/ecs210a/ecs210a_0_coho")!=0:
            filename=self.cfg.get("base_build_ecs210a")
            dut_name = "ecs210a"
        print filename
        image_str = filename.split(";")
        number=len(image_str)
        print number
        data=random.randint(0,12)
        print data
        Num = data
        if OTA_path == "Latest":
            base_image_link=image_str[0]
        elif OTA_path == "oldest":
            base_image_link=image_str[number-1]
        elif OTA_path == "Random":
            base_image_link=image_str[Num]
        print image_str
        print base_image_link
        file_str = base_image_link.split("/")
        print file_str
        file_num = len(file_str)
        print file_num
        print file_str[file_num-1]
        build_str = file_str[file_num-1].split(".")
        print build_str[0]
        print file_str
        base_directory="/tmp/gota/image/%s/%s/" %(dut_name,build_str[0])
        print base_directory
        print filename
        local_file="%s%s" %(base_directory,file_str[file_num-1])
        print local_file
        if not self.fileop.file_exists("/tmp/gota/image", "HOST"):
            os.system("mkdir /tmp/gota/image")
        if not self.fileop.file_exists(local_file, "HOST"):
            os.system("cd /tmp/gota/image/ecs27b/")
            cmd = "wget --no-check-certificate -c -P %s --user=sys_cti --password=uwlmo99@ %s" %(base_directory,base_image_link)
            print cmd
            wgetresult=os.system(cmd)
            print wgetresult
            print "haley"
            cmd2="unzip -n %s -d %s" %(local_file,base_directory)
            print cmd2
            os.system(cmd2)
            os.system("cp /tmp/gota/userdata.img %s" %base_directory)
            os.system("cp /tmp/gota/update_automation.xml %s" %base_directory)
        cmd="phoneflashtool --cli --os-sn %s -f %s/flash.json" %(sn,base_directory)
        print cmd
        result=os.popen(cmd).read()
        print result
        assert result.count("Flash success")==1, "[ERROR]: phone flash tool failed for %s" % filename
        self.wait_for_android_os()
        '''
        cmd2="adb -s %s get-state" %sn
        while os.popen(cmd2).read().count("device")==0:
            time.sleep(10)
        print "[INFO]: adb -s %s get-state: %s" %(sn, os.popen(cmd2).read())
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn

        while os.popen(cmd3).read().count("mCurrentFocus=null")!=0 or os.popen(cmd3).read()=="":
            print "[INFO]: wait for android os..."
            time.sleep(10)
        '''
        time.sleep(5)

        print "[INFO]: Phone flash tool successfully for %s" % filename
        # adb root after the phone flash successfully
        cmd4="adb -s %s root" %sn
        result4 = os.popen(cmd4).read()
        print "[INFO]: adb root for %s %s" %(sn,result4)
        time.sleep(3)
    
    def skip_initial_screen_after_flash(self):
        # setup guideline
        """
        Skip initial screen after factory reset.
        """
        self.d = g_common_obj.get_device()
        self.d.press.home()
        Imagetype = g_common_obj2.getAndroidVersion()
        time.sleep(2)
        if self.d(description="Apps").exists and not self.d(text="GOT IT").exists:
            return
        for i in range(10):
            self.d.press.back()
            if self.d(description="Comenzar").exists:
                self.d(scrollable=True).scroll.vert.to(textContains="English (United States)")
            if self.d(description="Start").exists or self.d(description="Iniciar").exists:
                break
        if self.d(description="Start").exists or self.d(description="Iniciar").exists:
            if self.d(description="Start").exists:
                self.d(description="Start").click.wait()
            else:
                self.d(description="Iniciar").click.wait()
            self.d(text="Skip").click.wait()
            time.sleep(3)
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
            time.sleep(3)
            self.d(text="Skip anyway").click.wait()
            time.sleep(3)
            if self.d(text="Date & time").exists:
                if self.d(text="More").exists:
                    self.d(text="More").click.wait()
                    time.sleep(3)
                if self.d(text="Next").exists:
                    self.d(text="Next").click.wait()
            time.sleep(3)
            self.d(text="First").click.wait()
            time.sleep(3)
            os.system("adb -s %s shell input text 'First'" % self.serial)
            time.sleep(2)
            self.d.press.back()
            time.sleep(2)
            while self.d(text="More").exists:
                self.d(text="More").click.wait()
            time.sleep(5)
            self.d(text="Next").click.wait()
            if (Imagetype == "M"):
                if self.d(text="Protect this device and require a PIN, pattern, or password to unlock the screen").exists:
                    if self.d(text="Protect this device and require a PIN, pattern, or password to unlock the screen").checked:
                        self.d(text="Protect this device and require a PIN, pattern, or password to unlock the screen").click.wait()
            time.sleep(3)
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
                time.sleep(5)
                self.d(text="Skip anyway").click.wait()
            time.sleep(5)
            while self.d(text="More").exists:
                self.d(text="More").click.wait()
                time.sleep(3)
            if (Imagetype == "M"):
                time.sleep(3)
                if self.d(description="More").exists:
                    self.d(description="More").click.wait()
            time.sleep(5)
            self.d(text="Next").click()
            time.sleep(5)
            if self.d(text="Finish").exists:
                self.d(text="Finish").click.wait()
        time.sleep(5)
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()
        time.sleep(5)
        while self.d(text="OK").exists:
            time.sleep(3)
            self.d(text="OK").click.wait()
        time.sleep(5)
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        assert self.d(description="Apps").exists and not self.d(text="GOT IT").exists

    def check_encryption_status(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        time.sleep(2)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "Security",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        if self.d(text="Encrypt tablet").sibling(text="Encrypted").exists:
            return "True"
        else:
            return "False"
        print "[INFO]: The DUT is encrypt status is encrypted"

    def check_bios_version(self,bios_version):
        base_BIOS=self.get_info("BIOS before GOTA")
        new_BIOS=self.get_info("BIOS after GOTA")
        if bios_version == "none":
            assert base_BIOS == new_BIOS
        else:
            assert new_BIOS == bios_version
        print "[INFO]:new_BIOS is %s " % new_BIOS
        return

    def check_fingerprint(self,fingerprint):
        fplist=fingerprint.split('/')
        assert len(fplist)==6
        for element in fplist:
            print "element %s" % element

        cmd="adb -s %s shell getprop ro.product.brand" % self.serial
        BRAND=os.popen(cmd).read()
        templist=BRAND.split()
        assert not cmp(templist[0],fplist[0])

        cmd="adb -s %s shell getprop ro.product.name" % self.serial
        NAME=os.popen(cmd).read()
        NAME=NAME.replace("\n","",1)
        templist=NAME.split()
        assert not cmp(templist[0],fplist[1])

        cmd="adb -s %s shell getprop ro.product.device" % self.serial
        device=os.popen(cmd).read()
        flisttemp=fplist[2].split(':')
        temp=flisttemp[0]
        templist=device.split()
        assert not cmp(templist[0],temp)

        cmd="adb -s %s shell getprop ro.build.id" % self.serial
        ID=os.popen(cmd).read()
        ID=ID.replace("\n","",1)
        templist=ID.split()
        assert not cmp(templist[0],fplist[3])

        cmd="adb -s %s shell getprop ro.build.version.incremental" % self.serial
        INCREMENTAL=os.popen(cmd).read()
        flisttemp=fplist[4].split(':')
        temp=flisttemp[0]
        INCREMENTAL=INCREMENTAL.replace("\n","",1)
        templist=INCREMENTAL.split()
        assert not cmp(templist[0],temp)

        cmd="adb -s %s shell getprop ro.build.tags" % self.serial
        TAG=os.popen(cmd).read()
        TAG=TAG.replace("\n","",1)
        templist=TAG.split()
        temp=fplist[5].split()
        assert not cmp(templist[0],temp[0])
        return

    def set_timezone(self, timezone):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "Date & time",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        self.d(text="Select time zone").click.wait()
        self.d(className="android.widget.ListView", resourceId="android:id/list") \
            .child_by_text(
            timezone,
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()

    def check_time(self):
        before=self.get_info("OStime before GOTA")
        after=self.get_info("OStime after GOTA")
        ibefore=string.atol(before)
        iafter=string.atol(after)
        cost=iafter-ibefore
        print "%s" %cost
        assert cost < 10800
        return

    def ota_update(self,script, ota_package, timeout=900):
        sn=g_common_obj2.getSerialNumber()
        cmd="%s %s" %(script, ota_package)
        count=0
        print "[INFO]: cmd is: %s" %cmd
        result=os.popen(cmd).read()
        print result
        time.sleep(10)
        count=count+10
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("com.google.android.googlequicksearchbox/com.google.android.launcher.GEL")==0:
            print "[INFO]: wait for android os..."
            time.sleep(10)
            count=count+10
            assert count<timeout, "[ERROR]: Upgrading exceeds limit %d second" %timeout
        time.sleep(10)
        count=count+10
        cmd4="adb -s %s root" %sn
        result = os.popen(cmd4).read()
        print "[INFO]: %s" %result
        assert result.find("error")==-1, "[ERROR]: adb root failed"
        print "[INFO]: adb root successfully"
        print "[INFO]: android upgrades successfully"
    def ota_through_flash_update_script(self, script, filename, timeout=1800):
        #ota the package in website
        if filename.count("http")!=0:
            file_path="/tmp/resource_ota/"
            assert self.file_download(filename, file_path), "[Error]: Download file failed"
            filename=file_path+(filename.split("/")[-1])
            self.ota_update(script,filename)
        #ota the package in local directory
        else:
            self.ota_update(script,filename)
        if self.cfg.get("auto_clean_download")=="yes":
            cmd="rm %s" %filename
            result=os.popen(cmd).read()
            print "[INFO]: %s" %result
            assert result=="", "[ERROR]:remove %s failed" %filename
    def unlock_oem(self,sn=""):
        if sn=="":
            sn=g_common_obj2.getSerialNumber()
        print "hahahahah %s" %sn
        cmd="fastboot -s %s getvar device-state 2>&1" %sn
        result1 =os.popen(cmd).read()
        print result1
        if result1.count("unlocked")==1:
            cmd2="fastboot -s %s oem lock" %sn
            result2=os.popen(cmd2).read()

        cmd3="fastboot -s %s oem unlock oem" %sn
        result3 =os.popen(cmd3).read()
        time.sleep(3)
        result4 =os.popen(cmd).read()
        print result4
        assert result4.count("unlocked")==1, "[ERROR]: fastboot oem unlock failed"
        print "[INFO]: fastboot oem unlock successfully"


    def adb_shell_getprop_to_file(self):
        BIOS_after_GOTA = os.popen("adb -s %s shell getprop ro.bootloader" %self.serial).read()
        BIOS_after_GOTA = BIOS_after_GOTA.replace("\n","",1)
        self.insert_info("BIOS after GOTA",BIOS_after_GOTA)
        Fingerprint_after_GOTA = os.popen("adb -s %s shell getprop ro.build.fingerprint" %self.serial).read()
        Fingerprint_after_GOTA = Fingerprint_after_GOTA.replace("\n","",1)
        self.insert_info("Fingerprint after GOTA",Fingerprint_after_GOTA)
        OStime_after_GOTA = os.popen("adb -s %s shell date +%%s" %self.serial).read()
        OStime_after_GOTA = OStime_after_GOTA.replace("\n","",1)
        self.insert_info("OStime after GOTA",OStime_after_GOTA)
        self.update_info(self.get_filename())
        return

    def fastboot_getvar_all_to_file(self,filename,sn=""):
        if sn=="":
            sn=g_common_obj2.getSerialNumber()
        self.reboot_from_mos_to_fastboot(sn)
        dir=os.path.dirname(filename)
        fn=os.path.os.path.basename(filename)
        cmd="mkdir %s; touch %s; pwd" %(dir,filename)
        cmd1="echo 'fastboot -s %s getvar all is: \n' >> %s" %(sn,filename)
        cmd2="fastboot -s %s getvar all >> %s 2>&1" %(sn,filename)
        cmd3="echo '\n\n' >> %s" %filename
        #print cmd1
        #print cmd2
        result=os.popen(cmd)
        time.sleep(3)
        result1=os.popen(cmd1)
        time.sleep(3)
        result2=os.popen(cmd2)
        time.sleep(3)
        result3=os.popen(cmd3)
        time.sleep(3)
        self.reboot_from_fastboot_to_mos(sn)

    def reboot_from_mos_to_fastboot(self, sn="", timeout=30):
        if sn=="":
            sn=g_common_obj2.getSerialNumber()
        cmd="adb -s %s reboot bootloader" %sn
        result1 =os.popen(cmd)
        print "[INFO]: rebooting from mos to fastboot...."
        count=0
        while (count<=timeout):
            result2=os.popen("fastboot devices").read()
            if result2.find(sn)!=-1:
                break
            time.sleep(5)
            count=count+5
        assert result2.find(sn)!=-1, "[ERROR]: adb reboot %s MOS to fastboot failed." %sn
        time.sleep(5)
        print "[INFO]: reboot %s from mos to fastboot successfully" %sn

    def reboot_from_mos_to_mos(self, sn="", timeout=40):
        if sn=="":
            sn=g_common_obj2.getSerialNumber()
        cmd="adb -s %s reboot" %sn
        result1 =os.popen(cmd)
        print "[INFO]: rebooting from mos to mos...."
        time.sleep(10)
        count=0
        while (count<=timeout):
            if sn=="":
                sn=g_common_obj2.getSerialNumber()
            result2=os.popen("adb devices").read()
            if result2.find(sn)!=-1:
                break
            time.sleep(5)
            count=count+5
        assert result2.find(g_common_obj2.getSerialNumber())!=-1, "[ERROR]: adb reboot MOS to mos failed."
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("mCurrentFocus=null")!=0 or os.popen(cmd3).read()=="":
            print "[INFO]: wait for android os..."
            time.sleep(5)
        time.sleep(5)
        print "[INFO]: reboot -s %s from mos to mos successfully" %sn

    def reboot_from_fastboot_to_mos(self, sn="", timeout=30):
        if sn=="":
            cmd="fastboot reboot"
        else:
            cmd="fastboot -s %s reboot" %sn
        result1=os.popen(cmd)
        print "[INFO]: rebooting from fastboot to mos...."
        count=0
        while (count<=timeout):
            if sn=="":
                sn=g_common_obj2.getSerialNumber()
            result2=os.popen("adb devices").read()
            if result2.find(sn)!=-1:
                break
            time.sleep(5)
            count=count+5
        assert result2.find(g_common_obj2.getSerialNumber())!=-1, "[ERROR]: adb reboot MOS to fastboot failed."
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("mCurrentFocus=null")!=0 or os.popen(cmd3).read()=="":
            print "[INFO]: wait for android os..."
            time.sleep(5)
        time.sleep(5)
        cmd4="adb -s %s root" %sn
        result4=os.popen(cmd4)
        print "[INFO]: reboot -s %s from fastboot to mos successfully" %sn

    def reboot_from_fastboot_to_fastboot(self, sn="", timeout=30):
        if sn=="":
            cmd="fastboot reboot-bootloader"
        else:
            cmd="fastboot -s %s reboot-bootloader" %sn
        result1=os.popen(cmd)
        print "[INFO]: rebooting from fastboot to fastboot...."
        count=0
        time.sleep(3)
        count=count+3
        while (count<=timeout):
            result2=os.popen("fastboot devices").read()
            if result2.find(sn)!=-1:
                break
            time.sleep(5)
            count=count+5
        assert result2.find(sn)!=-1, "[ERROR]: adb reboot %s from fastboot to fastboot failed." %sn
        time.sleep(5)
        assert result2.find(g_common_obj2.getSerialNumber())!=-1, "[ERROR]: adb reboot fastboot to fastboot failed."
        print "[INFO]: reboot -s %s from fastboot to fastboot successfully" %sn

    def check_build_no(self, buildnum):
        #launch setting->about tablet to check base build no
        print "[INFO]:check for build number: %s" %buildnum
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        build_check=self.d(className="android.view.View", resourceId="android:id/decor_content_parent") \
            .child_by_text(
            "Build number",
            allow_scroll_search=True,
            className="android.widget.TextView") \
            .sibling(textContains=buildnum).exists
        assert build_check, "[INFO]: The current build is not %s" % buildnum
        print "[INFO]: The current build is %s" % buildnum

    def downloading_screen_exist_in_setting(self, target, timeout=10800):
        #launch systm update to do ota update
        '''
        package_name="com.google.android.gms"
        activity_name=".update.SystemUpdateActivity"
        print "[INFO]: Download OTA update package"
        g_common_obj.launch_app_am(package_name, activity_name)
        '''
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        self.d(text="System updates").click.wait()
        time_total=0
        time.sleep(3)
        time_total=time_total+3
        #check if there is ota update package
        counter=0
        while self.d(text="Your system is up to date.").exists:
            if counter==6:
                break
            self.d(text="Check now").click()
            time.sleep(10)
            time_total=time_total+10
            counter=counter+1
        assert self.d(text="Your system is up to date.").exists==False, \
            "[ERROR]: Your system is update to date. No OTA package %s found" % target
        #download ota update package
        if self.d(text="Download").exists:
            self.d(text="Download").click()
        time.sleep(3)
        assert self.d(textContains=target).exists, "[ERROR]: The target OTA package is %s" % target
        print "[INFO]: The target OTA update package is: %s" %target
        #if the system is in downloading or Retry download button
        counter=0
        while True:
            if counter==10:
                return False
            if self.d(text="Download").exists:
                self.d(text="Download").click()
                time.sleep(1)
            if self.d(textContains="Downloading").exists:
                return True
            counter=counter+1

    def downloading_screen_exist(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        self.d(text="System updates").click.wait()
        time_total=0
        time.sleep(3)
        time_total=time_total+3
        #check if there is ota update package
        counter=0
        while self.d(textContains = "Waiting to download").exists or self.d(textContains="Downloading").exists:
            if counter==3:
                break
            if self.d(text="Check now").exists:
                self.d(text="Check now").click()
            time.sleep(10)
            time_total=time_total+10
            counter=counter+1
        assert self.d(textContains = "Waiting to download").exists==True, "[ERROR]: Your system is not downloading"

    def check_anti_roll_back(self):
        status=self.update_gotastatus()
        print "status %s" % status
        assert status=="no GOTA package"
        return
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        self.d(text="System updates").click.wait()
        time_total=0
        time.sleep(3)
        time_total=time_total+3
        #check if there is ota update package
        counter=0
        while self.d(text="Your system is up to date.").exists:
            if counter==6:
                break
            if self.d(text="Check now").exists:
                self.d(text="Check now").click()
            elif self.d(text="Check for update").exists:
                self.d(text="Check for update").click()
            time.sleep(10)
            time_total=time_total+10
            counter=counter+1
        assert self.d(text="Your system is up to date.").exists==True, \
            "[ERROR]: Your system can be rollback update!!!!"

    def download_gota_package_and_auto_retry(self, target, timeout=10800):
        #launch systm update to do ota update
        '''
        package_name="com.google.android.gms"
        activity_name=".update.SystemUpdateActivity"
        print "[INFO]: Download OTA update package"
        g_common_obj.launch_app_am(package_name, activity_name)
        '''
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        self.d(text="System updates").click.wait()
        time_total=0
        time.sleep(3)
        time_total=time_total+3
        #check if there is ota update package
        counter=0
        while self.d(text="Your system is up to date.").exists:
            if counter==6:
                break
            if self.d(text="Check now").exists:
                self.d(text="Check now").click()
            elif self.d(text="Check for update").exists:
                self.d(text="Check for update").click()
            time.sleep(10)
            time_total=time_total+10
            counter=counter+1
        assert self.d(text="Your system is up to date.").exists==False, \
            "[ERROR]: Your system is update to date. No OTA package %s found" % target
        #download ota update package
        if self.d(text="Download").exists:
            self.d(text="Download").click()
        time.sleep(3)
        assert self.d(textContains=target).exists, "[ERROR]: The target OTA package is %s" % target
        print "[INFO]: The target OTA update package is: %s" %target
        #if the system is in downloading or Retry download button
        while self.d(textContains="Downloading").exists or self.d(textContains="Retry download").exists or self.d(textContains="Waiting to download").exists:
            if self.d(textContains="Retry download").exists:
                self.d(textContains="Retry download").click()
            print "[INFO]: Downloading OTA packge..."
            time.sleep(20)
            time_total=time_total+20
            assert time_total<timeout, "[ERROR]: Download time exceeds limit %d second" %timeout
        assert self.d(text="Restart & install").exists, "[ERROR]: Restart & install button does not exist"
        print "[INFO]: The OTA package %s is download successfully" % target
        time.sleep(5)

    def download_package_and_auto_retry(self, timeout=10800):
        '''
        package_name="com.google.android.gms"
        activity_name=".update.SystemUpdateActivity"
        print "[INFO]: Download OTA update package"
        g_common_obj.launch_app_am(package_name, activity_name)
        '''
        #launch systm update to do ota update
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()
        self.d(text="System updates").click.wait()
        time_total=0
        counter=0
        time.sleep(3)
        time_total=time_total+3
        #check if there is ota update package
        while self.d(text="Your system is up to date.").exists:
            if counter==6:
                break
            if self.d(text="Check now").exists:
                self.d(text="Check now").click()
            elif self.d(text="Check for update").exists:
                self.d(text="Check for update").click()
            time.sleep(10)
            time_total=time_total+10
            counter=counter+1
        assert self.d(text="Your system is up to date.").exists==False, \
            "[ERROR]: Your system is update to date. No OTA package found"
        #download ota update package
        if self.d(text="Download").exists:
            self.d(text="Download").click()
        time.sleep(3)
        #if the system is in downloading or Retry download button
        while self.d(textContains="Downloading").exists or self.d(textContains="Retry download").exists or self.d(textContains = "Waiting to download").exists:
            if self.d(textContains="Retry download").exists:
                self.d(textContains="Retry download").click()
            print "[INFO]: Downloading OTA package..."
            time.sleep(30)
            time_total=time_total+30
            assert time_total<timeout, "[ERROR]: Download time exceeds limit %d second" %timeout
        assert self.d(text="Restart & install").exists, "[ERROR]: Restart & install button does not exist"
        print "[INFO]: The OTA package is download successfully"
        time.sleep(5)

    def restart_and_install_gota_package(self, timeout=1800):

        self.unlock_screen()
        time.sleep(2)
        #run youtube for checking app's data
        os.system("adb -s %s shell rm -rf /sdcard/android/data/com.google.android.apps.maps" %self.serial)
        os.system("adb -s %s shell rm -rf /sdcard/android/data/com.google.android.GoogleCamera" %self.serial)
        g_common_obj.launch_app_from_home_sc("Maps")
        time.sleep(2)
        g_common_obj.launch_app_from_home_sc("Camera")
        time.sleep(2)
        exist1=self.fileop.file_exists("/sdcard/android/data/com.google.android.apps.maps","DEVICE")
        print "maps's data exist %s" %exist1
        if exist1:
                self.insert_info("Maps data","exist")
        exist2=self.fileop.file_exists("/sdcard/android/data/com.google.android.GoogleCamera","DEVICE")
        print "toutube's data exist %s" %exist2
        if exist2:
                self.insert_info("Camera data","exist")
        #take a shot before gota
        self.d.press.home()
        self.locator.btn_apps.click()
        time.sleep(2)
        g_common_obj.take_screenshot("/tmp/gota/applistbefore.png")
        time.sleep(2)
        self.d.press.home()
        time.sleep(3)

        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        self.d(text='System updates').click()
        count=0
        sn=g_common_obj2.getSerialNumber()
        #restart & install ota package
        if self.d(text="Restart & install"):
            print "[INFO]: Restart device and install OTA package"
            self.d(text="Restart & install").click()
        time.sleep(10)
        count=count+10
        #check if update successfully
        cmd="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn

	#haley this for debugging
        while 1:
            print "[INFO]: wait for android os..."
            if os.popen(cmd).read().count("recovery")>0:
                print "[INFO]: entered revovery mode,then reboot to main os"
                g_common_obj2.system_reboot()
                print "[INFO]: reboot is over"
            elif os.popen(cmd).read().count("StatusBar")>0:
                print "[INFO]: entered main os successfully"
                self.insert_info("GOTA status","after GOTA")
                self.insert_info("GOTA update prompt", "fail")
                break
            elif os.popen(cmd).read().count("com.google.android.googlequicksearchbox/com.google.android.launcher.GEL")>0:
                print "[INFO]: entered main os successfully"
                self.insert_info("GOTA status","after GOTA")
                self.insert_info("GOTA update prompt", "fail")
                break
            elif os.popen(cmd).read().count("com.google.android.gms/com.google.android.gms.update.CompleteDialog")>0:
                print "[INFO]: entered main os successfully"
                self.insert_info("GOTA status","after GOTA")
                self.insert_info("GOTA update prompt", "Successfully")
                break
            time.sleep(30)
            count=count+30
            print "count=%d, timeout=%d" %(count,timeout)
            if count > timeout:
                self.insert_info("GOTA status","after GOTA")
                self.insert_info("GOTA update prompt","fail")
                break
        if self.d(text="OK").exists:
            self.d(text="OK").click()
        time.sleep(5)
        count=count+5


	cache_after_gota=os.popen("adb -s %s shell df |grep -E '/cache '" %self.serial).read()
	fplist=cache_after_gota.split()
	cache_used=fplist[2].encode('utf-8')
	self.insert_info("Cache patition used after GOTA",cache_used)
	cache_after_gota=os.popen("adb -s %s shell df |grep -E '/data '" %self.serial).read()
	fplist=cache_after_gota.split()
	data_used=fplist[2].encode('utf-8')
	self.insert_info("Data patition used after GOTA",data_used)

        self.update_info(self.get_filename())
        print "[INFO]: android upgrades successfully"
        self.unlock_screen()
        if self.d(text="Allow").exists:
            self.d(text="Allow").click()
        time.sleep(5)

	#take a shot after GOTA
	self.d.press.home()
	self.locator.btn_apps.click()
        time.sleep(2)
	g_common_obj.take_screenshot("/tmp/gota/applistafter.png")
        time.sleep(2)
	self.d.press.home()

    def restart_and_install_package(self, timeout=3600):
        #enter system updates
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className = "android.widget.ScrollView", resourceId = "com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search = True,
            className = "android.widget.TextView").click()
        self.d(text = 'System updates').click()
        #restart & install ota package
        if self.d(text = "Restart & install").exists:
            print "[INFO]: Restart device and install OTA package"
            self.d(text = "Restart & install").click()
        count = 0
        time.sleep(10)
        count = count + 10
        #check if update successfully
        adb_cmd="adb devices"
        cmd = "adb -s %s shell dumpsys window|grep mCurrentFocus" %self.serial
        while os.popen(cmd).read().count("com.google.android.gms/com.google.android.gms.update.CompleteDialog")==0:
            print "[INFO]: wait for android os..."
            if os.popen(adb_cmd).read().count("recovery")>0:
                print "[INFO]: entered revovery mode,then reboot to main os"
            elif os.popen(cmd).read().count("StatusBar")>0:
                print "[INFO]: entered main os successfully"
                break
            elif os.popen(cmd).read().count("com.google.android.googlequicksearchbox/com.google.android.launcher.GEL")>0:
                print "[INFO]: entered main os successfully"
                break
            time.sleep(30)
            count=count+30
            assert count<timeout, "[ERROR]: Upgrading exceeds limit %d second" %timeout

    def restart_and_install_package_after_remount(self, timeout=3600):
        #enter system updates
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className = "android.widget.ScrollView", resourceId = "com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search = True,
            className = "android.widget.TextView").click()
        self.d(text = 'System updates').click()
        #restart & install ota package
        if self.d(text = "Restart & install").exists:
            print "[INFO]: Restart device and install OTA package"
            self.d(text = "Restart & install").click()
        #check update status and enter mani os
        count = 0
        time_total=0
        adb_cmd="adb devices"
        cmd = "adb -s %s shell dumpsys window|grep mCurrentFocus" %self.serial
        while os.popen(cmd).read().count("com.google.android.googlequicksearchbox/com.google.android.launcher.GEL")==0:
            print "[INFO]: wait for android os..."
            if count==6:
                self.reboot_device(180)
                break
            time.sleep(60)
            time_total=time_total+60
            count=count+1
        print "[INFO]: entered main os successfully"
        time.sleep(30)

    def basic_gota_update(self,timeout=1800):
        #download_timeout=5.0/6.0 * timeout
        #install_timeout=1.0/6.0 * timeout
        #check the base build number
        #haley delete self.check_build_no(base)
        #download gota package and auto retry download, if download is interrupted
        #haley skip this for debug self.download_gota_package_and_auto_retry(target, download_timeout)
        #restart and install gota package
        self.restart_and_install_gota_package(timeout)
        #check the gota is successfully with target build number

        #haley delete self.check_build_no(target)
        #print "[INFO]: OTA update from %s to %s successfully" % (base, target)
        #time.sleep(10)

    def cancel_gota_before_GOTA_finish(self):
        #download_timeout=5.0/6.0 * timeout
        #install_timeout=1.0/6.0 * timeout
        #check the base build number
        #self.check_build_no(base)
        #download gota package and auto retry download, if download is interrupted
        #self.download_gota_package_and_auto_retry(target, download_timeout)

        self.unlock_screen()
        time.sleep(1)
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        self.d(text='System updates').click()
        if self.d(text="Restart & install").exists:
            print "[INFO]: Restart device and install OTA package"
            self.d(text="Restart & install").click()
            time.sleep(2)
        if self.d(text="Cancel install").exists:
            print "[INFO]: cancel install OTA package"
            self.d(text="Cancel install").click()
            time.sleep(2)
        assert self.d(text="Cancel install").exists==False
        time.sleep(2)
        self.d.press.home()

    def enter_system_updates(self):
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        self.d(text='System updates').click()

    def set_virtual_battery_level(self, percent = 0):
        print "[info]--- Set virtual battery level: %d" % percent
        cmd = "am broadcast -a android.intent.action.BATTERY_CHANGED --ei level %d --ei scale 100" % percent
        g_common_obj.adb_cmd(cmd)

    def system_update_downloaded_notification_exist(self):
        time.sleep(3)
        if self.d(textContains="Downloaded").exists !=True:
            self.d.open.notification()
            time.sleep(5)
        assert self.d(textContains="Downloaded").exists==True, "[ERROR]: the notification is not correct for gota package downloaded"
        print "[INFO]: The notification is correct for gota package downloaded"

    def low_power_get_reasonable_prompt(self):
        if self.d(text="Battery too low to install system update. Connect charger to continue.").exists:
            print "[INFO]: The notification is correct for gota low power"
            return
        assert self.d(text="Restart & install").exists == False
        print "[INFO]: The notification is not correct for gota low power"
        time.sleep(2)
        self.d.press.home()

    def appPrepare(self):
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        self.arti_obj = Artifactory(self.audio.cfg.get("datapath"))
        self.file_name = self.audio.cfg.get("push_audio").split("/")[-1].replace("\"","")
        self.push_path = self.audio.cfg.get("push_audio").split("\" \"")[1].replace("\"","")
        ret_file = self.arti_obj.get(self.file_name)
        #if os.path.exists(ret_file):
            #print "[Download]: Artifactory method"
            #g_common_obj.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + self.push_path + "\"")
        if 1:
            print "[Download]: FileDownloader method"
            audio = self.audio.cfg.get("push_audio")
            datapath= self.audio.cfg.get("datapath")
            host_file_path= self.cfg.get("host_file_path")
            print "audio is %s"%audio
            print "datapath is %s"%datapath
            #g_common_obj.sync_file_from_content_server(self.audio.cfg.get("host_file_path"), self.audio.cfg.get("Datapath"))
            self.file_download(datapath,host_file_path)
            print "*********************************************"
            g_common_obj.adb_cmd_common(self.audio.cfg.get("push_audio"))
            print "*********************************************"
        g_common_obj.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))
        self.audio.set_orientation_n()

    def audioPlayback(self):
        self.appPrepare()
        g_common_obj2.system_reboot()
        self.audio.enterPlayPageFromHome()
        self.enterSongsPage()
        self.d(scrollable=True).scroll.vert.to(textContains=self.audio.cfg.get("audio_name"))
        self.audio.playMusic(self.audio.cfg.get("audio_name"))
        time.sleep(3)
        self.audio.musicRepeat(1)


    def download_content(self, url, app_name):
    # download content from url
        for i in range(10):
            if os.path.isfile(app_name):
                print "%s exists" % app_name
                break
            cmd = "wget -c " + url + app_name
            os.system(cmd)
        assert os.path.isfile(app_name)
    def lock_screen(self):
        '''
        Lock screen.
        '''
        print "[Info] ---Lock screen."
        self.d.sleep()
        self.d.wakeup()
        assert self.dsystemdomainsImpl(description="Unlock").exists

    def unlock_screen_gota(self):
        print "[INFO]: Unlock screen"
        d = Device(self.serial)
        for i in range(10):
            d.wakeup()
            time.sleep(1.5)
            name = "com.android.systemui:id/lock_icon"
            if d(resourceId=name).exists:
                d(resourceId=name).drag.to(resourceId="com.android.systemui:id/clock_view", steps=100)
            if d(resourceId=name).exists == False:
                break
        assert d(resourceId=name).exists == False

    def unlock_screen(self, passwd="abcd", status=True):
        '''
        Unlock screen.
        '''
        print "[Info] ---Unlock screen."
        self.d.wakeup()
        if self.d(description="Unlock").exists:
            self.d(description="Unlock").drag.to(resourceId = "com.android.systemui:id/clock_view")
        time.sleep(3)
        assert not self.d(description="Unlock").exists
        if self.d(resourceId = "com.android.systemui:id/passwordEntry").exists:
            self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
            self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(passwd)
            self.d.click(700,1000)
            time.sleep(3)
            if status:
                assert not self.d(resourceId = "com.android.systemui:id/passwordEntry").exists
            else:
                assert self.d(resourceId = "com.android.systemui:id/passwordEntry").exists \
                        or self.d(textStartsWith = "You have incorrectly typed your password").exists


    def set_airplane_mode(self, status="OFF"):
        '''
        Set airplane mode status.
        '''
        print "[Info] ---Set airplane mode status %s." % status
        if not self.d(text = "Airplane mode").exists:
            self.d(text = "More…").click.wait()
        if not self.d(resourceId = "android:id/switchWidget").text == status:
            self.d(resourceId = "android:id/switchWidget").click.wait()
        assert self.d(resourceId = "android:id/switchWidget").text == status


    def check_keywork_in_logcat(self):
        print "[INFO]: launch_settings is pass"

    def get_systemtime(self):
        print "[INFO]: launch_settings is pass"

    def connect_wifi(self):
        print "[INFO]: launch_settings is pass"

    def disconnect_wifi(self):
        print "[INFO]: launch_settings is pass"

    def xxx(self):
        print "[INFO]: launch_settings is pass"

    def verify_cannot_input_chars(self):
        '''
        Verify cannot input chars.
        '''
        print "[Info] ---Verify cannot input chars."
        chars="a1A!"
        if self.d(text = "OK").exists:
            self.d(text = "OK").click.wait()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(chars)
        self.d.click(700,1000)
        assert self.d(textStartsWith = "Try again in").exists
        assert not self.d(text = "Wrong Password").exists

    def set_wallpaper_systemdomainsImplrandomly(self):
        '''
        Set wallpaper randomly.
        '''
        from random import randrange
        self.d(text = "Display").click.wait()
        self.d(text = "Wallpaper").click.wait()
        self.d(text = "Live Wallpapers").click.wait()
        i = randrange(5)
        print "[Info] ---Set wallpaper index %d." % i
        self.d(packageName = "com.android.wallpaper.livepicker", index=i).click.wait()
        self.d(text = "Set wallpaper").click.wait()
        assert not self.d(text = "Set wallpaper").exists
        self.d.press.back()
        self.d.press.back()

    def check_gota_title_description(self):
        #go to setting->`about tablet>system update>check the description of gota info
        title_check=self.get_info("title check")
        print "title %s" % title_check
        if not title_check==False:
            return
        status=self.update_gotastatus()
        assert not status=="no GOTA package"
        title_check=self.get_info("title check")
        print"***********************************"
        print title_check
        print "**********************************"
        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
        des_check=self.get_info("des check")
        print "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
        assert des_check and title_check
        print "[INFO]: the title and description does exist"

    def check_gota_upgrade_package(self):
        print 'start checking the size of gota package'
        size_check=self.get_info("GOTA package size")
        print "size %s" % size_check
        if not size_check == False:
            return
        status=self.update_gotastatus()
        assert not status=="no GOTA package"
        package_check=self.get_info("GOTA package size")
        assert package_check
        size=round(float(package_check))
        print "size %s" % size
        assert size<2000, "[ERROR]:the size:%s is larger than 700M" % size
        print "[INFO]:the size:%s is not larger than 2000M" % size

    def basic_gota_update_without_reboot(self,target,timeout=10800):
        print "[INFO]: gota without reboot  start             "
        download_timeout=5.0/6.0 * timeout
        install_timeout=1.0/6.0 * timeout
        #check the base build number
#         self.check_build_no(base)
        #download gota package and auto retry download, if download is interrupted
        self.download_gota_package_and_auto_retry( target,download_timeout)
        self.d.press.back()
        self.d.press.back()
        self.d.press.back()
        self.d.press.back()
        print "[INFO]: gota without reboot is finished"
        #restart and install gota package
#         self.restart_and_install_gota_package(install_timeout)
        #check the gota is successfully with target build number
#         self.check_build_no(target)
#         print "[INFO]: OTA update from %s to %s successfully" % (base, target)
        time.sleep(10)

    def restart_install_gota_and_verify(self,timeout=1800):
        count=0
        sn=g_common_obj2.getSerialNumber()
        #restart & install ota package
        if self.d(text="Restart & install"):
            print "[INFO]: Restart device and install OTA package"
            self.d(text="Restart & install").click()
        time.sleep(60)
        count=count+60
        #check if update successfully
        adb_cmd="adb devices"
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("com.google.android.gms/com.google.android.gms.update.CompleteDialog")==0:
            print "[INFO]: wait for android os..."
            if os.popen(adb_cmd).read().count("recovery")>0:
                print "[INFO]: entered revovery mode,then reboot to main os"
                g_common_obj2.system_reboot()
                print "[INFO]: reboot is over"
            elif os.popen(cmd3).read().count("StatusBar")>0:
                print "[INFO]: entered main os successfully"
                break
            elif os.popen(cmd3).read().count("com.google.android.googlequicksearchbox/com.google.android.launcher.GEL")>0:
                print "[INFO]: entered main os successfully"
                break
            time.sleep(10)
            count=count+10
            assert count<timeout, "[ERROR]: Upgrading exceeds limit %d second" %timeout
        cmd4="adb -s %s root" %sn
        result = os.popen(cmd4).read()
        print "[INFO]: %s" %result
        assert result.find("error")==-1, "[ERROR]: adb root failed"
        print "[INFO]: adb root successfully"
        time.sleep(5)
        count=count+5
        if self.d(text="OK").exists:
            self.d(text="OK").click()
        print "[INFO]: Restart & install is over"

    def check_detect_image(self):
        print "check_detect_image start"
        self.unlock_screen()
        time.sleep(1)
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        self.d(text='System updates').click()
        counter=0
        while self.d(text="Your system is up to date.").exists:
            if counter==6:
                break
            if self.d(text="Check now").exists:
                self.d(text="Check now").click()
            elif self.d(text="Check for update").exists:
                self.d(text="Check for update").click()
            time.sleep(10)
            time_total=time_total+10
            counter=counter+1
        assert not self.d(text="Your system is up to date.").exists

        if self.d(text="Download").exists:
            self.d(text="Download").click()
            time.sleep(3)
	    GOTA_status="dowoloading GOTA package"
        #if the system is in downloading or Retry download button
        if self.d(textContains="Downloading").exists or self.d(textContains="Retry download").exists or self.d(textContains="Waiting to download").exists:
	    target_image=self.get_info("Target image")
	    print "target %s" % target_image
            title_check=self.d(className="android.widget.TextView", resourceId="com.google.android.gms:id/title") \
            .sibling(textContains=target_image).exists
	    print "%s" %title_check
	    assert title_check

    def update_info(self,filename):
        cmd="cp -f '"+filename+"' '/tmp/gota/test.log'"
        os.system(cmd)
        print "finish update"

    def insert_info(self,key,value):
        print "insert %s:%s" % (key,value)
        filename=self.get_filename()
        print filename
        if self.get_info(key)==False:
            print "insert not exist"
            fp=open(filename,'a')
            cmd=key+":"+value
            fp.write(cmd)
            fp.write('\n')
            fp.close()
            return
        print "insert exist"
        fp=open(filename,'rw')
        record_list=fp.readlines()
        i=0
        for element in record_list:
            result=element.find(key)
            if result==0:
                new_element=key+":"+value+"\n"
                record_list[i]=new_element
                fp.close()
                fp=open(filename,'w')
                fp.writelines(record_list)
                fp.close()
                return
            i=i+1

    def get_info(self,key):
        print "get_info %s" % key
        fp=open(self.gota_record,'r')
        record_list=fp.readlines()
        for element in record_list:
            result=element.find(key)
            if element.find(key)==0:
                result=element.replace(key,"",1)
                result=result.replace(":","",1)
                result=result.replace("\n","",1)
                fp.close()
                return result
        fp.close()
        return False

    def get_filename(self):
	fp=open(self.gota_record,'r')
	filename=fp.readline()
	filename=filename.strip("\n")
	filename=filename.strip("latest GOTA test:")
	fp.close()
	return filename

    def update_gotastatus(self):
	print "update_gotastatus start"
	self.unlock_screen()
	time.sleep(1)
	g_common_obj.launch_app_am("com.android.settings", ".Settings")
	self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
	    .child_by_text(
	    "About tablet",
	    allow_scroll_search=True,
	    className="android.widget.TextView").click()
	self.d(text='System updates').click()
        counter=0
	time_total=0
	GOTA_status="before GOTA"
        while self.d(text="Your system is up to date.").exists:
	    print "counter %s" %counter
            if counter==6:
                break
            if self.d(text="Check now").exists:
                self.d(text="Check now").click()
            elif self.d(text="Check for update").exists:
                self.d(text="Check for update").click()
            time.sleep(5)
            time_total=time_total+10
            counter=counter+1
	print "point"
	current=self.get_info("GOTA status")
	print "%s" %current
	if not cmp(self.get_info("GOTA status"),"after GOTA"):
	    #haley for debug
	    #assert self.d(text="Your system is up to date.").exists
	    print "anti roll back check"
	    return "no GOTA package"
	if self.d(text="Your system is up to date.").exists:
	    GOTA_status="no GOTA package"
        if self.d(text="Download").exists:
            self.d(text="Download").click()
            time.sleep(3)
	    GOTA_status="dowoloading GOTA package"
        #if the system is in downloading or Retry download button
        if self.d(textContains="Downloading").exists or self.d(textContains="Retry download").exists or self.d(textContains="Waiting to download").exists \
		or self.d(textContains="Restart & install").exists:
            title_check=self.d(className="android.widget.TextView", resourceId="com.google.android.gms:id/title").exists or self.d(className="android.widget.TextView", resourceId="com.google.android.gms:id/suw_layout_title").exists 
            des_check=self.d(className="android.widget.TextView", resourceId="com.google.android.gms:id/description").exists
        
	    if title_check==True:
	        self.insert_info("title check","True")
	    if des_check==True:
	        self.insert_info("des check","True")
	    info = self.d(resourceId="com.google.android.gms:id/size").info
            utext=info[u'text']
            usize=utext.split(' ')[2]
	    print "usize %s" %usize
            size_str=usize.encode('utf-8')
	    self.insert_info("GOTA package size",size_str)
            if self.d(textContains="Retry download").exists:
                self.d(textContains="Retry download").click()
        if self.d(text="Restart & install").exists:
	    GOTA_status="dowoloaded GOTA package"
            print "[INFO]: The OTA package is download successfully"
        time.sleep(1)
	if cmp(self.get_info("GOTA status"),"after GOTA"):
	    self.insert_info("GOTA status",GOTA_status)
	    self.update_info(self.get_filename())
	    print "GOTA_status: %s" % GOTA_status
	return GOTA_status

    def wait_gotastatus_to_downloaded(self,timeout = 10800):
        time_total = 0
        self.unlock_screen()
        time.sleep(1)
        #self.gota.connect_AP(self.ssid, self.passwd)
        self.connect_AP(self.ssid, self.passwd)
        self.audio.launch_music_from_home()
        if self.d(description="Pause").exists:
            self.d(description="Pause").click()
        time.sleep(2)
        baterry_status=self.get_current_battery_status()
        print baterry_status
        while 1:
            if baterry_status == '4':
                break
            if baterry_status == '2':
                while 1:
                    capacity=self.get_current_battery_capacity()
                    batter_capacity = int(capacity)
                    print "baterry capacity is %s"%batter_capacity
                    if batter_capacity < 30:
                        self.d.sleep()
                        time.sleep(1800)
                    else:
                        #self.wait_for_android_os()
                        self.d.wakeup()
                        break
                break
        while not self.update_gotastatus() == "dowoloaded GOTA package":
            time.sleep(300)
            time_total = time_total+300
            if time_total > timeout:
                return False
        return True

    def download_gota_content(self):
        url = self.cfg.get("host_file_url")
        host_file_path = self.cfg.get("host_file_path")
        host_file_zip = self.cfg.get("host_file_zip")
        #exist = self.system2.file_exists(host_file_zip, "DEVICE")
        if self.fileop.file_exists("/tmp/resource_apk", "HOST"):
            os.system("rm /tmp/resource_apk")
        os.system("mkdir /tmp/resource_apk")
        if not self.fileop.file_exists(host_file_zip, "HOST"):
            cmd = "mkdir %s;cd %s;wget -c %s"% (host_file_path, host_file_path, url)
            print cmd
            os.system(cmd)
            assert os.path.isfile(host_file_zip)
            cmd2="unzip -n %s/gota.zip -d %s" %(host_file_path, host_file_path)
            print "[INFO]: unzip %s" %cmd2
            os.system(cmd2)
            os.system("cp /tmp/gota/sample.apk /tmp/resource_apk")

    def download_gota_content_artifactory(self, sub_path=None, download_file=None):
        if os.path.isfile("/etc/oat/sys.conf"):
            common_url = self.config.read(section='artifactory').get("location")
            remote_server = Artifactory(common_url + sub_path)
            for _ in range(5):
                return_file = remote_server.get(download_file)
                print return_file
                if return_file is not None and os.path.isfile(return_file):
                    break
                time.sleep(2)
        host_file_path = self.cfg.get("host_file_path")
        cmd = "mkdir %s"% (host_file_path)
        os.system(cmd)
        cmd2 = "cp %s %s"%(return_file, host_file_path)
        print cmd2
        os.system(cmd2)
        tmp_file=host_file_path+"/gota.zip"
        assert os.path.isfile(tmp_file)
        cmd2="unzip -n %s/gota.zip -d %s" %(host_file_path, host_file_path)
        print "[INFO]: aaaa %s" %cmd2
        os.system(cmd2)
        host_file_zip = self.cfg.get("host_file_zip")
        assert os.path.isfile(host_file_zip)
        if self.fileop.file_exists("/tmp/resource_apk", "HOST"):
            os.system("rm /tmp/resource_apk")

        os.system("mkdir /tmp/resource_apk")
        os.system("cp /tmp/gota/sample.apk /tmp/resource_apk")

    def mount_device(self):
        """Mount device and delete file under /oem/meida and /oem/app"""
        print "[INFO]: Mount device"
        mount_cmd = "mount -o remount,rw /oem"
        self.shell_command(mount_cmd)

    def shell_command(self, shell_cmd):
        """Run adb shell command"""
        cmd = "adb -s %s shell %s" % (self.serial, shell_cmd)
        r_code, r_mes = commands.getstatusoutput(cmd)
        return r_code, r_mes

    def push_oem_file(self):
        """Push file to device"""
        self.push_file = {
            "com.estrongs.android.pop-1.apk": "/oem/app/",
        }
        self.file_path=ConfigHandle().read_host_file_path()
        for new_file in self.push_file:
            old_file = self.file_path + "/file/" + new_file
            value = self.push_file[new_file]
            if isinstance(value, dict):
                device_path = value[self.version]
            else:
                device_path = value
            print "[INFO]:Push %s to %s" % (old_file, device_path)
            self.push_command(old_file, device_path)

    def factory_reset(self):
        print "[INFO]: Click factory reset"
        d = Device(self.serial)
        self.launch_settings()
        """
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.toBeginning()
        d(text="Backup & reset").click()
        """
        d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "Backup & reset",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        time.sleep(2)
        d(text="Factory data reset").click()
        time.sleep(2)
        d(text="Reset tablet").click()
        time.sleep(2)
        d(text="Erase everything").click()
        time.sleep(2)

    def launch(self, packageName, activityName):
        d = Device(self.serial)
        d.press.back()
        d.press.back()
        d.press.home()
        cmdStr = "am start -S -n %s/%s" % (packageName, activityName)
        self.shell_command(cmdStr)
        time.sleep(10)

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

    def switch_to_Guest(self, name="Guest"):
        print "[INFO]: Switch to guest"
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(scrollable=True).scroll.vert.to(text="Users")
        self.d(text="Users").click.wait()
        time.sleep(2)
        self.d(text="Guest").click.wait()
        time.sleep(5)
        if self.d(text="Yes, continue").exists:
            self.d(text="Yes, continue").click()
        time.sleep(2)
        #self.unlock_screen()
        #time.sleep(5)
        self.d.press.home()

    def switch_to_owner(self):
        print "[INFO]: Switch to owner"
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(text="Users").click.wait()
        time.sleep(2)
        self.d(text="Owner").click.wait()
        time.sleep(5)
        #self.unlock_screen()
        #time.sleep(5)
        self.d.press.home()

    def launch_oem_app(self):
        '''
        Implements the apks are visible to launch.
        '''
        print "[Info] ---Launch ES File Explorer app."
        g_common_obj.launch_app_am("com.estrongs.android.pop",".view.FileExplorerActivity")
        time.sleep(5)
        while self.d(text="Long press").exists:
            self.d(text="Long press").click()
            time.sleep(2)
        assert self.d(resourceId = "com.estrongs.android.pop:id/gridview").exists
        g_common_obj.stop_app_am("com.estrongs.android.pop")
        print "[Info] --- Launch ES File Explorer app success"

    def launch_app_am(self, packagename, activityname):
        """
        Launch app from am command
        Parameter packagename is the app's package name to be launched
        Parameter activityname is the app's entry point name
        """
        cmdstr = "adb -s %s shell am start -S -n %s/%s" % (self.serial,packagename, activityname)
        print "************************************"
        print cmdstr
        print "************************************"

        return os.system(cmdstr)

    def launch_es_filemanager_app(self):
        '''
        Implements the apks are visible to launch.
        '''
        print "[Info] ---Launch ES File Explorer app."
        self.launch_app_am("com.estrongs.android.pop",".view.FileExplorerActivity")
        print "lauuch ES filemanager successfully ......"
        time.sleep(5)
        while self.d(text="Long press").exists:
            self.d(text="Long press").click.wait()
        time.sleep(1)
        if self.d(text="New").exists:
            self.d(text="New").click.wait()
            time.sleep(2)
            self.d(text="File").click.wait()
            time.sleep(2)
            self.d(text="OK").click.wait()
            time.sleep(1)
        time.sleep(3)
        assert self.d(text="File").exists,"Error"
        print "[Info] --- create file success"
        time.sleep(2)
        assert self.d(resourceId = "com.estrongs.android.pop:id/gridview").exists
        g_common_obj.stop_app_am("com.estrongs.android.pop")
        print "[Info] --- Launch ES File Explorer app success"

    def switch_to_user(self, name="Users"):
        '''
        Add and switch to new user.
	    '''
        print "Switch to user"

        self.d(text="Users").click.wait()
        print "Add new user"
        if not self.d(text=name, resourceId="android:id/title").exists:
            self.d(text="Add user or profile").click.wait()
            self.d(text="User").click.wait()
            self.d(text="OK").click.wait()
            self.d(text="Set up now").click.wait()
            time.sleep(10)
            self.unlock_screen()
            self.go_through_guideline(name)
            time.sleep(3)

    def only_device_owner_could_GOTA(self):
        print "check only devices owner could GOTA"
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "About tablet",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        assert not self.d(text="System updates").exists
        self.d.press.home()

    def go_through_guideline(self, name="Users"):
        '''
        Go through guideline.
        '''
        print "Go through guideline."
        for i in range(5):
            if self.d(text="OK").exists:
                self.d(text="OK").click()
                time.sleep(2)
        while self.d(description="Start").exists==False:
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
            self.d(text="Finish").click.wait()
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()
        for i in range(5):
            if self.d(text="OK").exists:
                self.d(text="OK").click.wait()
                time.sleep(2)
        self.d.sleep()
        time.sleep(5)
        self.unlock_screen()
        while self.d(text="GOT IT").exists==False:
            time.sleep(2)
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        #assert self.locator.home_flag.exists

    def check_AFW_profile_work_before_GOTA(self):
        self.api.launch_app("Sample MDM")
        time.sleep(2)
        self.d(text = "Setup Managed Profile").click.wait()
        if self.d(text="No").exists:
            self.d(text ="No").click.wait()
        self.d(text = "Set up").click.wait()
        self.d(text = "OK").click.wait()
        time.sleep(10)
        if self.d(text="Setup all done").exists:
            if self.d(description="Icon").exists:
                self.d(description="Icon").click.wait()
            time.sleep(2)
        print "[Info] --- Profile work Sample MDM is successful before gota"

    def check_AFW_data_in_managed_profile_before_GOTA(self):
        self.api.launch_app("Work Contacts")
        time.sleep(2)
        self.d.press.back()
        self.api.launch_app("Work Contacts")
        time.sleep(2)
        self.d(resourceId = "com.android.contacts:id/floating_action_button").click.wait()
        self.d(text = "Keep local").click.wait()
        Contactsmanagedname = "ContactsInManagedProfile"
        os.system("adb -s %s shell input text '%s'" % (self.serial, Contactsmanagedname))
        self.d.press.back()
        self.d.press.back()
        if self.d(text = "ContactsInManagedProfile").exists:
            time.sleep(2)
        print "[Info] --- Contacts data exist in managed profile before gota"

    def check_AFW_data_in_personal_profile_before_GOTA(self):
        self.api.launch_app("Contacts")
        time.sleep(2)
        self.d.press.back()
        self.api.launch_app("Contacts")
        time.sleep(2)
        self.d(resourceId = "com.android.contacts:id/floating_action_button").click.wait()
        if self.d(text="OK").exists:
            self.d(text="OK").click.wait()
        if self.d(text="Keep local").exists:
            self.d(text = "Keep local").click.wait()
        Contactspersonalname = "ContactsInPersonalProfile"
        os.system("adb -s %s shell input text '%s'" % (self.serial, Contactspersonalname))
        self.d.press.back()
        self.d.press.back()
        if self.d(text="ContactsInPersonalProfile").exists:
            time.sleep(2)
        print "[Info] --- Contacts data exist in personal profile before gota"

    def check_AFW_chrome_restriction_before_GOTA(self):
        self.api.launch_app("Work Sample MDM")
        if self.d(description="Icon").exists:
            self.d(description="Icon").click.wait()
        self.d(text = "APP Provisioning").click.wait()
        self.d(text = "Enable System Applications").click.wait()
        #self.d(text = "com.android.chrome").click.wait()
        self.d(text="com.google.android.youtube").click.wait()
        self.d(text = "Enable System Applications to Profile").click.wait()
        self.d.press.back()
        self.d(text = "APP Config and Policy").click.wait()
        if self.d(text = "Clear Application Restriction (Chrome)").exists:
            self.d(text = "Clear Application Restriction (Chrome)").click.wait()
            self.d(text = "OK").click.wait()
        self.d(text = "Set Application Restriction (Chrome)").click.wait()
        self.d(text = "OK").click.wait()
        self.d(text = "Get Application Restriction (Chrome)").click.wait()
        self.d(text = "OK").click.wait()
        print "[Info] --- Enable chrome app to Profile"
        self.d.press.home()
        self.api.launch_app("Work YouTube")
        time.sleep(5)
        if self.d(text="Loading…").exists:
            self.d.press.back()
            time.sleep(2)
        if self.d(text="Checking info…").exists:
            self.d.press.back()
            time.sleep(2)
        if self.d(text="Add your account").exists:
            self.d.press.back()
            time.sleep(2)
        if self.d(text = "Done").exists:
            self.d(text = "Done").click.wait()
            time.sleep(2)
        if self.d(text="Next").exists:
            self.d(text="Next").click.wait()
            time.sleep(2)
        if self.d(text = "Got it").exists:
            self.d(text = "Got it").click.wait()
            time.sleep(2)
        self.d.press.menu()
        print "press menue......"
        time.sleep(3)
        self.d(text = "Settings").click.wait()
        print "click settings..............."
        time.sleep(3)
        self.d(text = "General").click.wait()
        time.sleep(2)
        if self.d(text="Managed by your administrator").exists:
            time.sleep(2)
        self.d.press.home()
        print "[Info] --- Chrome is restricted before gota"

    def check_AFW_profile_work_after_GOTA(self):
        self.api.launch_app("Work Sample MDM")
        time.sleep(2)
        if self.d(text="Sample MDM(Profile Owner)").exists:
            time.sleep(2)
        print "[Info] --- Profile work Sample MDM is successful after gota"

    def check_AFW_data_in_managed_profile_after_GOTA(self):
        self.api.launch_app("Work Contacts")
        time.sleep(2)
        if self.d(text="ContactsInManagedProfile").exists:
            time.sleep(2)
        print "[Info] --- Contacts data exist in managed profile after gota"

    def check_AFW_data_in_personal_profile_after_GOTA(self):
        self.api.launch_app("Contacts")
        time.sleep(2)
        if self.d(text="ContactsInPersonalProfile").exists:
            time.sleep(2)
        print "[Info] --- Contacts data exist in personal profile after gota"

    def check_AFW_chrome_restriction_after_GOTA(self):
        self.api.launch_app("Work YouTube")
        time.sleep(8)
        if self.d(text="Loading…").exists:
            self.d.press.back()
            time.sleep(2)
        if self.d(text="Checking info…").exists:
            self.d.press.back()
            time.sleep(2)
        if self.d(text="Add your account").exists:
            self.d.press.back()
            time.sleep(2)
        if self.d(text = "Done").exists:
            self.d(text = "Done").click.wait()
            time.sleep(2)
        if self.d(text="Next").exists:
            self.d(text="Next").click.wait()
            time.sleep(2)
        if self.d(text = "Got it").exists:
            self.d(text = "Got it").click.wait()
            time.sleep(2)
        print "press menue ..................."
        self.d.press.menu()
        print "click settings................."
        time.sleep(2)
        self.d(text = "Settings").click.wait()
        time.sleep(3)
        self.d(text = "General").click.wait()
        if self.d(text="Managed by your administrator").exists:
            time.sleep(2)
        print "[Info] --- Chrome is restricted after gota"

    def check_AFW_devices_owner_after_GOTA(self):
        self.api.launch_app("Sample MDM")
        time.sleep(2)
        if self.d(text="Sample MDM(Device Owner)").exists:
            time.sleep(2)
        print "[Info] --- Device owner work Sample MDM is successful after gota"

    def check_AFW_data_in_app_after_GOTA(self):
        self.api.launch_app("Contacts")
        time.sleep(2)
        if self.d(text="ContactsInManagedProfile").exists:
            time.sleep(2)
        print "[Info] --- Contacts data exist in device owner after gota"

    def check_gota_package_deleted(self):
        cache_file = '/cache/update.zip'
        data_file = '/data/data/com.google.android.gms/app_download/update.zip'
        cache = os.popen("adb -s %s shell ls /cache/update.zip" %self.serial).read().strip()
        data = os.popen("adb -s %s shell ls /data/data/com.google.android.gms/app_download/update.zip" %self.serial).read().strip()
        if cache_file != cache and data_file != data:
            print "[Info] --- The GOTA package is deteled after GOTA"
        else:
            assert False, "[Info] --- The GOTA package is not deteled after GOTA" 

    def remount_device(self):
        os.system("adb -s %s root" %self.serial)
        os.system("adb -s %s root" %self.serial)
        os.system("adb -s %s disable-verity" %self.serial)
        os.system("adb -s %s reboot" %self.serial)
        time.sleep(60)
        os.system("adb -s %s root" %self.serial)
        time.sleep(30)
        os.system("adb -s %s root" %self.serial)
        result = 'remount succeeded'
        remount_result = os.popen("adb -s %s remount" %self.serial).read().strip()
        print "[Info] --- remount result is %s" %remount_result
        if result == remount_result:
            print "[Info] --- remount successfull"
        else:
            assert False, "[Info] --- remount fail"
    def event_add(self):
        """
        @summary: add calendar event
        @parameter:
            waittime : the waittime between current and event start time
        @return: None
        """
        print "[INFO] Add a event "
        os.system("adb -s %s shell am start -a android.intent.action.EDIT -t vnd.android.cursor.item/event"%self.serial)
        time.sleep(3)
        cmdstr="adb -s %s shell input text 'Newevent'" % self.serial
        time.sleep(2)
        mesg = os.popen(cmdstr).read()
        time.sleep(1)
        assert mesg.find("Error:") == -1, \
        "ERROR:Event added command fail!"
        self.d(text = "Save").click()
    def launch_youtube(self):

        #self.func.init_youtube()
        self.d.press.home()
        os.system("adb -s %s shell am start -S com.google.android.youtube/com.google.android.apps.youtube.app.WatchWhileActivity" % self.serial)
        time.sleep(5)
        print "Youtube launched "
        for i in range(60):
            if self.d(text="What to Watch").exists and not self.d(text="OK").exists:
                break
            if self.d(text="OK").exists:
                self.d(text="OK").click.wait()
            if self.d(text="Skip").exists:
                self.d(text="Skip").click.wait()
            if self.d(text="Retry").exists:
                self.d(text="Retry").click.wait()
            time.sleep(2)
        #assert self.d(text="What to Watch").exists
        self.d.press.back()
        self.d(description="YouTube").click.wait()
        time.sleep(2)
        if self.d(text="Skip").exists:
            self.d(text="Skip").click.wait()
        assert not self.d(text="Skip").exists
        os.system("adb -s %s shell am force-stop com.google.android.youtube" % self.serial)
    def start_UI(self,conn,n):
        print "subprocess"
        self.chpid=os.getpid()
        print "subprocess chpid = %s" %self.chpid
        print "subprocess ppid = %s" %os.getppid()
        conn.send(self.chpid)
        n.value=1
        time.sleep(5)
        self.uiresult=os.system("adb -s %s shell uiautomator runtest  uiautomator-stub.jar bundle.jar -c com.github.uiautomatorstub.Stub"% self.serial)
        n.value=self.uiresult
        print "subprocess uiresult %s" %self.uiresult
        conn.send(self.uiresult)
        conn.close()
        print "subprocess finish"

    def wait_for_android_os(self):
        sn=self.serial
        time.sleep(60)
        cmd3="adb -s %s shell dumpsys window|grep mCurrentFocus" %sn
        while os.popen(cmd3).read().count("setupwizard")==0:
            if os.popen(cmd3).read().count("StatusBar")!=0:
                self.unlock_screen()
                break
            print "[INFO]: wait for android os..."
            time.sleep(15)
        time.sleep(5)
        print "RPC server start"
        print "main ppid = %s" %os.getpid()
        if os.popen("adb -s %s shell ps | grep uiautomator" % self.serial).read().count("uiautomator") > 0:
            result=os.popen("adb -s %s shell ps | grep uiautomator" % self.serial).read()
            print result
            plist=result.split()
            pid=plist[1]
            print pid
            os.system("adb -s %s shell ps | grep uiautomator"% self.serial)
            os.system("adb -s %s shell kill -9 %s"%(self.serial,pid))
            os.system("adb -s %s shell ps | grep uiautomator"% self.serial)
        print "1"
        if os.path.exists("/usr/lib/python2.7/dist-packages/uiautomator/libs"):
            jar_path = "/usr/lib/python2.7/dist-packages/uiautomator/libs"
        else:
            jar_path = "/usr/local/lib/python2.7/dist-packages/uiautomator/libs"
        bundle_path = jar_path + "/bundle.jar"
        uiautomator_path = jar_path + "/uiautomator-stub.jar"
        push_bundle_jar = "adb -s " + self.serial + " push " + bundle_path + " /data/local/tmp"
        push_uiautomator_jar = "adb -s " + self.serial + " push " + uiautomator_path + " /data/local/tmp"
        os.system("adb -s %s root > /dev/null 2>&1" % self.serial)
        print "push jar start ========"
        os.system(push_bundle_jar)
        os.system(push_uiautomator_jar)
        print "push jar end ========="
        i=0
        for i in range(0,100):
            parent_conn,child_conn=Pipe()
            num=Value('i',1)
            child_proc=Process(target=self.start_UI,args=(child_conn,num))
            child_proc.start()
            time.sleep(15)
            i+=10
            print i
            self.chpid=parent_conn.recv()
            print "point chpid %s" %self.chpid
            os.system("kill -9 %s" %self.chpid)
            print "finish"
            self.uiresult=num.value
            print "point uiresult %s" %self.uiresult
            if self.uiresult == 0:
                print "RPC server started !"
                break
    def startEmailApp(self):
        """
        Skip Email first launch screen
        """
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        #uername = self.cfg.get("email_account")
        #password = self.cfg.get("email_password")
        g_common_obj.launch_app_from_home_sc("Email")
        assert self.d(packageName = "com.android.email").wait.exists()
        if self.d(textContains="Getting").exists:
            self.d(textContains="Getting").wait.gone(timeout=600000)
        assert not self.d(textContains="Getting").exists, \
        "ERROR: SYNC for more than 10 mins"
        if self.d(resourceId="com.android.email:id/account_email").exists:
            self.d(resourceId="com.android.email:id/account_email").click()
            g_common_obj.adb_cmd_capture_msg(
                "input text " + self.cfg.get("email_account"))
            time.sleep(2)
            self.d(resourceId="com.android.email:id/next").click()
            self.d(resourceId="com.android.email:id/regular_password").\
            wait.exists()
            self.d(resourceId="com.android.email:id/regular_password").click()
            time.sleep(2)
            g_common_obj.adb_cmd_capture_msg(
                "input text " + self.cfg.get("email_pwd"))
            time.sleep(2)
            self.d(resourceId="com.android.email:id/next").click()
            assert self.d(text="Account options").wait.exists(timeout=60000)
            for _ in (0, 10):
                if self.d(
                    resourceId="com.android.email:id/next")\
                .wait.exists(timeout=60000):
                    self.d(resourceId="com.android.email:id/next").click()
                    if self.d(resourceId="com.android.email:id/account_name").wait.exists(timeout=60000):
                        self.d.press.back()
                        #self.d(resourceId="com.android.email:id/account_name").click()
                else:
                    break
            assert self.d(text="Inbox").wait.exists(timeout=60000)
        g_common_obj.back_home()
        
    def get_current_battery_status(self):
        sn=g_common_obj2.getSerialNumber()
        cmd = "adb  -s %s shell dumpsys battery" %sn
        result=os.popen(cmd).read()
        print result
        fplist=result.split(": ")
        print len(fplist)
        Bingo = False
        for element in fplist:
            print "element %s" % element
            if Bingo==True:
                elelist=element.split()
                print elelist
                bat_status= elelist[0]
                return bat_status
            if "status" in element:
                print "bingo"
                Bingo=True
                continue
        return True
    def get_current_battery_capacity(self):
        sn=g_common_obj2.getSerialNumber()
        cmd = "adb  -s %s shell dumpsys battery " %sn
        result=os.popen(cmd).read()
        print result
        fplist=result.split(": ")
        print len(fplist)
        Bingo=False
        for element in fplist:
            print element
            if Bingo==True:
                fsublist=element.split()
                print fsublist
                battery=int(fsublist[0])
                print battery
                return battery
            if "level" in element:
                print "bingo"
                Bingo=True
                continue
    def launch_calculator(self):
        '''
        Launch Calculator app.
        '''
        print "[Info] ---Launch Calculator app."
        #self.launch_app_am("com.android.calculator2", ".Calculator")
        self.launch_app_am("com.google.android.calculator", "com.android.calculator2.Calculator")
        time.sleep(1)
        assert self.d(text="=").exists
    def startApp():
        """
        Skip Calendar first launch screen
        """
        g_common_obj.launch_app_from_home_sc("Calendar")
        if self.d(text="Google Calendar").exists:
            while 1:
                self.d(scrollable=True).scroll.horiz.backward()
                if self.d(text="Got it").exists:
                    self.d(text="Got it").click.wait()
                    break
        if self.d(resourceId="com.android.calendar:id/add_event_button").exists:
            return
        time.sleep(10)
        if self.d(resourceId="com.android.calendar:id/add_event_button").exists:
            return
        while self.d(resourceId="com.android.calendar:id/right_arrow").wait.exists(timeout=3000):
            self.d(resourceId="com.android.calendar:id/right_arrow").click()
            self.d.wait.update()
        if self.d(
            resourceId="com.android.calendar:id/done_button").\
        wait.exists(timeout=60000):
            self.d(resourceId="com.android.calendar:id/done_button").click()
        g_common_obj.back_home()
    def enterSongsPage(self):
        """
            @summary: enter Albums Page
        """
        self.logger.debug("enter Songs Page")
        time.sleep(2)
        if self.d(text="Allow").exists:
            self.d(text="Allow").click()
        time.sleep(1)
        self.audio.click_menu()
        assert ((self.d(text = 'Google Play Music').exists) or (self.d(text='Listen Now').exists)),'Google Play Music not exists, click home not responded'
        self.audio.dut(text = 'My Library').click.wait()
        assert self.audio.dut(text = 'ALBUMS').exists,'ALBUMS not exists, click My Library not responded'
        self.logger.debug("click Songs")
        self.audio.dut(text = 'SONGS').click.wait()
        print "Enter Songs page successfully"

    def launch_calculator(self):
        '''
        Launch Calculator app.
        '''
        print "[Info] ---Launch Calculator app."
        g_common_obj.launch_app_am("com.google.android.calculator", "com.android.calculator2.Calculator")
        print "launch seccessfully.................."
        time.sleep(2)
        assert self.d(text="=").exists
