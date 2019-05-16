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
from uiautomator import Device
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.audio.audio_impl import AudioImpl
from testlib.util.repo import Artifactory
import testlib.system.system_impl
from testlib.util.process import shell_command,killall
import sys
from macpath import realpath
from multiprocessing import Process
from multiprocessing import Pipe
from multiprocessing import Value

class AutodetectImpl:
    """
    Implements System android framework actions.
    """
    gota_record="/tmp/GOTA_test/test.log"
    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.child_pid = 0
        self.cfg=cfg
        self.serial=g_common_obj2.getSerialNumber()
    """
    Android_Framework
    -------------------------------------------------------------------
    """
    def getprop_grep(self,keywords):
        cmdstr="adb -s %s shell getprop | grep  %s"%(self.serial,keywords)
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

    """
    IRDA_feature
    -------------------------------------------------------------------
    """

    """
    halctl -para para1
    """
    def halctl_cmd(self,para,para1):
        cmdstr="adb -s %s shell halctl %s %s"%(self.serial,para,para1)
        print "---------------this is halctl_cmd----------------"
        print cmdstr
        halctl_cmd=os.popen(cmdstr).read()
#         print  "The result of %s is %s"%(cmdstr,halctl_cmd)
        return halctl_cmd

    """
    find kmod via "adb shell halctl -l | grep kmod"
    """
    def find_kmod(self):
        search_module_str="adb -s %s shell halctl -l | grep kmod"% self.serial
        kmod_list=os.popen(search_module_str).read()
        kmod=kmod_list.split("kmod:")
        lines = [x.strip() for x in kmod]
        del lines[0]
        print lines
        return lines

    """
    kmod -para para1
    """
    def kmod_cmd(self,para,para1):
        cmdstr="adb -s %s shell kmod %s %s"%(self.serial,para,para1)
        kmod_cmd=os.popen(cmdstr).read()
        print  "The result of %s is %s"%(cmdstr,kmod_cmd)
        return kmod_cmd
    def kmod_i(self,para):
        return self.kmod_cmd("-i",para)
    def kmod_r(self,para):
        return self.kmod_cmd("-r",para)
    """
    kmod -para para1
    """
    def lsmod_grep(self,para):
        cmdstr="adb -s %s shell lsmod | grep %s"%(self.serial,para)
        lsmod_cmd=os.popen(cmdstr).read()
        print  "The result of %s is %s"%(cmdstr,lsmod_cmd)
        return lsmod_cmd


    """
    adb shell cat /proc/keywords
    """
    def cat_proc(self,keywords):
        cmd_str="adb -s %s shell cat /proc/%s"% (self.serial,keywords)
        cat_proc=os.popen(cmd_str).read()
        print  cat_proc
        return cat_proc
    """
    check keyword in logcat
    """
    def search_in_logcat(self,keywords):
        cmd_str="adb -s %s shell logcat -d | grep %s"%(self.serial,keywords)
        print "[INFO]: %s" %cmd_str
        result=os.popen(cmd_str).read()
#         print  "Search in logcat is %s"%result
        return result
    '''
    para:the keyword used to search in logcat
    para1:the error type to search in the result of logcat_search
    '''
    def check_logcat_error(self,para,para1):
        logcat_search=self.search_in_logcat(para)
        assert logcat_search.count(para1)==0,"[Error]There is %s in the result of searching %s in logcat "%(para1,para)
        print "[Info]There is no %s in the result of searching %s in logcat "%(para1,para)
    """
    check keyword in logcat
    """
    def search_in_dmesg(self,keywords):
        cmd_str="adb -s %s shell dmesg  | grep %s"%(self.serial,keywords)
        print "[INFO]: %s" %cmd_str
        result=os.popen(cmd_str).read()
        print  "Search in dmesg is %s"%result
        return result

    """
    check keyword in logcat
    """
    def halctl_li (self,keywords):
        halctl_li_search=self.halctl_cmd('-li', keywords)
        print  halctl_li_search
        return halctl_li_search

    def halctl_i (self,keywords):
        halctl_i_search=self.halctl_cmd('-i', keywords)
        print  halctl_i_search
        return halctl_i_search
    """
    halctl -a para
    then halctl -li para
    """
    def halctl_a (self,para):
        halctl_add=self.halctl_cmd('-a', para)
        print "halctl -a %s is %s"%(para,halctl_add)
        time.sleep(3)
        halctl_i_result=self.halctl_i(para).count('Binding')
        assert halctl_i_result>0, "[ERROR]: halctl -a %s  is fail"%para
        print "[INFO]: halctl -a %s is success"%para

    """
    halctl -s para
    then halctl -li para
    """
    def halctl_s (self,para):
        i=1
        while self.halctl_li(para).count('Binding'):
            halctl_add=self.halctl_cmd('-s', para)
            print "halctl -s %s is %s"%(para,halctl_add)
            i+=1
            if i>5:
                break
        halctl_li_result=self.halctl_li(para).count('Binding')
        print "halctl_li_result is ",halctl_li_result
        assert halctl_li_result==0, "[ERROR]: halctl -s %s  is fail"%para
        print "[INFO]: halctl -s %s is success"%para

    """
    adb shell mount | grep para
    then halctl -li para
    """
    def shell_mount_grep (self,para):
        cmdstr="mount | grep %s"%para
        result=g_common_obj.adb_cmd_capture_msg(cmdstr)
        print "adb shell mount grep %s is %s"%(para,result)
        assert result>0, "[ERROR]: adb shell mount grep %s is fail"%para
        print "[INFO]: adb shell |mount grep %s is fail"%para

    """
    switch_to_multi_user
    """
    def switch_to_multi_user (self,keywords):
        halctl_li_search=self.halctl_cmd('-li', keywords)
        print  halctl_li_search
        return halctl_li_search
    
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
        time.sleep(2)
        if not self.d(text = "Users").exists:
            self.d(scrollable=True).scroll.vert.to (text = "Users")
        self.d(text = "Users").click.wait()
        time.sleep(3)
        self.del_all_usrs()
        if self.d(text = "Add user or profile").exists:
            time.sleep(2)
            self.d(text = "Add user or profile").click()
            time.sleep(2)
            self.d(text = "User").click()
            time.sleep(2)
            self.d(text = "OK").click()
            time.sleep(2)
            self.d(text = "Set up now").click()
            time.sleep(2)
            print 'start sleep 15s'
            time.sleep(15)
            print 'end sleep 15s'

    def setup_guideline_new_user(self):

        print "start setup guideline of new_user"
        self.d(resourceId="com.google.android.setupwizard:id/start").click()
        self.d(textContains="Skip").click()
        self.d(textContains="Skip anyway").click()
        self.d(text="First").click()
        time.sleep(3)
        print "input text XXXXXXXXXXXXXXXXXXXXXXXX"
        self.d(index="2").set_text('123')
        print 'press back'
        self.d.press.back()
        self.d(text="Next").click()
        self.d(text="More").click()
        self.d(text="Next").click()
        self.d(textContains="Finish").click()
        self.d(textContains="GOT IT").click()
    def switch_to_owner(self):
        '''
        switch to owner user from the new added user
        '''
        print "switch to owner user from the new added user"
        self.wait_for_android_os()
        time.sleep(2)
        self.d.wakeup()
        self.d(className="android.widget.FrameLayout",resourceId="com.android.systemui:id/multi_user_switch")
        time.sleep(2)
        self.d(className="android.widget.TextView",text="Owner")
        time.sleep(10)
        self.wake_up()
        print "wake up"
        self.launch_settings()
        self.d(text = "Users")
        time.sleep(3)
        self.del_all_usrs()
        print "del_all_usrs OK"


    def boot_safemode(self):
        '''
        boot to safemode
        '''
        print "boot to safemode: start"
        g_common_obj2.long_press_power()
        self.d(text="Power off").long_click()
        time.sleep(5)
        self.d(text="OK").click()
        print 'Waiting for device online'
        time.sleep(150)
        print "boot to safemode: end"

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

        cmd_get_pid = "adb -s %s shell ps |grep %s"%(self.serial,pid_name)
        cmd_result = os.popen(cmd_get_pid).read().split()
        if len(cmd_result) == 0:
            self.child_pid = None
        else:
            self.child_pid = int(os.popen(cmd_get_pid).read().split()[1])
        print self.child_pid
        return self.child_pid

    def kill_pid(self,pid):

        cmd_kill_pid = "adb -s %s shell kill %s"%(self.serial,pid)
        cmd_result = os.popen(cmd_kill_pid).read().split()
        print cmd_result

    def launch_settings(self):
        '''
        Launch Settings app.
        '''
        print "[Info] ---Launch Settings app."
        g_common_obj.launch_app_am("com.android.settings", ".Settings")



    def get_process_owner(self,pid_name):
        cmd_get_pid = "adb -s %s shell ps |grep %s"%(self.serial,pid_name)
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
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        self.d(textContains="Developer options").click.wait()
        self.d(className="android.view.View", resourceId="android:id/decor_content_parent") \
            .child_by_text(
            "Verify apps over USB",
            allow_scroll_search=True,
            className="android.widget.TextView").click.wait()

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

# ____________________________________________________________________________________________________
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
        dev_opt_name = "Developer options"
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings"
                  % self.serial)
        self.d(scrollable=True).scroll.toEnd()  # it's at the end

        if not self.d(textContains=dev_opt_name).exists:
            # if "Developer options" not enabled, enable it
            self.enable_developer_option()
        self.d(textContains=dev_opt_name).click.wait()

        # scroll to the option
        self.d(scrollable=True).scroll.vert.to(textContains=option_name)
        checkbox = self.d(textContains=option_name).right(
            resourceId="android:id/checkbox")
        if checkbox.checked != enable:
            checkbox.click()
        self.d.press.back()  # back to Settings
        self.d.press.back()  # exit Settings Activity

    def enable_developer_option(self):
    # enable developer option
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(scrollable=True).scroll.vert.to(textContains="About tablet")
        if self.d(textContains="Developer options").exists:
            return
        self.d(textContains="About tablet").click()
        for i in range(8):
            self.d(textContains="Build number").click()
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




    def keep_awake(self):
    # keep screen awake
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        self.d(textContains="Developer options").click.wait()
        if not self.d(text="Stay awake").right(resourceId="android:id/checkbox").checked:
            self.d(text="Stay awake").right(resourceId="android:id/checkbox").click()
        self.d.press.back()
        self.d(scrollable=True).scroll.vert.to(text="Display")
        self.d(text="Display").click.wait()
        if not self.d(text="After 30 minutes of inactivity").exists:
            self.d(text="Sleep").click.wait()
            self.d(text="30 minutes").click.wait()
        assert self.d(text="After 30 minutes of inactivity").exists

    def lock_screen(self):
        '''
        Lock screen.
        '''
        print "[Info] ---Lock screen."
        self.d.sleep()
        self.d.wakeup()
        assert self.dsystemdomainsImpl(description="Unlock").exists

    def unlock_screen(self, passwd="abcd", status=True):
        '''
        Unlock screen.
        '''
        print "[Info] ---Unlock screen."
        self.d.wakeup()
        if self.d(description="Unlock").exists:
            self.d(description="Unlock").drag.to(resourceId = "com.android.systemui:id/clock_view")
        assert not self.d(description="Unlock").exists
        if self.d(resourceId = "com.android.systemui:id/passwordEntry").exists:
            self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
            self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(passwd)
            self.d.click(700,1000)
            if status:
                assert not self.d(resourceId = "com.android.systemui:id/passwordEntry").exists
            else:
                assert self.d(resourceId = "com.android.systemui:id/passwordEntry").exists \
                        or self.d(textStartsWith = "You have incorrectly typed your password").exists



    def check_keywork_in_logcat(self):
        print "[INFO]: launch_settings is pass"



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

autodetect=AutodetectImpl(g_common_obj)
