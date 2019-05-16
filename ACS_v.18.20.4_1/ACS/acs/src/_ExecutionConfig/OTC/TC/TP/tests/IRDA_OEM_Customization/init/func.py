#!/usr/bin/env python
#-*- encoding: utf-8 -*-

import os
import time
import shutil
import commands
from uiautomator import Device
from tools import DeviceHandle
from tools import ConfigHandle
from testlib.util.repo import Artifactory
from testlib.util.config import TestConfig
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
import sys
from macpath import realpath
from multiprocessing import Process
from multiprocessing import Pipe
from multiprocessing import Value

class OEMFunc():
    def __init__(self, serial):
        self.serial = serial
        self.config = TestConfig()
        self.version = self.getVersion()
        self.cur_path = os.path.split(os.path.realpath(__file__))[0]
        #self.usr_path = (os.sep).join(self.cur_path.split(os.sep)[:3])
        self.usr_path = os.path.expanduser("~")
        self.file_path = os.path.join(self.cur_path, "file")
        self.push_file = {
            "CustomChoiceList.apk": "/oem/app/",
            "bootanimation.zip": "/oem/media/",
            "default_wallpaper.jpg": "/oem/media/",
        }
        self.l_only = {
            "oem.prop": "/oem/"
        }
        self.origin_wall = "default_wallpaper.jpg"
        self.reset_wall = "default_wallpaper2.jpg"

        self.settings_package = "com.android.settings"
        self.settings_activity = ".Settings"

    @classmethod
    def download_oem_content(self):
        url = ConfigHandle().read_content_url()
        host_file_path=ConfigHandle().read_host_file_path()
        #cmd = "wget -c " + url
        cmd = "mkdir %s;cd %s;wget -c %s"% (host_file_path, host_file_path, url)
        print cmd
        os.system(cmd)
        tmp_file=host_file_path+"/oem_content.zip"
        assert os.path.isfile(tmp_file)
        cmd2="unzip -n %s/oem_content.zip -d %s" %(host_file_path, host_file_path)
        print "[INFO]: aaaa %s" %cmd2
        os.system(cmd2)

    def download_oem_content_artifactory(self, sub_path=None, download_file=None):
        if not os.path.isfile("/etc/oat/sys.conf"):
            #real_path = os.path.realpath(__file__)
        
            cmd = "cp acs_test_suites/OTC/TC/TP/support/prebuild/sys.conf /etc/oat/"
            os.system(cmd)
        common_url = self.config.read(section='artifactory').get("location")
        remote_server = Artifactory(common_url + sub_path)
        for _ in range(5):
            return_file = remote_server.get(download_file)
            print return_file
            if return_file is not None and os.path.isfile(return_file):
                break
        time.sleep(2)
        host_file_path=ConfigHandle().read_host_file_path()
        cmd = "mkdir %s"% (host_file_path)
        os.system(cmd)
        cmd2 = "cp %s %s"%(return_file, host_file_path)
        print cmd2
        os.system(cmd2)
        tmp_file=host_file_path+"/OEM_file.zip"
        assert os.path.isfile(tmp_file)
        cmd2="unzip -n %s/OEM_file.zip -d %s" %(host_file_path, host_file_path)
        print "[INFO]: aaaa %s" %cmd2
        os.system(cmd2)

    def download_file_from_artifactory(self, sub_path=None, download_file=None):
        """
        download file from artifcatory server
        :param sub_path: sub location for download file in server
        :param download_file: file to download
        :return: local location of downloaded file or None
        """
        if os.path.isfile("/etc/oat/sys.conf"):
            common_url = self.config.read(section='artifactory').get("location")
            remote_server = Artifactory(common_url + sub_path)
            for _ in range(5):
                return_file = remote_server.get(download_file)
                if return_file is not None and os.path.isfile(return_file):
                    return return_file
                time.sleep(2)
        # try to download file from mirror server
        mirror_url = self.config.read('tests.tablet.dut_init.conf', 'download_server').get('mirror_url')
        mirror_server = Artifactory(mirror_url + sub_path)
        for _ in range(5):
            return_file = mirror_server.get(download_file)
            if return_file is not None and os.path.isfile(return_file):
                return return_file
            time.sleep(2)
        return None

    def modify_adbkey(self):
        print "[WARN]: This step will modify the adbkey file, which may impact the testing"
        key_path = self.usr_path + os.sep + ".android"
        file_list = os.listdir(key_path)
        new_name = "adbkey"
        backup_name = "adbkey.backup"
        if backup_name in file_list:
            print "Has been modified, no need to do"
        else:
            if new_name in file_list:
                os.rename(key_path + os.sep + new_name, key_path + os.sep + backup_name)
            shutil.copy(self.file_path + os.sep + new_name, key_path + os.sep + new_name)
            os.system("adb kill-server")
            os.system("adb start-server")

    def shell_command(self, shell_cmd):
        """Run adb shell command"""
        cmd = "adb -s %s shell %s" % (self.serial, shell_cmd)
        r_code, r_mes = commands.getstatusoutput(cmd)
        return r_code, r_mes

    def push_command(self, old_path, new_path, check_flag=True):
        """Run adb push command"""
        cmd = "adb -s %s push %s %s" % (self.serial, old_path, new_path)
        r_code, r_mes = commands.getstatusoutput(cmd)
        folder_list = self.check_folder("ls %s" % new_path)
        if check_flag:
            print "------------------"
            print os.path.basename(old_path)
            print old_path
            print folder_list
            assert os.path.basename(old_path) in folder_list

    def getVersion(self):
        """Get the image is L or KK"""
        cmd = "getprop ro.build.version.sdk"
        r_code, r_mes = self.shell_command(cmd)
        result = "kk"
        if r_mes.find("21") != -1 or r_mes.find("22") != -1:
            result = "l"
        print "[INFO]: Image Version: %s" % result
        return result

    def root_device(self):
        """Root the device"""
        print "[INFO]: Root device"
        os.system("adb -s %s root" % self.serial)
        time.sleep(3)
        result = os.popen("adb -s %s root" % self.serial).read()
        if " is already running as root" in result:
            print "root succeed"
        else:
            assert False, "the DUT can't root"

    def setup_connection(self):
        print "[INFO]: Set up connection"
        d = Device(self.serial)
        for i in range(3):
            if d.server.alive:
                break
            else:
                d.server.stop()
                d.server.start()
        if d.server.alive:
            print "[INFO]: Success"
        else:
            assert False, "[INFO]: Fail"

    def unlock_screen(self):
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

    def launch(self, packageName, activityName):
        d = Device(self.serial)
        d.press.back()
        d.press.back()
        d.press.home()
        cmdStr = "am start -S -n %s/%s" % (packageName, activityName)
        self.shell_command(cmdStr)
        time.sleep(10)


    def factory_reset_local(self):
        print "[INFO]: Click factory reset"
        d = Device(self.serial)
        self.launch(self.settings_package, self.settings_activity)
        time.sleep(2)
        """
        if d(scrollable=True).exists:
            d(scrollable=True).scroll.toBeginning()
        d(text="备份和重置").click()
        """
        d(className="android.widget.ScrollView", resourceId="com.android.settings:id/dashboard") \
            .child_by_text(
            "备份和重置",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        time.sleep(2)
        d(text="恢复出厂设置").click()
        time.sleep(2)
        if d(text="恢复平板电脑出厂设置").exists:
            d(text="恢复平板电脑出厂设置").click()
        if d(text="恢复手机出厂设置").exists:
            d(text="恢复手机出厂设置").click()
        time.sleep(2)
        d(text="清除全部内容").click()
        time.sleep(2)

    def factory_reset(self):
        print "[INFO]: Click factory reset"
        d = Device(self.serial)
        self.launch(self.settings_package, self.settings_activity)
        time.sleep(2)
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
        d(className="android.widget.ListView", resourceId="android:id/list") \
            .child_by_text(
            "Factory data reset",
            allow_scroll_search=True,
            className="android.widget.TextView").click()
        time.sleep(2)
        if d(text="Reset tablet").exists:
            d(text="Reset tablet").click()
        if d(text="Reset phone").exists:
            d(text="Reset phone").click()
        time.sleep(2)
        d(text="Erase everything").click()
        time.sleep(2)
    def skip_initial_screen_after_factory_reset(self):
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
        time.sleep(7)
        if self.d(text="Google services").exists:
            self.d.press.back()
            time.sleep(3)
        while self.d(text="OK").exists:
            time.sleep(3)
            self.d(text="OK").click.wait()
        time.sleep(5)
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        assert self.d(description="Apps").exists and not self.d(text="GOT IT").exists
    def check_reset(self):
        print "[INFO]: Check factory resetl"
        reset = False
        boot = False
        for i in range(60):
            devices = DeviceHandle().getAllDevices()
            if self.serial not in devices:
                reset = True
                break
            time.sleep(10)
        if not reset:
            print "[INFO]: Factory reset not work, please recheck it!"
            sys.exit(1)
        else:
            print "[INFO]: Factory reset work"
            for i in range(90):
                devices = DeviceHandle().getAllDevices()
                if self.serial in devices:
                    boot = True
                    break
                time.sleep(10)

            if boot:
                print "[INFO]: Success to reboot after factory reset"
            else:
                print "[INFO]: Fail to reboot after factory reset"
                sys.exit(1)

    def check_folder(self, cmd):
        c_code, c_mes = self.shell_command(cmd)
        folder_list = c_mes.replace("\r", "").split("\n")
        return folder_list

    def mount_device(self):
        """Mount device and delete file under /oem/meida and /oem/app"""
        print "[INFO]: Mount device"
        mount_cmd = "mount -o remount,rw /oem"
        self.shell_command(mount_cmd)

    def remove_folder(self):
        print "[INFO]: Remove app and media folder in /oem"
        folder_list = self.check_folder("ls /oem")
        remove_cmd = "rm -rf /oem/%s/*"
        folder1 = "app"
        folder2 = "media"
        if folder1 in folder_list:
            print "[INFO]: Remove /oem/app folder"
            self.shell_command(remove_cmd % folder1)
            c_code, c_mes = self.shell_command("ls /oem/%s" % folder1)
            assert len(c_mes) == 0
        if folder2 in folder_list:
            print "[INFO]: Remove /oem/media folder"
            self.shell_command(remove_cmd % folder2)
            c_code, c_mes = self.shell_command("ls /oem/%s" % folder2)
            assert len(c_mes) == 0

    def make_folder(self):
        """Create folder in /oem"""
        print "[INFO]: Create folder"
        cmd1 = "mkdir -p /oem/app"
        cmd2 = "mkdir -p /oem/media"
        for i in [cmd1, cmd2]:
            self.shell_command(i)
        folder_list = self.check_folder("ls /oem")

        assert "app" in folder_list
        assert "media" in folder_list

    def push_oem_file(self):
        """Push file to device"""
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

    def push_prop(self):
        """Push oem.prop to l image"""
        self.file_path=ConfigHandle().read_host_file_path()
        if self.version == "l":
            print "[INFO]: Push l only file to device"
            for new_file in self.l_only:
                old_file = self.file_path + "/file/" + new_file
                device_path = self.l_only[new_file]
                print "[INFO]: Push %s to %s" % (new_file, device_path)
                self.push_command(old_file, device_path)
            cmd = "chmod 600 /oem/oem.prop"
            self.shell_command(cmd)

    def reset_wallpaper(self):
        """Reset the wall paper"""
        wall_path = self.push_file[self.origin_wall]
        folder_list = self.check_folder("ls %s" % wall_path)
        if self.origin_wall in folder_list:
            print "[INFO]: Remove %s from %s" % (self.origin_wall, wall_path)
            self.shell_command("rm -rf %s" % (wall_path + self.origin_wall))
            new_list = self.check_folder("ls %s" % wall_path)
            print new_list
            print self.origin_wall
            assert (self.origin_wall not in new_list)
        print "[INFO]: Push new wall paper to %s" % wall_path
        old_file = self.file_path + os.sep + self.reset_wall
        new_file = wall_path + self.origin_wall
        self.push_command(old_file, new_file, check_flag=False)
        folder_list = self.check_folder("ls %s" % wall_path)
        assert self.origin_wall in folder_list
        print "[INFO]: Reboot device, please check it after reboot finished"
        os.system("adb reboot")
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
    
    def keep_awake(self):
    # keep screen awake
        sn=self.serial
        d = Device(self.serial)
        if os.system("adb -s %s shell dumpsys power | grep 'mStayOn=true'" % self.serial) == 0:
            if os.system("adb -s %s shell dumpsys power | grep 'mScreenOffTimeoutSetting=1800000'" % self.serial) == 0:
                return
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        d(scrollable=True).scroll.vert.to(textContains="Developer options")
        d(textContains="Developer options").click.wait()
        if d(text="Stay awake").right(resourceId="android:id/checkbox") != None:
            if not d(text="Stay awake").right(resourceId="android:id/checkbox").checked:
                d(text="Stay awake").right(resourceId="android:id/checkbox").click()
        else:
            if not d(text="Stay awake").right(resourceId="android:id/switchWidget").checked:
                d(text="Stay awake").right(resourceId="android:id/switchWidget").click()
        d.press.back()
        d(scrollable=True).scroll.vert.to(text="Display")
        d(text="Display").click.wait()
        if not d(text="After 30 minutes of inactivity").exists:
            d(text="Sleep").click.wait()
            d(text="30 minutes").click.wait()
        assert d(text="After 30 minutes of inactivity").exists

    def close_lock_screen(self):
    # delete clock screen
        sn=self.serial
        d = Device(self.serial)
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        d(text="Security").click()
        if d(text="Screen lock").down(text="None") != None:
            return
        d(text="Screen lock").click()
        d(text="None").click()
        assert d(text="Screen lock").down(text="None") != None

    def enable_developer_option(self):
        # enable developer option
        d=Device(self.serial)
        os.system("adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        d(scrollable=True).scroll.vert.to(textContains="About tablet")
        if d(textContains="Developer options").exists:
            return
        d(textContains="About tablet").click()
        for i in range(8):
            d(textContains="Build number").click()
        d.press.back()
        d(scrollable=True).scroll.vert.to(textContains="Developer options")
        assert d(textContains="Developer options").exists

if __name__ == "__main__":
    OEMFunc("set")
