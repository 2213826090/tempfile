import sys
import os
import time
import ConfigParser
import cmd
import serial
import unittest
from shutil import copyfile

from testlib.util.common import g_common_obj
from testlib.util.log import Logger
g_logger = Logger.getlogger()

def getConfValue(confFile, sectionName, optionName):
    try:
        cf = ConfigParser.ConfigParser()
        cf.read(confFile)
        return cf.get(sectionName, optionName)
    except:
        #raise
        return None

def reportSemiAutoVerdict():
    raise unittest.TestCase.failureException("{INCONCLUSIVE}")

def uploadSemiAutoLogFile(testcase, src):
    # Default setting and paramters handling 
    filename = src.split("/")[-1]
    mount_point = "/aftlog"
    checkfile = "checkfile"
    nfs_server = "prdshtwsv2d01.sh.intel.com"
    nfs_point = "/aftlog"

    # Get nfs config from sys.conf
    opt_nfs_server = getConfValue("/etc/oat/sys.conf", "aftlog", "nfs_server")
    if opt_nfs_server:
        nfs_server = opt_nfs_server
    opt_nfs_point = getConfValue("/etc/oat/sys.conf", "aftlog", "nfs_point")
    if opt_nfs_point:
        nfs_point = opt_nfs_point

    # Check mount point, if not create folder
    if not os.path.exists(mount_point):
        os.mkdir(mount_point)
    
    # Check nfs mount, if not mount the nfs drive
    if not os.path.exists(mount_point + "/" + checkfile):
        os.system("sudo mount -t nfs {nfs_server}:{nfs_point} {mount_point}".format(nfs_server=nfs_server, nfs_point=nfs_point, mount_point=mount_point))
    
    # Get ACS folder name, so that make the folder name align with ACS campaign folder name
    target_path = testcase._acs_params["report_path"].split("/")[-1] + "/" + testcase._testMethodName + "/"
    if not os.path.exists(mount_point + "/" + target_path):
        os.makedirs(mount_point + "/" + target_path)
    
    # Upload file
    copyfile(src, "{0}/{1}/{2}".format(mount_point, target_path, filename))
    
    # Print out the http address for users
    g_logger.info("[SEMI-AUTO-LOG-PATH] http://{0}{1}/{2}{3}".format(nfs_server, nfs_point, target_path, filename))


class Common2(object):
    """
    Common class is a lightweight wrapper to make using test case 
    common utilities eaiser.

    """

    def __init__(self, obj):
        self.__obj = obj

    def __getattr__(self, attr):
        return getattr(self.__obj, attr)

    def getprop(self, context=""):
        return self.adb_cmd_capture_msg("getprop %s"%context).strip()

    def getDeviceVersion(self):
        if not hasattr(self, "device_version"):
            self.device_version = self.getprop("ro.build.version.release")
        return self.device_version

    def getConfValue(self, secName, optName):
        confFile = os.path.join(os.path.dirname(__file__), 'common.conf')
        result = getConfValue(confFile, secName+"_"+self.getDeviceVersion(), optName)
        #print secName+"_"+self.getDeviceVersion(), result
        if result is None:
            result = getConfValue(confFile, secName, optName)
        #print result
        return result

    def getConfKey(self, name):
        return name.replace(" ", "")

    def getPackageActiveByName(self, appName):
        appConfName = self.getConfKey(appName)
        amstart = self.getConfValue("amstart_conf", appConfName)
        return amstart

    def launchAppByName(self, appName):
        amstart = self.getPackageActiveByName(appName)
        if amstart:
            return self.adb_cmd("am start -S %s"%amstart)
        else:
            self.get_device().press.home()
            return self.launch_app_from_home_sc(appName)

    def stopAppByName(self, appName):
        amstart = self.getConfValue("amstart_conf", self.getConfKey(appName))
        if amstart:
            pkgName = amstart.split("/")[0]
            return self.stop_app_am(pkgName)
        else:
            self.get_device().press.home()

    def back_to_home_screen(self,num=20):
        d = self.get_device()
        loop_num = 0 
        for i in range(num):
            d.press.back()
            loop_num = loop_num + 1
            if d(description = "Apps",className = "android.widget.TextView").exists:
                break
        if loop_num >=19:
            assert False,"can't back to home screen"
        d.press.home()

    def adb(self, cmdstr):
        cmd = "adb %s %s"
        deviceId = "-s %s"%self.globalcontext.device_serial if self.globalcontext.device_serial else ""
        return os.system(cmd%(deviceId, cmdstr))

    def adb_message(self, cmdstr):
        cmd = "adb %s %s"
        deviceId = "-s %s"%self.globalcontext.device_serial if self.globalcontext.device_serial else ""
        return os.popen(cmd%(deviceId, cmdstr)).read()

    def unlock(self, raiseError=False):
        d = self.get_device()
        d.wakeup()
        time.sleep(0.5)
        if d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            d(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.right()
        if d(resourceId="com.android.systemui:id/lock_icon").exists:
            w=d.info[u'displayWidth']
            h=d.info[u'displayHeight']
            d.swipe(w/2,h,w/2,0)

    def check_device_connectivity(self,  retry=10):
        """Check device connectivity
        use adb get-state to check device connection status
        @parameter: device_dsn (string)
            device device dsn
        @parameter: retry (int)
            retry times to get-state of device, default 10
        """
        device_dsn = self.globalcontext.device_serial
        print "Check device:%s connectivity" % device_dsn
        cmd = "adb -s %s get-state" % device_dsn
        for _ in range(0, retry):
            pipe = os.popen(cmd)
            device_status = pipe.read().rstrip()
            if device_status == "device":
                return True
            else:
                time.sleep(5)
        return False

    def get_device_dsn_state(self):
        """ Get device dsn and state
        @return (tuple)
            (devicedsn, state) if only one device connected),
            (None, None) if no device or more than one devices connected)
        """
        device_serial = self.globalcontext.device_serial
        print "Get device dsn and state"
        lines = self.adb_message("devices").splitlines()
        print "[INFO] lines", lines
        lines = [x.strip() for x in lines]
        if lines[-1] == '':
            del lines[-1]
            if lines[-1] == '':
                del lines[-1]
            device_cnt = len(lines)
            for line in lines:
                if line.find("List of devices attached") != -1:
                    device_cnt = device_cnt -1
                    continue
                if device_cnt == 0:
                    print "No device attached"
                    return None, None
                elif device_cnt == 1:
                    device_dsn, state = [x.strip() for x in line.split()]
                    print "Device [%s] state: %s" % (device_dsn, state)
                    return device_dsn, state
                else:
                    if device_serial:
                        lists = [x.strip() for x in lines.split('\n')]
                        for l in lists:
                            device_dsn, state = [x.strip() for x in l.split()]
                            if device_serial == device_dsn:
                                return device_dsn, state
                            else:
                                device_dsn = None
                                state = None
                    else:
                        print "More than one device attached"
                        return None, None
        return None, None

    def reboot(self, waittime):
        """ Reboot device
        reboot device by 'adb reboot', check device startup normally after reboot.
        @parameter: waittime (int)
            seconds to wait reboot complete
        """
        print "Start reboot device"
        device_dsn = g_common_obj.globalcontext.device_serial
        cmd_reboot = 'reboot 2>&1; echo $?'
        ret = self.adb(cmd_reboot)
        suc_reboot = (ret == 0)
        assert suc_reboot, "Run 'adb reboot' fail"

        #check device connect status after reboot
        while waittime >= 0:
            time.sleep(20)
            if device_dsn is None:
                _, state = self.get_device_dsn_state()
                if state == "device":
                    break
            else:
                if self.check_device_connectivity():
                    break
            time.sleep(10)
            waittime -= 10

        msg_fail = "After reboot, device connection state incorrect"
        if device_dsn is None:
            _, state = self.get_device_dsn_state()
            print state
            assert state == "device", msg_fail
        else:
            assert self.check_device_connectivity(), msg_fail
        #check device startup normally after reboot by run a simple adb shell
        msg_fail = "After reboot, device startup fail"
        cmd = 'ls /; echo $?'
        res_list = os.popen(cmd).readlines()
        if len(res_list) > 0:
            ret = int(res_list[-1].strip())
            assert ret == 0, msg_fail
        else:
            assert False, msg_fail
        print "Reboot device success"

    def system_reboot(self,delaytime=200):
        """ Reboot device (solve the problem:"RPC server not started!")
        reboot device by 'adb reboot', check device startup normally after reboot.
        @parameter: waittime (int)
            seconds to wait reboot complete
        @author:yuhui xu
        @date: 11/10/2014
        """
        self.serial = g_common_obj.globalcontext.device_serial
        print "the self.serial is: ",self.serial
        adbcmdstr = None
        if (self.serial):
            adbcmdstr = "adb -s %s reboot" % (self.serial)
        else:
            adbcmdstr = "adb reboot"
        print'Execute adb reboot'
        ret= self.shell_cmd(adbcmdstr)
        print ret
        if (self.serial):
            adbcmdstr = "adb -s %s wait-for-device" % (self.serial)
        else:
            adbcmdstr = "adb wait-for-device"
        print 'Waiting for device online'
        ret = self.shell_cmd(adbcmdstr,delaytime + 10)
        print "This is to root device for RPC not start problem"
        print "wait for boot :200s start"
        time.sleep(delaytime)
        print "wait for boot :200s end"
        g_common_obj.root_on_device()
        print "Reboot device success"
        return ret
    def long_press_power(self):
        """
        @summary: Long press power key to power off DUT
        @guimei1x
        @6/16/2015
        """
        print 'long press power key for 2s :start'
        d = self.get_device()
        serial = g_common_obj2.getSerialNumber()
        for i in range(2):
            cmd_press = "adb -s %s shell sendevent /dev/input/event%d 0001 116 1"% (serial,i)
            print cmd_press
            os.system(cmd_press)
            cmd_press = "adb -s %s shell sendevent /dev/input/event%d 0000 0000 0000"% (serial,i)
            print cmd_press
            os.system(cmd_press)
            time.sleep(3)
            if d(textContains="Power off").exists:
                break
        assert d(textContains="Power off").exists, "Long press power failed!"
        print 'long press power key for 2s :end'

    def getSerialNumber(self):
        device_dsn = g_common_obj.globalcontext.device_serial
        if not device_dsn:
            device_dsn = os.environ.get("ANDROID_SERIAL", None)
        if not device_dsn:
            device_dsn = os.popen("""adb devices | awk '{if($2 == "device")print $1}' | head -1""").read().strip()
        return device_dsn

    def checkDevices(self):
        dsn = self.getSerialNumber().strip()
        if dsn == "":
            return False
        return os.system("adb devices | grep %s 1>/dev/null"%dsn) == 0

    def waitForDeviceListInDevices(self, timeout=30):
        s=time.time()
        while time.time() - s < timeout:
            if self.checkDevices():
                return True
        return False

    def getMd5(self, path):
        md5 = self.adb_cmd_capture_msg("md5 %s"%path)
        if md5:
            return md5.split()[0]
        return None

    def getAndroidVersion(self):
        try:
            serial = g_common_obj2.getSerialNumber()
            cmd = "adb -s %s shell getprop ro.build.version.release"% serial
            r=os.popen(cmd)
            text = r.read().strip()
            r.close()
        except Exception as e:
            print e
        if text == "":
            assert False,"can't get version"
        textlist = text.split(".")
        if (textlist[0]=="6" or textlist[0]=="M"):
            print "Image type is M"
            ImgType="M"
        elif (textlist[0]=="5"):
            print "Image type is L"
            ImgType="L"
        else:
            print "Image type is L"
            ImgType="L"
        return ImgType

g_common_obj2 = Common2(g_common_obj)
