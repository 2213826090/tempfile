#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
import sys
import time
from testlib.common.device import *

from testlib.common.common import g_common_obj2, g_logger
from testlib.common.map import DeviceMap
from testlib.common.utils import catchException

class CPSMode(object):

    def __init__(self, data):
        self._data = data

    def __getattr__(self, name):
        return self._data[name]

class CPS(object):

    __addr1_list = ["host1 host2"]
    __addr2_list = ["host1 host2", "host3"]
    __port_list = ["","host1", "host2", "host3", "host4", "host5", "host6"]
    __port_dic = {10:'A',11:'B',12:'C'}

    def __init__(self, serverIp="localhost", serverPort=11300, serverCps="cps", default_host=None):
        self.d = g_common_obj2.get_device()
        self._dsn = g_common_obj2.globalcontext.device_serial
        #self._logger = newLogfile(self._dsn)
        self.serverCmd = "%s %s %s '%%s' %s"%\
            (os.path.join(os.path.dirname(__file__),"MsgQueueProxy.py"), 
             serverIp, serverPort, serverCps)
        host = DeviceMap().getValue("cps_port")
        if host is  None:
            g_logger.warning("get cps_port from map.conf failed, use default value")
            host = default_host
        if host is None:
            g_logger.warning("get cps_port from default value failed, use 01")
            host = "01"
        self.mode = self.get_cps_mode(host)

    def __get_rc_addr1(self, host):
        #if host not in __addr1_list:
        #    raise Exception("Cant get addr1 of %s" % host)
        #return __addr1_list.index(host)+30
        return 30

    def __get_rc_addr2(self, host):
        #index = 0
        #for x in __addr2_list:
        #    if host in x:
        #        return index+30
        #    index += 1
        #raise Exception("Cant get addr2 of %s" % host)
        return 30

    def __get_port(self, host):
        return host
        loop = 0
        for x in self.__port_list:
            if host in x:
                return "0%s" % self.__port_dic.get(loop, str(loop))
            loop += 1    
        raise Exception("Can't get port of %s" % host)

#     def arp(self):
#         print("Add ARP address")
#         cmd = "arp -s %s %s" % (cfg.cps.main_ip, cfg.cps.mac_addr)
#         print(cmd)
#         os.popen(cmd)

#     def unset_proxy(self):
#         print("Unset Proxy")
#         basepath = os.path.split(os.path.realpath(__file__))[0]
#         fpath = os.path.join(basepath, "unset_proxy.sh")
#         cmd = '/bin/bash %s' % fpath
#         os.popen(cmd)

    def get_cps_mode(self, host):
        addr1 = self.__get_rc_addr1(host)
        addr2 = self.__get_rc_addr2(host)
        port = self.__get_port(host)
        return CPSMode({"addr1":addr1, "addr2":addr2, "port":port})

    def bootup(self, bootTimeout):
        """ Boot up device """
        g_logger.info("### Start to boot up device ###")
        if self.__get_device_status():
            g_logger.info("Device has already boot up")
            return
        # Press physical power button
        self.__press_power_button(4)
        # wait boot up complete
        bootup_timeout = bootTimeout
        _timeout = bootup_timeout
        msg_fail = "Boot up not complete within %d seconds" % _timeout
        for i in range(2):
            bootup_timeout = bootTimeout
            while bootup_timeout > 0:
                if self.__get_device_status():
                    # if screen lock, slide to unlock
                    self.__unlock_screen()
                    self.d.press.home()
                    self.d(description="Apps").wait.exists(timeout=10*1000)
                    assert self.d(description="Apps").exists, "[ERROR] " + msg_fail
                    g_logger.info("Boot up complete")
                    return
                g_logger.info("Boot up ...")
                time.sleep(10)
                bootup_timeout -= 10
            self.unplugUsb()
            time.sleep(1)
            self.plugUsb()
        g_logger.error( msg_fail)
        assert False, "[ERROR] " + msg_fail

    def shutdown(self, shutdownTimeout):
        """ Shut down device """
        g_logger.info("### Start to shut down device ###")
        if self.__get_device_status() is False:
            g_logger.info("Device has already shutdown")
            return
        # if device sleep, wake up device
        self.wakeup_device()
        self.__unlock_screen()
        # Press physical power button
        self.__press_power_button()
        time.sleep(5)
        assert self.__click_power_off(shutdownTimeout), "[ERROR] Shut down not complete"

    def long_press_shutdown(self):
        """ Long press power button to shutdown device """
        g_logger.info("### Start to long press power button to shutdown device ###")
        if self.__get_device_status() is False:
            g_logger.info("Device has already shutdown")
            return
        # if device sleep, wake up device
        self.wakeup_device()
        self.__long_press_power_button()
        #self.__click_power_off()

    def __tryPushScript(self):
        if g_common_obj2.adb_cmd_capture_msg("\"ls /data/local/tmp/sleep.sh > /dev/null 2>&1 ; echo $?\"").strip() == "0":
            return True
        shFile = os.path.join(os.path.dirname(__file__), "sleep.sh")
        os.system("%s push %s /data/local/tmp/"%(self.__get_adb_prefix(self._dsn), shFile))
        return True

    def sleep_device(self):
        """ sleep device
        press power button to let device enter sleep by CPS board
        """
        g_logger.info("Sleep device")
        #self.__press_power_button(mode, 0.3)
        #self.d.sleep()
        self.__tryPushScript()
        cmd = "%s shell \"/data/local/tmp/sleep.sh &\" &"%self.__get_adb_prefix(self._dsn)
        g_logger.info (cmd)
        os.system(cmd)
        time.sleep(2)
        os.system(cmd)
        time.sleep(1)
        #assert self.__get_screen_status() is False, "[ERROR] Sleep device fail"

    def get_s3_num(self):
        g_logger.info("Check device S3 Mode")
        cmd = 'cat /sys/kernel/debug/pmc_atom/sleep_state | grep S0I3'
        result = g_common_obj2.adb_cmd_capture_msg(cmd).strip()
#         msg_fail = "Get device S0I3 value fail"
#         ret = int(result_list[-1].rstrip())
#         assert ret==0, msg_fail
        g_logger.info(result)
        _match = "^S0I3\s*Residency:\s*(\d+)us"
        state_num = -1
        try:
            ret = re.search(_match, result)
            state_num = int(ret.group(1))
        except Exception:
            pass
        return state_num

    def check_s3_mode(self, num1, num2):
        """ Check device S3 Mode
        """
        assert num2>num1


    def wakeup_device(self):
        """ Wake up device
        wake up device by press power button
        """
        g_logger.info("Wake up device")
        self.d.wakeup()
        time.sleep(2)
        #assert self.__get_screen_status() is True, "[ERROR] Wake up device fail"

    def __click_power_off(self, shutdownTimeout=100):
        g_logger.info("Click 'Power off' on device screen")
        if not self.d(text="Power off").exists:
            self.__press_power_button()
            time.sleep(5)
        self.d(text="Power off").wait.exists(timeout=30*1000)
        self.d(text="Power off").click()
        time.sleep(2)
        try:
            if self.d(text="Power off", resourceId="android:id/message").exists:
                g_logger.warning("click power off fail, try again")
                self.d(text="Power off").click()
        except Exception,e:
            pass
        #self.d(text="OK").click()
        # wait device shut down complete
        shutdown_timeout = shutdownTimeout
        _timeout = shutdown_timeout
        while shutdown_timeout > 0:
            if self.__get_device_status() is False:
                time.sleep(20)
                g_logger.info("Shut down complete")
                return True
            time.sleep(3)
            shutdown_timeout -= 3
            g_logger.info("Shut down ...")
        g_logger.info("Shut down not complete within %d seconds" % _timeout)
        return False

    def __long_press_power_button(self, dsn=None, sleeptime=2):
        """
        @summary: Long press power key to power off DUT
        """
        g_logger.info("Long press power button")
        self.__press_power_button()
        time.sleep(2)

    def __unlock_screen(self):
        """ Unlock screen via swipe screen
        """
        g_common_obj2.unlock()

    def __get_screen_status(self):
        """Get screen status: on/off
        @return Ture(on), False(off)
        """
        cmd = 'dumpsys power | grep mScreenOn; echo $?'
        msg_fail = "Get screen status fail"
        res_list = g_common_obj2.adb_cmd_capture_msg(cmd).splitlines()
        screenOn = False
        if len(res_list) > 0:
            ret = int(res_list[-1].strip())
            assert ret == 0, msg_fail
            if 'true' in ''.join(res_list).lower():
                screenOn = True
        else:
            assert False, msg_fail
        g_logger.info("Screen %s" % (screenOn and "on" or "off"))
        return screenOn

    def __get_device_status(self, dsn=None):
        """ Get device status: boot complete/shut down
        @return True = boot complete, False = shut down
        """
        if not dsn:
            dsn = self._dsn
        g_logger.info("Get boot up/shut down status of device: %s" % dsn)

        adb_prefix = self.__get_adb_prefix(dsn)
        cmd = '%s get-state' % adb_prefix
        cmd2 = '%s shell getprop | grep sys.boot_completed' % adb_prefix
        res = os.popen(cmd).read()
        if 'device' in res:
            res = os.popen(cmd2).read()
            if '[1]' in res:
                return True
        return False

    def waitForStatus(self, status=True, timeout=60):
        s=time.time()
        e=s+timeout
        while time.time()<e:
            if self.__get_device_status() == status:
                break
            time.sleep(3)
        else:
            return False
        return True

    def __get_adb_prefix(self, dsn):
        if dsn:
            return 'adb -s %s' % dsn
        return 'adb'

    def pressPowerButton(self, press_time_gap=2):
        return self.__press_power_button(press_time_gap)

    def __press_power_button(self, press_time_gap=2):
        g_logger.info("Press power button")
        cmd =self.serverCmd%"cps %s %f"%(self.mode.port, press_time_gap)
        print cmd
        os.popen(cmd)
        time.sleep(press_time_gap+3)

    def unplugUsb(self):
        unplugUsb()

    def plugUsb(self):
        plugUsb()

    def checkDevicesAndTryResume(self):
        for i in range(2):
            try:
                self.d.info
                return True
            except:
                self.unplugUsb()
                time.sleep(1)
                self.plugUsb()
            time.sleep(5)
        self.__click_power_off()
        for i in range(10):
            try:
                self.d.info
                return True
            except:
                self.unplugUsb()
                time.sleep(1)
                self.plugUsb()
            time.sleep(5)
        self.d.info

    def getTmpDir(self):
        path = "/tmp"
        if not os.access(path, os.R_OK|os.W_OK):
            path = "~/tmp"
            if not os.path.exists(path):
                os.mkdir(path)
        path = os.path.join(path, "logs")
        if not os.path.exists(path):
            os.mkdir(path)
        return path

    def installReboottest(self):
        g_common_obj2.adb("install %s"%os.path.join(os.path.dirname(__file__),"ME181C_fac_Reboot.apk"))

    def startReboottest(self, rebootTimes, sleepTime):
        g_common_obj2.launchAppByName("reboot_test_v37")
        time.sleep(1)
        self.d(text="ShutDown").click()
        if not self.d(text="Start").exists:
            self.d(text="Stop").click()
        self.d(resourceId="com.asus.atd.reboot:id/EditDelay").click()
        rebootTimes = int(rebootTimes)
        sleepTime = int(sleepTime)
        allTime = "%f"%(1.0*(sleepTime*rebootTimes-sleepTime/2-10)/3600)
        g_logger.info (sleepTime*rebootTimes-sleepTime/2-10)
        g_logger.info (allTime)
        self.__clear_text(resourceId="com.asus.atd.reboot:id/EditDelay")
        self.d(resourceId="com.asus.atd.reboot:id/EditDelay").set_text("10")
        self.d(resourceId="com.asus.atd.reboot:id/EditCountdownTimer").click()
        self.__clear_text(resourceId="com.asus.atd.reboot:id/EditCountdownTimer")
        self.d(resourceId="com.asus.atd.reboot:id/EditCountdownTimer").set_text(allTime)
        self.d(text="Start").click()

    def startS3Sleep(self, sleepTime):
        g_common_obj2.launchAppByName("reboot_test_v37")
        time.sleep(1)
        self.d(text="Sleep/Wakeup").click()
        if not self.d(text="Start").exists:
            self.d(text="Stop").click()
        self.d(resourceId="com.asus.atd.reboot:id/EditDelay").click()
        rebootTimes = 2
        #sleepTime = 15
        allTime = "%f"%(1.0*(sleepTime*rebootTimes-sleepTime/2-5)/3600)
        g_logger.info (sleepTime*rebootTimes-sleepTime/2-10)
        g_logger.info (allTime)
        self.__clear_text(resourceId="com.asus.atd.reboot:id/EditDelay")
        self.d(resourceId="com.asus.atd.reboot:id/EditDelay").set_text(str(sleepTime))
        self.d(resourceId="com.asus.atd.reboot:id/EditCountdownTimer").click()
        self.__clear_text(resourceId="com.asus.atd.reboot:id/EditCountdownTimer")
        self.d(resourceId="com.asus.atd.reboot:id/EditCountdownTimer").set_text(allTime)
        self.d(text="Start").click()

    def __clear_text(self, *args, **kwargs):
        #while self.d(*args, **kwargs).info[u"text"] != u"":
        for i in range(20):
            #back
            self.d.press(67)
            #del
            self.d.press(112)

    def endReboottest(self):
        assert self.d(text="PASS").exists

    def coldBoot(self, timeout=60):
        try:
            #self.unplugUsb()
            #time.sleep(15)
            self.pressPowerButton(5)
            time.sleep(10)
        finally:
            #self.plugUsb()
            pass
        g_common_obj2.waitForDeviceListInDevices(timeout)
        #time.sleep(10)
        assert self.waitForStatus(True, timeout), "boot up failed, couldn't get device in adb devices"

    def bootIfPowerOff(self,timeout=60):
        if not self.__get_device_status():
            g_logger.info("devie is power off, try cold boot")
            self.coldBoot(timeout)
        else:
            g_logger.info("devie is power on")
