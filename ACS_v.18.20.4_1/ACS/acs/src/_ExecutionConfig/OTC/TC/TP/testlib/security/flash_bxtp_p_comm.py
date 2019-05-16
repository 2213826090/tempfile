#! /usr/bin/env python
# -*- coding: utf-8 -*-

import json
import zipfile
import os
import sys
import time
import serial
import commands
from uiautomator import Device
import threading
from subprocess import Popen

FLASHLOCAL = "FLASHLOCAL"

class Bxtp:
    def __init__(self, *args):
        if len(args) < 2:
            print "args not correct"
            return
        self._flash_ioc_cmd = "/opt/intel/platformflashtool/bin/ioc_flash_server_app -s {port}  -grfabd -t ioc_firmware_gp_mrb_fab_d_slcan.ias_ioc"
        self._flash_ifwi_cmd = "/opt/intel/platformflashtool/bin/ias-spi-programmer --adapter {adapter_num} --write ifwi_gr_mrb_b1.bin"
        self._configuration = "blank_gr_mrb_b1"
        self._release_port_cmd = "sudo fuser -k {port}"
        self._get_port_order_cmd = "ls -t"
        self._debugCard_port = "ls /dev/ttyUSB*_debugCard_"
        self._serials = args[0].strip().split(",")
        self._serial_port = self._get_port()
        self._serial_port_adapter_num = []
        self._get_adapter_num()
        self._image_zip = args[1]
        self._CTS_user_image_zip = None
        if len(args) == 3:
            self._CTS_user_image_zip = args[2]
        self._get_image_dir()
        self._unzip()
        self._check_flash_json()
        self._flash_image_cmd = None
        self._flash_CTS_user_image_cmd = None
        self._get_flash_cmd()
        self._mutex = threading.Lock()
        self.flash_success = {}

    def _get_port(self):
        d = {}
        for serial in self._serials:
            #if os.getenv(FLASHLOCAL) is None:
            cmd = self._debugCard_port + serial
            #port = os.popen(cmd).read().strip().split("\n")[-2]
            info = os.popen(cmd).read().strip()
            if info == '':
                cmd_local = "ls /dev/ttyUSB*"
                port = os.popen(cmd_local).read().strip().split("\n")[-2]
            else:
                port = os.popen(cmd).read().strip().split("\n")[-2]
            d[serial] = port
        print d
        return d

    @staticmethod
    def shell_cmd_timeout(cmd, timeout=0, cwd=None, env=None):
        """Execute shell command with timeout specified
        """
        cmd_open = Popen(cmd, shell=True, cwd=cwd, env=env)
        if not cmd_open:
            return -1
        t_timeout = timeout
        tick = 3
        ret = None
        while True:
            time.sleep(tick)
            ret = cmd_open.poll()
            if ret is not None:
                break
            if t_timeout > 0:
                t_timeout -= tick
            if t_timeout <= 0:
                # timeout, kill command
                try:
                    cmd_open.kill()
                    cmd_open.wait()
                except OSError:
                    pass
                ret = -99999
                break
        return ret

    def _unzip(self):
        for zip_file in [self._image_zip, self._CTS_user_image_zip]:
            if zip_file:
                print zip_file
                dest_dir = os.path.splitext(os.path.abspath(zip_file))[0]
                print dest_dir
                if not os.path.exists(dest_dir):
                    os.mkdir(dest_dir)
                print "----start unzip {} to {}".format(zip_file, dest_dir)
                zfile = zipfile.ZipFile(zip_file, 'r')
                zfile.extractall(dest_dir)
                print "----finished unzip {} to {}".format(zip_file, dest_dir)

    def _get_image_dir(self):
        self._image_dir = os.path.splitext(os.path.abspath(self._image_zip))[0]
        if self._CTS_user_image_zip:
            self._CTS_user_image_dir = os.path.splitext(os.path.abspath(self._CTS_user_image_zip))[0]

    def _check_flash_json(self):
        for zip_file in [self._image_zip, self._CTS_user_image_zip]:
            if zip_file:
                image_dir = os.path.splitext(os.path.abspath(zip_file))[0]
                flash_json_file = os.path.join(image_dir, "flash.json")
                if not os.path.exists(flash_json_file):
                    sys.exit("Sorry, can't find 'flash.json' in {}".format(zip_file))

    def _get_blank_flash_cmd(self, image_zip):
        image_dir = os.path.splitext(os.path.abspath(image_zip))[0]
        flash_json_file = os.path.join(image_dir, "flash.json")
        flash_dict = json.load(open(flash_json_file))
        commands = flash_dict["flash"]["commands"]
        configuration_dict = flash_dict["flash"]["configurations"][self._configuration]
        parameters = flash_dict["flash"]["parameters"]
        cmd = []
        for command in commands:
            if self._configuration in command["restrict"] and command["mandatory"]:
                if "${" in command["args"] and "}" in command["args"]:
                    var = command["args"].split("${")[-1].split("}")[0]
                    if 'value' in parameters[var]:
                        value = parameters[var]['value']
                    elif 'options' in parameters[var]:
                        value = parameters[var]['options'][configuration_dict["parameters"][var]]['value']
                    args = command["args"].replace('${'+var+'}', value)
                else:
                    args = command["args"]
                flash_cmd = command["tool"] + ' -s {serial}' + ' ' + args
                cmd.append(flash_cmd)
        cmd.remove("fastboot -s {serial} oem rm /ESP/BIOSUPDATE.FV")
        return cmd

    def _get_flash_cmd(self):
        self._flash_image_cmd = self._get_blank_flash_cmd(self._image_zip)
        if self._CTS_user_image_zip:
            user_cmd_list = self._get_blank_flash_cmd(self._CTS_user_image_zip)
            user_cmd_list.remove("fastboot -s {serial} erase persistent")
            user_cmd_list.remove("fastboot -s {serial} erase metadata")
            user_cmd_list.remove("fastboot -s {serial} format data")
            self._flash_CTS_user_image_cmd = user_cmd_list

    def _get_adapter_num(self):
        for port in self._serial_port.values():
            self._get_port_order_cmd += " "
            self._get_port_order_cmd += port
        port_time_order_list = os.popen(self._get_port_order_cmd).read().strip().split("\n")
        for serial, port in self._serial_port.items():
            serial_port_adapter_num_dict = {"serial":serial, "port":port}
            serial_port_adapter_num_dict["adapter_num"] = port_time_order_list.index(port)
            self._serial_port_adapter_num.append(serial_port_adapter_num_dict)

    @staticmethod
    def wait_device(cmd, serial=None, timeout=60):
        cmd = cmd.format(serial)
        while timeout:
            process = os.popen(cmd)
            out_put = process.read()
            print '----wait device output'
            print out_put
            if out_put:
                return True
            else:
                time.sleep(2)
                timeout -= 2
                print '----wait device {serial} {timeout} ...'.format(serial=serial, timeout=timeout)
                if timeout <= 0:
                    return False

    def goto_mode(self, mode=None, port=None):
        '''

        Args:
            mode: n4#   fastboot  use this mode to flash IFWI, Image
                  r     shutdown
                  2l4\r debug     use this mode to flash IOC
            port: the 3rd port of debug card, such as "dev/ttyUSB2"

        Returns: None

        '''
        os.popen(self._release_port_cmd.format(port=port))
        usb2SerialConnection = serial.Serial(port=port, baudrate=115200, timeout=0)
        usb2SerialConnection.write("r")
        time.sleep(2)
        usb2SerialConnection.write(mode)
        time.sleep(2)
        usb2SerialConnection.close()
        time.sleep(2)

    def goto_fastboot(self, **kwargs):
        self.goto_mode(mode="r", port=kwargs["port"])
        time.sleep(2)
        self.goto_mode(mode="n2#", port=kwargs["port"])
        if self.wait_device('fastboot -s {} devices', serial=kwargs["serial"]):
            print "----success goto fastboot {}".format(kwargs["serial"])
            time.sleep(2)
            return True
        else:
            self.goto_mode(mode="g", port=kwargs["port"])
            self.shell_cmd_timeout("adb -s {} wait-for-device".format(kwargs["serial"]))
            self.shell_cmd_timeout("adb -s {} reboot bootloader".format(kwargs["serial"]))
            if self.wait_device('fastboot -s %s devices', serial=kwargs["serial"]):
                print "----success goto fastboot {}".format(kwargs["serial"])
                time.sleep(2)
                return True
            else:
                print "----fail goto fastboot {}".format(kwargs["serial"])
                return False
                # sys.exit("goto fastboot fail")

    def set_verdict(self, serial, true_false):
        # true_false: can be True or False
        if self._mutex.acquire():
            self.flash_success[serial] = true_false
            self._mutex.release()

    def flash_ioc(self, **kwargs):
        # go to debug mode to flash ioc, device go to closed mode after flash ifwi
        for i in range(3):
            self.goto_mode(mode="2l4\r", port=kwargs["port"])
            print "\n----start to flash IOC on {}".format(kwargs["serial"])
            flash_log = os.popen(self._flash_ioc_cmd.format(port=kwargs["port"])).read()
            if "success" in flash_log:
                print "----IOC flash success on {}".format(kwargs["serial"])
                self.set_verdict(kwargs["serial"], True)
                return True
            else:
                print flash_log
                self.set_verdict(kwargs["serial"], False)
        return False

    def flash_ifwi(self, **kwargs):
        # go to fastboot mode to flash ifwi, device go to closed mode after flash ifwi
        for i in range(3):
            if self.goto_fastboot(**kwargs):
                print "----start to flash IFWI on {}".format(kwargs["serial"])
                flash_log = os.popen(self._flash_ifwi_cmd.format(adapter_num=kwargs["adapter_num"])).read()
                if "success" in flash_log:
                    print "----success flash IFWI on {}".format(kwargs["serial"])
                    self.set_verdict(kwargs["serial"], True)
                    return True
                else:
                    print "----fail flash IFWI on {}".format(kwargs["serial"])
                    print flash_log
                    self.set_verdict(kwargs["serial"], False)
        return False

    def flash_image(self, flash_image_cmd_list, **kwargs):
        for i in range(3):
            if self.goto_fastboot(**kwargs):
                print "----start to flash image on {}".format(kwargs["serial"])
                for cmd in flash_image_cmd_list:
                    out_put = os.popen(cmd.format(serial=kwargs["serial"])).read()
                    print out_put
                    if "FAILED" in out_put:
                        print "----failed flash image on {}".format(kwargs["serial"])
                        self.set_verdict(kwargs["serial"], False)
                    else:
                        print "----success flash image on {}".format(kwargs["serial"])
                        self.set_verdict(kwargs["serial"], True)
                        return True
        return False

    def flash_ioc_ifwi_image(self, *flash_image_cmd_list, **kwargs):
        if self.flash_ioc(**kwargs):
            if self.flash_ifwi(**kwargs):
                self.flash_image(flash_image_cmd_list[0], **kwargs)
                print "----success flash ioc_ifwi_image on {}".format(kwargs["serial"])

    def flash_ioc_ifwi_image_p(self, flash_image_cmd_list, image_dir=None):
        os.chdir(image_dir)
        threads = []
        for kwargs in self._serial_port_adapter_num:
            t = threading.Thread(target=self.flash_ioc_ifwi_image, args=(flash_image_cmd_list,), kwargs=kwargs)
            threads.append(t)
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    @staticmethod
    def enable_developer_option(serial):
        #the time from flash finish to screen on is about 15s, this func should be excuted after screen on
        #sleep 60s to wait screen on
        time.sleep(60)
        print "\n----start enalbe developer option on {}".format(serial)
        d = Device(serial)
        time.sleep(2)
        if d(textContains="Drive safely").exists:
            d(text="Owner").click.wait()
        os.system("adb -s {} shell am start -S com.android.settings/.Settings".format(serial))
        time.sleep(1)
        if not d(textContains="System").exists and d(scrollable=True).exists:
            time.sleep(.5)
            d(scrollable=True).scroll.vert.to(textContains="System")
        d(text="System").click.wait()
        if d(text="Developer options").exists:
            return
        d(text="About phone").click.wait()
        for _ in range(8):
            d(textContains="Build number").click()
        d.press.back()
        if d(text="Developer options").exists:
            print "----success enable developer options on {}".format(serial)
            return

    @staticmethod
    def enable_OEM_unlocking(serial):
        print "----start to enable OEM unlocking on {}".format(serial)
        d = Device(serial)
        os.system("adb -s {} shell am start -S com.android.settings/.Settings".format(serial))
        time.sleep(1)
        if not d(textContains="System").exists and d(scrollable=True).exists:
            time.sleep(.5)
            d(scrollable=True).scroll.vert.to(textContains="System")
        d(text="System").click.wait()
        d(text="Developer options").click.wait()
        if not d(text="OEM unlocking").right(resourceId="android:id/switch_widget").checked:
            d(text="OEM unlocking").right(resourceId="android:id/switch_widget").click.wait()
        if d(text="ENABLE").exists:
            d(text="ENABLE").click.wait()
        if d(text="OEM unlocking").right(resourceId="android:id/switch_widget").checked:
            print "----success enable OEM unlock on {}".format(serial)

    def push_adbkey(self, serial):
        home_dir = os.path.expanduser("~")
        adbkey_pub = os.path.join(home_dir, ".android/adbkey.pub")
        print '11' * 50
        print home_dir
        self.shell_cmd_timeout("adb -s {} root".format(serial))
        self.shell_cmd_timeout("adb -s {} push {} /data/misc/adb/adb_keys".format(serial, adbkey_pub))
        #the below cmd is to use to enable adb
        self.shell_cmd_timeout("adb -s {} wait-for-device".format(serial))
        time.sleep(2)
        self.shell_cmd_timeout("adb -s {} ls /data/misc/adb/adb_keys".format(serial))
        time.sleep(2)
        for _ in range(3):
            if "adb_keys" in commands.getoutput("adb -s {} shell ls /data/misc/adb/adb_keys".format(serial)):
                print "----success push adbkey.pub to {}".format(serial)
                break
            else:
                print "\n----{} push adbkey.pub fail\n".format(serial)
            print commands.getoutput("adb -s {} push {} /data/misc/adb/adb_keys".format(serial, adbkey_pub))
            time.sleep(1)

    def enable_adb(self, *serial):
        #enable adb for user image, for CTS use
        serial = serial[0]
        self.enable_developer_option(serial)
        self.enable_OEM_unlocking(serial)
        self.push_adbkey(serial)

    def enable_adb_p(self):
        threads = []
        for kwargs in self._serial_port_adapter_num:
            serial = kwargs["serial"]
            t = threading.Thread(target=self.enable_adb, args=(serial,))
            threads.append(t)
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def flash(self):
        self.flash_ioc_ifwi_image_p(self._flash_image_cmd, image_dir=self._image_dir)
        if self._CTS_user_image_zip:
            self.enable_adb_p()
            self.flash_ioc_ifwi_image_p(self._flash_CTS_user_image_cmd, image_dir=self._CTS_user_image_dir)
        #if all success return True, or return False
        if False in self.flash_success.values():
            return False
        else:
            return True

