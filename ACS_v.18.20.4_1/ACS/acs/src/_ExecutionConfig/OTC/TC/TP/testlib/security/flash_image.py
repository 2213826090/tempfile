#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import argparse
import serial
import urllib2
import re
import commands
import requests
import time
import datetime
from testlib.util.config import TestConfig
from testlib.util.common import g_common_obj
from testlib.security.security_impl import SecurityImpl
from init_flash_device import InitFlashDevices

d = g_common_obj.get_device()
NOSERUNNER = "NOSERUNNER"

class FlashImage(object):

    def __init__(self):
        self.cfg_file = "tests.tablet.security.conf"
        self.dsn = str(d.server.adb.device_serial())
        self._debugCard_port = "ls /dev/ttyUSB*_debugCard_"
        self.securityImpl = SecurityImpl()
        self.init_dut = InitFlashDevices()

    def check_product(self):
        cmd = "getprop ro.product.name"
        product = g_common_obj.adb_cmd_capture_msg(cmd)
        return product

    def get_port(self, port_num = -2):
        sn_port = {}
        if self.dsn:
            cmd = self._debugCard_port + self.dsn
            # port = os.popen(cmd).read().strip().split("\n")[-2]
            info = os.popen(cmd).read().strip()
            if info == '':
                cmd_local = "ls /dev/ttyUSB*"
                port = os.popen(cmd_local).read().strip().split("\n")[port_num]
            else:
                port = os.popen(cmd).read().strip().split("\n")[port_num]
            sn_port[self.dsn] = port
        print sn_port
        return sn_port

    def download_artifactory_content(self, option_file):
        from testlib.util.repo import Artifactory
        cfg = TestConfig().read(self.cfg_file, "artifactory")
        arti_obj = Artifactory(cfg.get("location"))
        ret_file = arti_obj.get(cfg.get(option_file))
        return ret_file

    def download_image_file(self, option_file, file_name):
        from testlib.util.repo import Artifactory
        ret_file = ""
        cfg = TestConfig().read(self.cfg_file, "ebimage")
        try:
            arti_obj = Artifactory(cfg.get("location"))
            ret_file = arti_obj.get(cfg.get(option_file)+file_name)
            print ret_file
        except Exception, e:
            print "[Debug]", Exception, ":", e
            if e:
                print cfg.get("chengdu_location")
                arti_obj = Artifactory(cfg.get("chengdu_location"))
                ret_file = arti_obj.get(cfg.get(option_file) + file_name)
                print ret_file
        return ret_file

    def get_testconfig_url(self, option_file, location_site):
        cfg = TestConfig().read(self.cfg_file, "ebimage")
        ret_file = cfg.get(location_site)
        target_name = cfg.get(option_file)
        url = os.path.join(ret_file, target_name)
        print url
        return url

    def get_download_file_res(self, option_file, target_case_name):
        url = self.get_testconfig_url(option_file, "location")
        html = ""
        try:
            request_url = urllib2.Request(url)
            response = urllib2.urlopen(request_url)
            html = response.read()
        except Exception, e:
            print "[Debug]", Exception, ":", e
            if e:
                url = self.get_testconfig_url(option_file, "chengdu_location")
                request_url = urllib2.Request(url)
                response = urllib2.urlopen(request_url)
                html = response.read()
        file_last_date = {}
        pattern_obj = target_case_name
        # '<a href="gordon_peak-flashfiles-O10000645.zip">gordon_peak-flashfiles-O10000645.zip</a>   14-Nov-2017 13:51  898.62 MB\n',
        pattern_res = re.compile('<a href="(.*{}-.*\d+.zip)">'.format(pattern_obj))
        pattern_date = re.compile('<a href=".*{}-.*\d+.zip">.*\s(\d+-\D+-\d+\s\d+:\d+)'.format(pattern_obj))
        i = 1
        for line in html.splitlines():
            if line.startswith('<a'):
                m = pattern_res.search(line)
                if m:
                    file_name = m.group(1)
                    file_last_date.setdefault("ebfile_" + str(i), []).append(file_name)
                n = pattern_date.search(line)
                if n:
                    last_date = n.group(1)
                    file_last_date.setdefault("ebfile_" + str(i), []).append(last_date)
                i = i + 1
        print "[Info]---EB image file dict: {}".format(file_last_date)
        date_keys = file_last_date.keys()
        local_time = datetime.datetime.now()
        str_date = local_time.strftime('%Y-%m-%d %H:%M:%S')
        current_date_time = datetime.datetime.strptime(str_date, '%Y-%m-%d %H:%M:%S')
        if len(date_keys) == 0:
            raise EnvironmentError("[Info]This Page is not exist eb image, please upload eb image")
        elif len(date_keys) == 1:
            print "[Info]--------EB file num is 1"
            key_1 = date_keys[0]
            eb_image = file_last_date.get(key_1)[0]
            eb_data = file_last_date.get(key_1)[1]
            print "[Info]---Now, current date time is: {}".format(current_date_time)
            upload_date = datetime.datetime.strptime(eb_data, '%d-%b-%Y %H:%M')
            days_diff = current_date_time - upload_date
            res_days = datetime.timedelta(days=5)
            if days_diff > res_days:
                raise EnvironmentError, "This link no newest weekly eb image,date: {}, please upload new eb image".format(upload_date)
            else:
                print "The test EB image is: {} Last Data: {}".format(eb_image, eb_data)
                return eb_image
        elif len(date_keys) > 1:
            print "[Info]--------EB file num:{}  > 1".format(len(date_keys))
            # {'ebfile_3':['gordon_peak-flashfiles-O10000645.zip','4-Nov-2017 13:51'],'ebfile_4':['gordon_peak-flashfiles-O10000651.zip','6-Nov-2017 16:05']}
            # ['ebfile_3', 'ebfile_4']
            # [1509774660, 1509955500]
            date_list = []
            for i in range((len(date_keys))):
                get_date = file_last_date.get(date_keys[i])[1]
                # switch string to float timestamp
                date_stamp = time.mktime(time.strptime(get_date, '%d-%b-%Y %H:%M'))
                date_list.append(int(date_stamp))
            max_date = max(date_list)
            #print max_date
            index_date = date_list.index(max_date)
            eb_image = file_last_date.get(date_keys[index_date])[0]
            eb_data = file_last_date.get(date_keys[index_date])[1]
            print "[Info]---Now, current date time is: {}".format(current_date_time)
            upload_date = datetime.datetime.strptime(eb_data, '%d-%b-%Y %H:%M')
            days_diff = current_date_time - upload_date
            res_days = datetime.timedelta(days=5)
            if days_diff > res_days:
                raise EnvironmentError, "This link no newest weekly eb image,date: {}, please upload new eb image".format(upload_date)
            else:
                print "The test EB image is: {} Last Data: {}".format(eb_image, eb_data)
                return eb_image
        else:
            print "[Info]--------EB file other num:{}  ".format(len(date_keys))

    def kill_minicom_process_PID(self):
        result = os.popen("ps -aux |grep minicom| awk '{print $2}'").readlines()
        print result
        for pid in result:
            pid = pid.strip("\r\n")
            pid = pid.strip()
            os.system("echo '123456' | sudo -S  kill " + pid)

    def get_argument(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("--bin")
        parser.add_argument("--zip")
        args = parser.parse_args()
        return args

    def flash_files(self, bin_file = None, zip_file = None):
        ret_file = self.download_image_file("bootdevicepy", "bootDeviceTo.py")
        port = "/dev/ttyUSB2"
        #self.get_argument()
        flash_bin_result = False
        flash_zip_result = False
        while flash_bin_result == False or flash_zip_result == False:
            flash_bin_result = False
            flash_zip_result = False
            #os.system("sudo python " + ret_file + " --port /dev/ttyUSB2 --mode elk")
            for i in range(5):
                os.system("sudo python " + ret_file + " --port /dev/ttyUSB2 --mode elk")
                time.sleep(10)
                fastboot_dvs = os.popen("fastboot devices").readlines()
                print fastboot_dvs
                if fastboot_dvs != []:
                    break
            return_result = os.popen("/opt/intel/platformflashtool/bin/ias-spi-programmer --write " + bin_file).readlines()
            for line in return_result:
                line = line.strip("\r\n")
                print "<Flash bin>: " + line
                if "Write verification completed successfully" == line: flash_bin_result = True
            #os.system("sudo python " + ret_file + " --port /dev/ttyUSB2 --mode fastboot")
            for i in range(5):
                os.system("sudo python " + ret_file + " --port /dev/ttyUSB2 --mode fastboot")
                time.sleep(40)
                fastboot_dvs = os.popen("fastboot devices").readlines()
                print fastboot_dvs
                if fastboot_dvs != []:
                    break
            # os.system("sudo python ./temp/files/bootDeviceTo.py --port /dev/ttySerial3 --mode fastboot")
            return_result = os.popen("cflasher -f " + zip_file + " -c blank_gr_mrb_b1").readlines()
            for line in return_result:
                line = line.strip("\r\n")
                print "<Flash zip>: " + line
                if "Flash success" in line: flash_zip_result = True

    def auto_flash_image(self, image, user_image = None):
        #Bxtp(self.dsn, image, user_image).flash()
        # sudo python flash_bxtp_p.py --serial_port="{'R1J56Lac06b382':'/dev/ttyUSB7', 'R1J56Lfd8f9708':'/dev/ttyUSB3'}" --image_zip=..
        #echo "123456" | sudo -S minicom -D /dev/ttyUSB3
        flash_py = self.download_artifactory_content("flash_bxtp")
        print "[Info]------The flash script is: {}".format(flash_py)
        ser_port = str(self.get_port())
        if os.environ.get(NOSERUNNER) is None:
            print "[Info]------For AFT team flash image"
            time.sleep(30)
            if user_image is None:
                print os.system('sudo python {} --serial_port="{}" --image_zip="{}" '.format(flash_py, ser_port, image))
            else:
                print os.system('sudo python {} --serial_port="{}" --image_zip="{}" --CTS_user_image_zip="{}" '.format
                                (flash_py, ser_port, image,user_image))
            time.sleep(30)
        else:
            print "[Info]------For local flash image"
            time.sleep(30)
            if user_image is None:
                print os.system('echo "123456" |sudo -S python {} --serial_port="{}" --image_zip="{}" '.format(flash_py, ser_port, image))
            else:
                print os.system('echo "123456" |sudo -S python {} --serial_port="{}" --image_zip="{}" --CTS_user_image_zip="{}" '.format
                                (flash_py, ser_port,image,user_image))
            time.sleep(30)
        try:
            self.securityImpl.check_boot_completed()
            if self.securityImpl.check_adb_connection() is False:
                fastboot_dvs = commands.getoutput("fastboot devices")
                if fastboot_dvs != '':
                    os.system("fastboot reboot")
                    time.sleep(30)
            self.init_dut.init_main()
        except Exception, e:
            print Exception, ":", e


