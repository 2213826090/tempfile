#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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
@summary: Class for reliability System cases
@since: 06/09/2014
@author: yongzoux (yongx.zou@intel.com)
"""

import os
import re
import random
import time
from nose.tools import assert_equals

from testlib.util.common import g_common_obj

class SystemImpl(object):

    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self.brightness = {'start':None, 'end':None, 'step':10,
                           'inc':0, 'current':[0, 0]}
        self.volumes = {'media':
                            {'start':None, 'end':None, 'step':10,
                           'inc':0, 'current':[0, 0]},
                        'notification':
                            {'start':None, 'end':None, 'step':10,
                           'inc':0, 'current':[0, 0]},
                        'alarm':
                            {'start':None, 'end':None, 'step':10,
                           'inc':0, 'current':[0, 0]}
                       }

    @staticmethod
    def check_device_info():
        """
        check device info
        """
        cmd = "getprop ro.build.version.incremental"
        print "[INFO] Device name %s" % str(g_common_obj.get_device_name())
        print "[INFO] Device type %s" % str(g_common_obj.get_device_type())
        print "[INFO] Device version %s" % g_common_obj.adb_cmd_capture_msg(cmd)

    def launch_home_screen(self):
        self.d.wakeup()
        g_common_obj.back_home()
        for _ in xrange(10):  # max 10s timeout
            if self.d.exists(description="Apps"):
                break
            time.sleep(1)
        else:
            raise Exception("launch home screen fails")

    @staticmethod
    def delete_file(path, location, is_check_result=False):
        """ delete file on device or host """
        print "Delete file: %s on %s" % (path, location)
        if location.upper() == "DEVICE":
            cmd = 'rm -f %s; echo $?' % path
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
        elif location.upper() == "HOST":
            cmd = "rm -f %s 2>&1; echo $?" % path
            result_list = os.popen(cmd).readlines()
        else:
            if is_check_result:
                assert_equals(False, True, \
                    "Can not support this location information")
            print "Can not support this location information"

        suc_del = False
        if len(result_list):
            ret = result_list[-1].rstrip()
            if int(ret) == 0:
                suc_del = True
        if is_check_result:
            assert_equals(suc_del, True, \
                "Delete file: %s on %s fail" % \
                (path, location))
        print "Delete file: %s %s" % \
        (path, suc_del and "successfully" or "fail")
        return suc_del

    @staticmethod
    def delete_folder(path, location, is_check_result=False):
        """delete folder on device or host"""
        print "Delete folder: %s on %s" % (path, location)
        if location.upper() == "DEVICE":
            cmd = 'rm -rf %s; echo $?' % path
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
        elif location.upper() == "HOST":
            cmd = "rm -rf %s 2>&1; echo $?" % path
            result_list = os.popen(cmd).readlines()
        else:
            if is_check_result:
                assert_equals(False, True, \
                    "Can not support this location information")
            print "Can not support this location information"

        suc_del = False
        if len(result_list):
            ret = result_list[-1].rstrip()
            if int(ret) == 0:
                suc_del = True
        if is_check_result:
            assert_equals(suc_del, True, "Delete folder: %s fail" % path)
        print "Delete folder: %s %s" % \
        (path, suc_del and "successfully" or "fail")
        return suc_del

    def launch_sound(self):
        """launch sound"""
        g_common_obj.launch_app_am("com.android.settings", "com.android.settings.Settings")
        self.d(textMatches="Sound & notification").click()
        print "launch sound success <-"

    def click_media_volumes(self):
        """click volumes"""
        print "start to click Volumes ->"
        item='media'

        #d(text="Media volume").sibling(resourceId="android:id/seekbar")
        rect = self.d(text="Media volume").sibling(resourceId="android:id/seekbar").info["bounds"]
        y = (rect["top"] + rect["bottom"])/2
        self.volumes[item]["start"] = (rect["left"] + 10, y)
        self.volumes[item]["end"] = (rect["right"] -10, y)
        if self.volumes[item]["step"] <= 0:
            self.volumes[item]["step"] = 10
        self.volumes[item]["inc"] = (rect["right"] - \
                                    rect["left"])/self.volumes[item]["step"]
        self.volumes[item]["current"] = list(self.volumes[item]["start"])
        self.d.click(self.volumes[item]["start"][0], self.volumes[item]["start"][1])
        print "click Volumes success <-"
    def media_volumes_up(self):
        """Up the volumes"""
        print "start to up the Volumes ->"
        item='media'
        x = self.volumes[item]["current"][0] + self.volumes[item]["inc"]
        y = self.volumes[item]["current"][1]
        if x > self.volumes[item]["end"][0]:
            x = self.volumes[item]["end"][0]
        self.d.click(x, y)
        self.volumes[item]["current"] = [x, y]
        print "up the Volumes success <-"

    def media_volumes_down(self):
        """Down the volumes"""
        print "start to down the volumes ->"
        item='media'
        x = self.volumes[item]["current"][0] - self.volumes[item]["inc"]
        if x < self.volumes[item]["start"][0]:
            x = self.volumes[item]["start"][0]
        y = self.volumes[item]["current"][1]
        self.d.click(x, y)
        self.volumes[item]["current"] = [x, y]
        print "down the volumes success <-"

    def media_volumes_middle(self):
        """Middle the volumes"""
        print "start to middle the volumes ->"
        item='media'
        x = (self.volumes[item]["start"][0] + self.volumes[item]["end"][0])/2
        y = self.volumes[item]["current"][1]
        self.d.click(x, y)
        self.volumes[item]["current"] = [x, y]
        if self.d(text="OK", packageName="com.android.settings").exists:
            self.d(text="OK", packageName="com.android.settings").click.wait()
        print "middle the volumes success <-"

    def adjust_volume_in_setting_sound(self):
        self.click_media_volumes()
        for i in range(10):
            self.media_volumes_up()
            time.sleep(3)
        for i in range(10):
            self.media_volumes_down()
            time.sleep(3)
        self.media_volumes_middle()

    def adjust_volume_by_volume_buttons(self):
        for i in range(10):
            print "press volume up button"
            g_common_obj.adb_cmd("input keyevent 24")
            time.sleep(3)
        for i in range(10):
            print "press volume down button"
            g_common_obj.adb_cmd("input keyevent 25")
            time.sleep(3)
        self.media_volumes_middle()

    def adjust_volume_by_volume_buttons_in_sleep(self):
        print "[INFO]: press power button"
        g_common_obj.adb_cmd("input keyevent 26")
        for i in range(10):
            print "press volume up button"
            g_common_obj.adb_cmd("input keyevent 24")
            time.sleep(3)
        for i in range(10):
            print "press volume down button"
            g_common_obj.adb_cmd("input keyevent 25")
            time.sleep(3)
        g_common_obj.adb_cmd("input keyevent 26")
        time.sleep(3)
        self.media_volumes_middle()

    #add 2 method for adb connection
    #samx.lan@intel.com
    #2014/10/22
    def adb_devices(self):
        result=os.popen("adb devices").read()
        assert result.count("device")>1, "[ERROR]: adb devices failed"
        print "[INFO]: adb devices successfully"
    def adb_shell_ls(self):
        result=os.popen("adb shell ls").read()
        assert result.find("device not found")==-1, "[ERROR]: adb shell ls failed"
        print "[INFO]: adb shell ls successfully"
    def generate_test_file(self, dir_path, filename, fsize, location):
        """generate file with specified size"""
        file_path = os.path.join(dir_path, filename)
        print "Start to generate file: %s with size: %s on %s" % \
        (file_path, fsize, location)
        bs, count = self.parse_bs_count(fsize)
        assert_equals(bs>0, True, \
            "Fail to get proper bs and count value for dd command")
        cmd = "dd if=/dev/zero of=%s bs=%d count=%d" % (file_path, bs, count)
        print "Generate file with cmd: %s" % cmd
        if location.upper() == "DEVICE":
            # create folder
            create_folder_cmd = 'mkdir -p %s; echo $?' % dir_path
            g_common_obj.adb_cmd_capture_msg(create_folder_cmd)
            # create file
            cmd = "%s; echo $?" % cmd.rstrip()
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
        elif location.upper() == "HOST":
            # create folder
            create_folder_cmd = 'mkdir -p %s 2>&1; echo $?' % dir_path
            os.popen(create_folder_cmd)
            # create file
            cmd = "%s 2>&1; echo $?" % cmd.rstrip()
            result_list = os.popen(cmd).readlines()
        else:
            assert_equals(False, True, \
                "Can not support this location information")
        if len(result_list):
            ret = result_list[-1].rstrip()
            assert_equals(int(ret), 0, "Generate file fail")
            print "Generate file successful"
        else:
            assert_equals(False, True, "Generate file fail")

    def adb_pull_file(self, device_path, \
        host_path, is_check_file=False, file_size=None):
        """ adb pull file from device to host """
        print "Adb pull file from device: %s to host: %s" % \
        (device_path, host_path)
        cmd = 'adb pull %s %s' % (device_path, host_path)
        print "Pull file: %s" % cmd
        os.popen(cmd)
        if is_check_file:
            exist = self.file_exists(host_path, location="HOST")
            assert_equals(exist, True, \
                "Expect file exist, Actual file not exist")
            if file_size:
                self.verify_file_size(host_path, file_size)
        print "Pull file successfully"

    def adb_push_file(self, host_path, device_path, is_check_file=False):
        """ adb push file from host to device """
        print "Adb push file from host: %s to device: %s" % \
        (host_path, device_path)
        cmd = 'adb push %s %s' % (host_path, device_path)
        print "Push file: %s" % cmd
        os.popen(cmd)
        if is_check_file:
            exist = self.file_exists(device_path, location="DEVICE")
            assert_equals(exist, True, \
                "Expect file exist, Actual file not exist")
        print "Push file successfully"

    @staticmethod
    def file_exists(file_path, location):
        """ check file exist, support check on device or host """
        print "Check file: %s exist on %s or not" % \
        (file_path, location.upper())
        exist = False
        if location.upper() == "DEVICE":
            cmd = 'ls %s; echo $?' % (file_path)
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
        elif location.upper() == "HOST":
            cmd = 'ls %s 2>&1; echo $?' % (file_path)
            result_list = os.popen(cmd).readlines()
        else:
            print "Can not support this location:[%s] information" % location
            return False

        if len(result_list):
            ret = result_list[-1].rstrip()
            ret2= result_list[0].rstrip()
            if ret2.count("No such file or directory")!=0:
                exist = False
                return exist
            if int(ret) == 0:
                exist = True
        print "File " + (exist and "Exist" or "Not Exist")
        return exist

    @staticmethod
    def get_file_size(file_path):
        """ get size of file on host, return size of bytes """
        print "Get size of file: %s" % file_path
        cmd = ''.join(['stat -c %s ', file_path, ' 2>&1; echo $?'])
        result_list = os.popen(cmd).readlines()
        if len(result_list):
            if int(result_list[-1].rstrip()) == 0:
                return int(result_list[0].rstrip())
        return -1

    def verify_file_size(self, file_path, exp_size):
        """ verify file size on host """
        print "Verify file size of %s, expect size: %s" % (file_path, exp_size)
        bs, count = self.parse_bs_count(exp_size)
        assert_equals(bs>0, True, "Parse bs, count for dd command fail")
        exp_size = bs*count
        act_size = self.get_file_size(file_path)
        print "Expect size: %d bytes, Actual size: %d bytes" % \
        (exp_size, act_size)
        assert_equals(act_size, exp_size, \
            "File %s expect size:[%d], but actual size:[%d]" % \
            (file_path, exp_size, act_size))

    @staticmethod
    def parse_bs_count(file_size):
        """ according to file size, parse bs and count value for dd command """
        size_match = "(\d+)(\S*)"
        try:
            ret = re.search(size_match, file_size)
            size = int(ret.group(1))
            unit = ret.group(2)
        except Exception:
            print "Match file size fail."
            return -1, -1

        #set bs, count value
        if unit in ["GB", "Gb", "gb", "g", "G"]:
            bs = 1024*1024
            count = size * 1024
        elif unit in ["MB", "Mb", "mb", "m", "M"]:
            bs = 1024*1024
            count = size
        elif unit in ["KB", "Kb", "kb", "k", "K"]:
            bs = 1024
            count = size
        else:
            print "Can not get bs, count with unit: %s." % unit
            return -1, -1
        return bs, count

    @staticmethod
    def get_host_md5sum(file_path):
        """ Getting host file's md5sum """
        cmd = "md5sum %s 2>&1; echo $?" % file_path
        result_list = os.popen(cmd).readlines()
        if len(result_list):
            if int(result_list[-1].rstrip()) == 0:
                result = result_list[0].rstrip()
                message = result.split(' ')
                if not message[0]:
                    print "Getting file's md5sum failed!"
                    return -1
                md5sum = message[0]
                print "Get %s md5sum: %s" % (file_path, md5sum)
                return md5sum
        print "Getting file's md5sum failed!"
        return -1

    def compare_file_md5sum(self, file1_path, file2_path):
        """ Compare two files md5sum """
        md5_file1 = self.get_host_md5sum(file1_path)
        md5_file2 = self.get_host_md5sum(file2_path)
        assert_equals(md5_file1, md5_file2, \
            "File %s md5sum: %s, but file %s md5sum: %s" % (\
                file1_path, md5_file1, file2_path, md5_file2))

    @staticmethod
    def compare_folder_md5sum(folder1_path, folder2_path):
        """ Compare two host folder md5sum
        """
        # Get all files's md5sum of folder1_path and save to /tmp/folder.md5
        cmd = 'cd %s; find . -type f -print0 | \
        xargs -0 md5sum > /tmp/folder.md5; echo $?' % folder1_path
        result_list = os.popen(cmd).readlines()
        ret = int(result_list[-1].rstrip())
        assert ret == 0, "Get md5sum of folder: %s fail" % folder1_path
        # Check md5sum of folder2_path is same as folder1_path
        cmd = 'cd %s; md5sum --status -c /tmp/folder.md5; echo $?' \
        % folder2_path
        result_list = os.popen(cmd).readlines()
        ret = int(result_list[-1].rstrip())
        assert ret == 0, "Two folder's md5sum not same"

    def random_create_file(self, dir_path, min_size, \
        max_size, file_count, location):
        """ Create file with random size """
        for i in range(1, file_count+1):
            print "Create  %d/%d file" % (i, file_count)
            file_name = file_size = str(random.randint(min_size, max_size)) \
            + 'KB'
            self.generate_test_file(dir_path, file_name, file_size, location)

    def copy_by_cmd_on_device(self, src_path, dst_path):
        """ Copy file/folder by cp cmd on device """
        cmd = 'cp -rf "%s" "%s"; echo $?' % (src_path, dst_path)
        result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
        copy_suc = False
        if len(result_list)>0:
            ret = result_list[-1].rstrip()
            if int(ret) == 0:
                copy_suc = True
        msg_fail = "Copy from %s to %s fail" % (src_path, dst_path)
        assert_equals(copy_suc, True, msg_fail)
        file_exists = self.file_exists(dst_path, "Device")
        assert_equals(file_exists, True, msg_fail)
        print "Copy %s to %s successfully" % (src_path, dst_path)

    @staticmethod
    def get_device_available_disk_space(mount_point):
        """ Get available disk space of mount point of device
        return size with MB or KB
        """
        cmd = 'df %s; echo $?' % mount_point
        result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        available_space = None
        if len(result_list)>0:
            ret = result_list[-1].rstrip()
            if int(ret) == 0:
                del result_list[-1]
                for line in result_list:
                    if line.strip() and line.find(mount_point) != -1:
                        parts = line.strip().split()
                        if len(parts) == 5 and parts[3]:
                            available_space = parts[3].upper()
                            if not available_space.endswith("B"):
                                available_space = available_space + "B"
        if available_space:
            size_match = "(\d+\.?\d?)(\S*)"
            try:
                ret = re.search(size_match, available_space)
                size = float(ret.group(1))
                unit = ret.group(2)
            except Exception:
                print "Match file size fail."
                return -1, -1

            if unit in ["GB", "Gb", "gb"]:
                count = size * 1024
                unit = "MB"
            elif unit in ["MB", "Mb", "mb"]:
                count = size
            elif unit in ["KB", "Kb", "kb"]:
                count = size
            else:
                print "Fail to parse this disk space: %s." % available_space
                return -1, -1
            return int(count), unit
        return -1, -1

    def make_device_to_low_memeory(self, percent, \
        lowresource_folder, file_name):
        """ Generate a big file with size is almost of available device memory,
            to make device into low memory state
        """
        print "Make device to low memory"
        data_folder = self.cfg.get("device_data_folder")
        available_disk_space, unit = self.\
        get_device_available_disk_space(data_folder)
        assert_equals(available_disk_space > 0, True, \
            "Fail to get available disk space of %s" % data_folder)
        file_size = str(available_disk_space*percent/100) + unit
        self.generate_test_file(lowresource_folder, \
            file_name, file_size, "Device")
        exists = self.file_exists(\
            os.path.join(lowresource_folder, file_name), "Device")
        assert_equals(exists, True, "Set device to low memory fail")

    @staticmethod
    def get_usb_mount_path():
        """
        @summary: get usb mount folder from mount output message
        @return: char usb mount path
        """
        print "[Grace_Debug]:Get usb mount path"
        cmd = "mount|grep /mnt/media_rw/usbdisk"
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        result = [i for i in result if i != '']
        assert len(result) > 0, "Get mount info error: [%s], \
        Please make sure the usb has been connected" % result
        mbit = result[-1].find("usbdisk")
        print("INFO:Get usb mount path is %s" % result[-1][mbit:mbit+8])
        return result[-1][mbit:mbit+8]

    @staticmethod
    def folder_md5sum_check(folder_path):
        """
        @summary: Compare the folder between device and host
        @parameter:
            folder_path : the file path
        @return: None
        """
        os.system("mkdir /tmp/md5_check")
        cmd_pull = "adb pull %s /tmp/md5_check;echo $?" % folder_path
        result = os.popen(cmd_pull)
        assert result[0] == "0", "ERROR:Please make sure %s is existed!"
        cmd = " find /tmp/md5_check -type f -print0 | \
        xargs -0 md5sum > /tmp/my.md5"
        if not os.path.exists("/tmp/my.md5"):
            os.system(cmd)
        md5sum = os.popen("md5sum -c /tmp/my.md5").read()
        #print("[@@Grace_Debug]:MD5 check result %s" % md5sum)
        assert md5sum.find("FAILED"), "ERROR:Transfer Error"

    def folder_file_transfer(self, ori_file, tar_file):
        """
        @summary: Transfer the ori_file to tar_file and verify if the size is different
        @parameter:
            ori_file: the original file
            tar_file: the target file
        @return: None
        """
        print("INFO:Translate file")
        print ("INFO:[%s] to [%s]" % (ori_file, tar_file))
        cmd = "cp -rf %s/* %s;echo $?" % (ori_file, tar_file)
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        #print "[@@Grace_Debug]:Cp command return %s" % result , type(result)
        assert int(result) == 0, "ERROR:Cp command return False, \
        Please make sure the usb or sdcard is available"
        self.folder_md5sum_check(tar_file)

    def iozone_setup(self):
        """
        @summary: check if iozone is existed and chmod bin file
        """
        assert self.check_file("/data/local/tmp", "iozone_android"), \
        "ERROR: Please run deploy_bat_content_file.sh first"
        g_common_obj.adb_cmd_capture_msg(\
            "chmod 777 /data/local/tmp/iozone_android")

    def device_rw(self, path, p_size, t_size):
        """
        @summary: R/W devices by iozone command
        @parameter:
            path: Test folder path
            p_size: per size for iozone to rw
            t_size: total size for iozont to rw
        @return: None
        """
        print("INFO:Test R/W for %s" % path)
        result_file = "iozone_result.xls"
        result_path = "/data/local/tmp"
        iozone_rw = "/data/local/tmp/iozone_android -r " \
        + p_size + " -s " + t_size \
        + " -i 0 -i 1 -f " + path + \
        "/iozone_file -Rb " + result_path + "/" + result_file
        re_rw = g_common_obj.adb_cmd_capture_msg(iozone_rw, 100)
        assert self.check_file(result_path, result_file), \
        "ERROR: The iozone run error. msg: %s" % re_rw
        cmd_delete = "rm " + result_path + "/" + result_file
        g_common_obj.adb_cmd_capture_msg(cmd_delete)

    @staticmethod
    def check_file(path, file_name):
        """
        @summary: check if file is existed in path
        @parameter:
            path: the folder path to check on device
            file_name: file name
        @return True or False
        """
        cmd_verify = "ls " + path + "/|grep " + file_name
        pipe = g_common_obj.adb_cmd_capture_msg(cmd_verify)
        if len(pipe) == 0:
            print("WARNING: The %s is not existed in %s" % (file_name, path))
            return False
        return True
