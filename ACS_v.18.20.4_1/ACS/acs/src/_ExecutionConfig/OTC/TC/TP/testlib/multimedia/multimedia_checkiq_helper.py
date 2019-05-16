# coding: UTF-8
'''
Created on May 9, 2017

@author: Li Zixi
'''
import os
import sys
import stat
import subprocess
from testlib.camera.checkIQ import CheckIQ
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.camera.CameraCommon import CameraCommon

from testlib.util.log import Logger
logger = Logger.getlogger()

old_percent = -1

def progress(width, percent):
    global old_percent
    if  percent != old_percent:
        print "\r%s %d%%" % (('%%-%ds' % width) % (width * percent / 100 * '='), percent),
        old_percent = percent
        sys.stdout.flush()

class MultiMediaCheckiqHelper:
    def __init__(self, host_path):
        self.host_path = host_path

        multimedia_setting = MultiMediaSetting(CameraCommon().DEFAULT_CONFIG_FILE)
        self.iq_check_file_path = multimedia_setting.download_file_to_host("Multimedia_Camera/apk/IQCheck")
        os.chmod(self.iq_check_file_path, stat.S_IRWXU|stat.S_IRGRP|stat.S_IROTH)
        self.__execute_command_with_popen("chmod 777 %s" % self.iq_check_file_path)
        self.zxing_jar_file_path = multimedia_setting.download_file_to_host("Multimedia_Camera/apk/zxing-analyzer.jar")
        self.home_path = os.environ['HOME']
        self.temp_dir = os.path.join(self.home_path, "tmp/checkiq_temp_dir/")
        if not os.path.exists(self.temp_dir):
            os.makedirs(self.temp_dir)
        self.check_IQ = CheckIQ(self.iq_check_file_path, self.temp_dir)

        self.num_list = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"

    def __execute_command_with_popen(self, cmd):
        logger.debug("cmd=%s" % cmd)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

    def compare_picture_similarity(self, file_1, file_2, region=CheckIQ.Region(max=True)):
        result = self.check_IQ.cmpHist(file_1, file_2, region)
        return result

    def convert_video_to_jpg(self, video_file_name):
        video_file_path = "%s/%s" % (self.host_path, video_file_name)
        jpg_folder = "%s/%s_jpg_folder" % (self.host_path, video_file_name)
        logger.debug("convert_video_to_jpg video_file_path=%s, jpg_folder=%s" % (video_file_path, jpg_folder))
        os.makedirs(jpg_folder)
        cmd = "ffmpeg -i %s %s" % (video_file_path, jpg_folder) + "/1m%04d.jpg -vcodec mjpeg -ss 0:0:0 -t 0:0:10"
        self.fdp = self.__execute_command_with_popen(cmd)
        self.fdp.wait()
        return video_file_path, jpg_folder

    def get_barcode_list(self, jpg_folder):
        jpg_name_list = os.listdir(jpg_folder)
        jpg_name_list.sort()
        return jpg_name_list

    def cut_picture(self, jpg_folder, jpg_name_list):
        from PIL import Image
        cut_temp_dir = os.path.join(self.home_path, "tmp/high_speed_temp_dir_p/")
        if not os.path.exists(cut_temp_dir):
            os.makedirs(cut_temp_dir)
        cut_tuple = (100, 90, 860, 510)
        for jpg_name in jpg_name_list:
            im = Image.open(os.path.join(jpg_folder, jpg_name))
            region = im.crop(cut_tuple)
            region.save(os.path.join(cut_temp_dir, jpg_name))

    def get_barcode_with_jpg_list(self, jpg_folder, jpg_name_list):
        barcode_list = []
        i = 1
        t_sum = len(jpg_name_list)
        print "==========analyze barcode=========="
        for jpg_name in jpg_name_list:
            jpg_path = os.path.join(jpg_folder, jpg_name)
            barcode = self.check_IQ.getBarcode(jpg_path, self.zxing_jar_file_path)
            barcode_list.append((jpg_name, barcode))
            if self.analyze_barcode(barcode[0]) != -1:
                break

            t_percent = int(i*100.0/t_sum)
            progress(50, (t_percent))
            i += 1
        progress(50, (100))
        print
        self.cut_picture(jpg_folder, jpg_name_list)
        return barcode_list

    def bit_sum(self, num):
        sum = 0
        while num > 0:
            sum += num % 10
            num = num / 10
        return sum

    def analyze_barcode(self, barcode):
        if len(barcode) > 5 or len(barcode) < 4:
            return -1
        barcode_num = barcode[:-1]
        if not barcode_num.isdigit():
            return -1
        barcode_num = int(barcode_num)
        if self.num_list[self.bit_sum(barcode_num)] != barcode[-1]:
#             print self.num_list[self.bit_sum(barcode_num)], "=========", barcode[-1]
            return -1
        return barcode_num

    def check_barcode_list(self, barcode_list):
        t_barcode_num = self.analyze_barcode(barcode_list[0])
        actual_barcode_num_list = [t_barcode_num]
        pass_num = 0
        error_num = 0
        for jpg_name, (barcode, _) in barcode_list:
#             print jpg_name, barcode
            barcode_num = self.analyze_barcode(barcode)
            if barcode_num == -1:
                continue
            elif barcode_num == t_barcode_num:
                continue
            elif barcode_num == t_barcode_num + 1:
                pass_num += 1
                actual_barcode_num_list.append(barcode_num)
                t_barcode_num = barcode_num
            else:
                pass_num += 1
                error_num += 1
#                 actual_barcode_num_list.append("error!")
                actual_barcode_num_list.append(barcode_num)
                t_barcode_num = barcode_num
        return pass_num, error_num, actual_barcode_num_list

    def check_video_with_barcode(self, video_file_name):
        video_file_path, jpg_folder = self.convert_video_to_jpg(video_file_name)
        jpg_name_list = self.get_barcode_list(jpg_folder)
        barcode_list = self.get_barcode_with_jpg_list(jpg_folder, jpg_name_list)
        self.check_barcode_list(barcode_list)
