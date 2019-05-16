# -*- coding: UTF-8 -*-
'''
#@author:chengjiangx.jin@intel.com
#@since: 06/14/2016
'''


import subprocess
import shutil
import os
from testlib.util.common import g_common_obj
from testlib.common.common import reportSemiAutoVerdict, uploadSemiAutoLogFile


def pull_camera_folder_to_host():
    HOST_DATA_DIR = "/tmp/chromecast_semi_data"
    DEVICE_CAMERA_DATA_DIR = "/sdcard/DCIM/Camera"
    if os.path.exists(HOST_DATA_DIR):
        shutil.rmtree(HOST_DATA_DIR)
        os.mkdir(HOST_DATA_DIR)
    os.popen("adb pull " + DEVICE_CAMERA_DATA_DIR + " " + HOST_DATA_DIR).read()
    full_file_name = os.popen("ls -lt " + HOST_DATA_DIR + "/*" + " |awk '{print $9}' |head -n 1").read().strip("\n")
    return full_file_name


def upload_file_to_tcr_attachment(full_file_name):
    file_name = os.path.basename(full_file_name)
    shutil.copyfile(full_file_name, self._acs_params["report_path"] + "/PHONE1/DEBUG_LOGS/" + file_name)


def upload_file_to_prdshtwsv2d01(testcase, full_file_name):
    file_name = os.path.basename(full_file_name)
    live_report_log_path = g_common_obj.get_user_log_dir()
    shutil.copyfile(full_file_name, live_report_log_path + file_name)
    uploadSemiAutoLogFile(testcase, full_file_name)
    reportSemiAutoVerdict()


class UsbCamera(object):
    def __init__(self, output_file):
        self._usb_camera = os.popen('ls /dev/video*').read().strip()
        self._video_dir = self._get_video_dir()
        self._output_file = output_file + ".avi"
        self._output_file = self._video_dir + self._output_file
        self._record_process = ""
        self._record_cmd = "ffmpeg -y -f video4linux2 -i %s -f alsa -ac 2 -ar 48000 -i pulse -acodec mp3 -ar 48000 -vcodec libxvid -r 24 -s 800x600 -b 2000k %s" % (self._usb_camera, self._output_file)

    def _get_video_dir(self):
        if os.path.exists("/tmp/UsbCamera/"):
            shutil.rmtree("/tmp/UsbCamera/")
        os.mkdir("/tmp/UsbCamera/")
        return "/tmp/UsbCamera/"

    def start_record(self):
        self._record_process = subprocess.Popen(self._record_cmd, \
                                                stdin=subprocess.PIPE, \
                                                stdout=subprocess.PIPE, \
                                                stderr=subprocess.STDOUT, \
                                                shell=True)
        #(stdoutput, erroutput) = self._record_process.communicate()
        #print stdoutput
        #print erroutput
        if os.path.exists(self._output_file):
            print "-"*20 + "start to record, the recorded video is" + "-"*20
            print self._output_file

    def stop_record(self):
        self._record_process.communicate("q")
        if os.path.exists(self._output_file):
            print "-" * 20 + "record finished, the recorded video is" + "-" * 20
            print self._output_file
            return self._output_file
