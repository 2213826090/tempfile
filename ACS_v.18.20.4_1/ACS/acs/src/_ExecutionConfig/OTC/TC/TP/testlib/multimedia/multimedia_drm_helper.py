# coding: UTF-8
import os
import sys
import time
import subprocess
import re
import thread
from testlib.util.config import TestConfig
from testlib.util.common import g_common_obj
from testlib.common.common import reportSemiAutoVerdict, uploadSemiAutoLogFile

from testlib.util.log import Logger
logger = Logger.getlogger()

def cur_file_dir():
    path = sys.path[0]
    if os.path.isdir(path):
        return path
    elif os.path.isfile(path):
        return os.path.dirname(path)

def real_file_dir():
    return os.path.split(os.path.realpath(__file__))[0]

class MultiMediaDRMHelper:
    
    config = TestConfig()
    
    def __init__(self):
        cfg_file = os.path.join(real_file_dir(), "multimedia_drm_helper.conf")
        self.cfg = self.config.read(cfg_file,  "config")
        
        time_str = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        self.output_folder = os.path.abspath(os.path.join(self.cfg.get("output_file_path"), time_str))
        self.upload_flag = self.cfg.get("upload_flag")
        self.upload_folder = os.path.join(self.cfg.get("upload_path"), time_str)
        self.record_cmd = self.cfg.get("record_cmd")
        self.record_with_time_cmd = self.cfg.get("record_with_time_cmd")
        self.check_video_device_cmd = self.cfg.get("check_video_device_cmd")
        
        if "multimedia_drm_camera_input_device" in os.environ:
            self.input_device = os.environ["multimedia_drm_camera_input_device"]
        else:
            camera_node = self.config.read("/etc/oat/sys.conf", "camerasetting").get("camera_node")
            if camera_node == "" or camera_node == None:
                camera_node = self.cfg.get("input_device")
            self.input_device = self.checkVideoDevice(camera_node)
            os.environ["multimedia_drm_camera_input_device"] = self.input_device
        logger.debug("self.input_device = %s" % self.input_device)
        
        self.fdp = ""

    def executeCommandWithPopen(self, cmd):
        logger.debug("cmd=%s" % cmd)
#         return subprocess.Popen(cmd, shell=True)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)

    def getOutputVideoFolder(self):
        if "output_video_folder" not in dir(self):
            self.output_video_folder = os.path.join(self.output_folder, "video")
        if not os.path.exists(self.output_video_folder):
            os.makedirs(self.output_video_folder)
        return self.output_video_folder
    
    def getOutputLogFolder(self):
        if "output_log_folder" not in dir(self):
            self.output_log_folder = os.path.join(self.output_folder, "log")
        if not os.path.exists(self.output_log_folder):
            os.makedirs(self.output_log_folder)
        return self.output_log_folder

    def saveLog(self, t_str, log_file):
        file_object = open(log_file, "w")
        file_object.write(t_str)
        file_object.close()

    def startRecord(self, output_file_name):
        assert self.fdp == "", "Record already start!"
        self.output_file_name = output_file_name
        self.output_file = os.path.join(self.getOutputVideoFolder(), "%s.avi" % self.output_file_name)
        cmd = self.record_cmd % (self.input_device, self.output_file)
        logger.debug("cmd :%s" %cmd )
        self.fdp = self.executeCommandWithPopen(cmd)
        return self.fdp

    def stopRecord(self):
        if self.fdp == "":
            logger.debug("Record not start!")
            return
        assert self.fdp != "", "Record not start!"
        t_str = self.fdp.communicate("q")
        self.fdp = ""
        log_file = os.path.join(self.getOutputLogFolder(), "%s.log" % self.output_file_name)
        self.saveLog(t_str[0], log_file)
        class TempClass:
            def __init__(self, upload_folder):
                self._acs_params = {}
                upload_folder = upload_folder.split("/",1)
                self._acs_params["report_path"] = upload_folder[0]
                self._testMethodName = upload_folder[1]
        if int(self.upload_flag) == 1:
            uploadSemiAutoLogFile(TempClass(os.path.join(self.upload_folder, "video")), self.output_file)
            uploadSemiAutoLogFile(TempClass(os.path.join(self.upload_folder, "log")), log_file)

    def terminateRecord(self):
        if self.fdp == "":
            logger.debug("Record not start!")
            return
        self.fdp.terminate()
        self.fdp = ""

    def recordWithTime(self, output_file_name, record_time):
        self.output_file_name = output_file_name
        self.output_file = os.path.join(self.getOutputVideoFolder(), "%s.avi" % self.output_file_name)
        cmd = self.record_with_time_cmd % (record_time, self.input_device, record_time, self.output_file)
        return self.executeCommandWithPopen(cmd)

    def checkVideoDevice(self, video_deivce=""):
        self.output_file = os.path.join(self.getOutputVideoFolder(), "check_device_video.avi")
        if video_deivce == "":
            video_deivce = "/dev/video0"
        for i in range(0, 5):
            cmd = self.check_video_device_cmd % (video_deivce, self.output_file)
            time.sleep(2)
            fdp = self.executeCommandWithPopen(cmd)
            result = fdp.stdout.read()
            if "not installed" in result or "command not found" in result:
                logger.debug("cmd result = %s" % result)
                assert 0, "Can not find ffpmeg command! please install it"
            if "Invalid" not in result and "Cannot" not in result and "error" not in result and "fail" not in result:
                if not os.path.exists(self.output_file):
                    logger.debug("cmd result = %s" % result)
                    continue
                self.input_device = video_deivce
                logger.debug("video_deivce = %s" % video_deivce)
                return self.input_device
            video_deivce = "/dev/video%d" % i
        assert 0, "Can not find camera!"

class MultiMediaVideoQualityHelper:
    config = TestConfig()
    
    def __init__(self):
        self.d = g_common_obj.get_device()
        self.serial = self.d.server.adb.device_serial()
        
        self.get_SurfaceView_status_cmd = "adb shell dumpsys SurfaceFlinger | grep '| SurfaceView'"
        self.get_play_status_in_SurfaceView_pattern = re.compile("[^\|]+ \| ([0-9a-fA-F]+) \|")
        self.get_resolution_in_SurfaceView_pattern = re.compile('([^|]+\|){7}([^,]+,){3}\s+([0-9]+).*')
    
    def executeCommandWithPopen(self, cmd):
#         logger.debug("cmd=%s" % cmd)
        return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    
    def executeAdbShellCommandWithPopen(self, cmd):
        fdp = self.executeCommandWithPopen(cmd.replace("adb shell", "adb -s %s shell" % self.serial))
        result = fdp.stdout.read()
        logger.debug("cmd result = %s" % result)
        return result
    
    def getSurfaceViewHandlerStatus(self):
        result = self.executeAdbShellCommandWithPopen(self.get_SurfaceView_status_cmd)
        match = self.get_play_status_in_SurfaceView_pattern.findall(result)
        logger.debug("match=%s" % match)
        if match:
            handler1 = match[0]
            logger.debug("handler1=%s" % handler1)
        else:
            logger.debug("Returning Status: NOVIDEO...")
            return "NOVIDEO"
        time.sleep(0.3)
        result = self.executeAdbShellCommandWithPopen(self.get_SurfaceView_status_cmd)
        match = self.get_play_status_in_SurfaceView_pattern.findall(result)
        logger.debug("match=%s" % match)
        if match:
            handler2 = match[0]
            logger.debug("handler2=%s" % handler2)
        else:
            logger.debug("Returning Status: NOVIDEO...")
            return "NOVIDEO"
        
        if handler1 == handler2:
            logger.debug("Returning Status: PAUSED...")
            return "PAUSED"
        else:
            logger.debug("Returning Status: PLAYING...")
            return "PLAYING"
    
    def getSurfaceViewResolution(self):
        result = self.executeAdbShellCommandWithPopen(self.get_SurfaceView_status_cmd)
        height = self.get_resolution_in_SurfaceView_pattern.sub('\\3', result)
        assert height, "Can't find resolution!"
        height = int(height)
        logger.debug("height=%s" % height)
        return height
    
    def checkSurfaceViewResolution_start(self, expect_height, interval_time=3):
        self.expect_flag = -1
        self.check_flag = 1
        print "expect_height=:%s" % expect_height
        while(self.check_flag):
            if self.getSurfaceViewResolution() >= expect_height:
                self.expect_flag = 1
                break
            time.sleep(interval_time)
    
    def checkSurfaceViewResolutionWithThread_start(self, expect_resolution=480):
        thread.start_new_thread(self.checkSurfaceViewResolution_start, (expect_resolution,))
    
    def checkSurfaceViewResolutionWithThread_stop(self):
        self.check_flag = 0
        logger.debug("expect_flag:%s" % self.expect_flag)
        return self.expect_flag