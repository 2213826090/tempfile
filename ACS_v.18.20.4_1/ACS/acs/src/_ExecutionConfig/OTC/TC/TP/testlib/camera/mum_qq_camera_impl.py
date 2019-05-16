# coding: utf-8
'''
Created on Aug 14, 2014

@author: yongga1x
'''

from testlib.util import constant
from testlib.util.common import g_common_obj
import time

class QQCameraImpl():
    def __init__(self):
        self.d = constant.Device
        self.dut = self.d
        self.build_number = constant.BUILD_TYPE
        self.path = ""

    def enter_qq_camera_from_home(self):
        self.dut.press.back()
        g_common_obj.launch_app_from_home_sc("My Cam")
        time.sleep(5)

    def switch_module_in_camera(self , module = "Video"):
        self.d(resourceId = "com.tencent.qqcamera:id/photo_pattern_switch_button").click()
        self.d(text = module).click()
        time.sleep(3)

    # enter camera and set video resolution
    def enter_camera_setting_video(self):
        print "enter_camera_setting_video"
        self.d(resourceId = "com.tencent.qqcamera:id/photo_pattern_switch_button").click()
        self.d( text = "Settings").click()
        time.sleep(3)
        assert self.d( text = "Camera settings").exists,"Enter camera Setting page failed"

    def set_camera_setting_video_storage_location(self, storage = constant.STORAGE_LOCATION_EXTRAPOSITION):
        print "set_camera_setting_video_storage_location"
        self.d( text = "Storage location").click()
        self.d( text = storage).click()
        if self.d( text = "OK").exists:
            self.d( text = "OK").click()
        assert self.d( text = "APPS").exists,"set storage location " + storage + " failed"

    def enter_camera_setting_quality(self , quality):
        print "set_camera_setting_video_storage_location"
        self.d( text = "Camera quality").click()
        assert self.d( text = quality).exists,"quality " + quality + " not exists"
        self.d( text = quality).click()

    # initial page and capture Video via camera
    def capture_video_camera_initial_page(self  , recordTime , times=1):
        for t in range(times):
            print "times is:", t
            time.sleep(3)
            print "Recording video times is :[" + str(t) + "]"
            self.d(resourceId="com.tencent.qqcamera:id/shutter_button").click()
            time.sleep(3)
            try :
                self.d(resourceId="com.tencent.qqcamera:id/recording_time").text
            except:
                time.sleep(3)
                print "not find time ,video record not started"
            try :
                self.d(resourceId="com.tencent.qqcamera:id/recording_time").text
            except:
                assert False, "video record not started"
            while True:
                time.sleep(30)
                text = self.d(resourceId="com.tencent.qqcamera:id/recording_time").text.split(":")
                text = int(text[0]) * 60 + int(text[1])
                print "times is :", t, " recordTime text is :", text
                if (text >= recordTime):
                    self.d(resourceId="com.tencent.qqcamera:id/shutter_button").click()
                    break

#qq_camera_impl = QQCameraImpl()
