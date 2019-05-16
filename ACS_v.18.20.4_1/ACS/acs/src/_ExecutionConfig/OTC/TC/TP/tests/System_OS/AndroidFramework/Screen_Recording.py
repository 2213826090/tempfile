# -*- coding:utf-8 -*-

'''
@summary: Android screen recording test.
@since: 07/06/2016
@author: Lijin Xiong
'''

import os,time,subprocess
from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.common import OrientationChanger,d,UiAutomatorUtils
from testlib.androidframework.screen_recorder_utils import ScreenRecorder,ScreenRecorderManager
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

class ScreenRecording(UIATestBase):

    def setUp(self):
        super(ScreenRecording, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
        d.freeze_rotation(False)

    def test_screen_recording(self):
        recorder = ScreenRecorder(3000000, 50, 1)
        test_file_name = "recording_test_video.mp4"
        screen_record_file_path = None
        try:
            if ScreenRecorderManager.start_recording_delay_var_name in os.environ:
                del os.environ[ScreenRecorderManager.start_recording_delay_var_name]
            recorder.start_recording()
            recording_time_secs = 20
            orientations = ["n", "l", "r"]
            for i in range(recording_time_secs):
                time.sleep(1)
                if i % 4 == 0:
                    d.open.notification()
                    time.sleep(1)
                    d.press.recent()
                    OrientationChanger.change_orientation(orientations[(i / 4) % 3])
            recorder.stop_recording()
            time.sleep(2)
            screen_record_file_path = recorder.retrieve_recording(test_file_name)
            print screen_record_file_path
            self.assertTrue(screen_record_file_path is not None)
            statinfo = os.stat(screen_record_file_path)
            self.assertTrue(statinfo.st_size > 80000L)  # magic size number
            subprocess.call("rm -rf *.mpg", shell=True)  # ensure there is no duplicate file
            # throws exception if conversion yields errors
            subprocess.check_output("file " + screen_record_file_path, shell=True)
        finally:
            if screen_record_file_path is not None:
                os.remove(screen_record_file_path)  # remove the pulled media file
                subprocess.call("rm -rf *.mpg", shell=True)