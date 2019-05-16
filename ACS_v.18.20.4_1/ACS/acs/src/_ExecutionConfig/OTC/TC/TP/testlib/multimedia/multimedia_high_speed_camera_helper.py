# coding: UTF-8
'''
Created on Oct 10, 2017

@author: Li Zixi
'''
import os
import flycapture2 as fc2
import numpy as np
from scipy import misc

from testlib.util.log import Logger
logger = Logger.getlogger()

class MultiMediaHighSpeedCameraHelper:
    def __init__(self):
        self.home_path = os.environ['HOME']
        self.temp_dir = os.path.join(self.home_path, "tmp/high_speed_temp_dir/")
        if not os.path.exists(self.temp_dir):
            os.makedirs(self.temp_dir)

        self.c = fc2.Context()
#         c.get_num_of_cameras()
        self.connect()
#         c.get_camera_info()
#         m, f = self.c.get_video_mode_and_frame_rate()
#         print m, f

    def connect(self):
        self.c.connect(*self.c.get_camera_from_index(0))

    def disconnect(self):
        self.c.disconnect()

    def clear_temp_dir(self):
        os.system("rm -rf %s/*" % self.temp_dir)
        if not os.path.exists(self.temp_dir):
            os.makedirs(self.temp_dir)

    def start_capture(self, num=1):
        self.c.start_capture()
        for i in range(1, num+1):
            im = self.c.retrieve_buffer()
            im = im.convert_to(fc2.PIXEL_FORMAT_RGB8)
            misc.imsave(os.path.join(self.temp_dir, 'tmp_%003d.jpg' % i), im)
        self.c.stop_capture()
        return self.temp_dir
