# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/24/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.videopostprocessing_impl import VideoPPImpl


class VideoPP(RenderAppTestBase):

    def setUp(self):
        super(VideoPP, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._vidPP = VideoPPImpl()
        self._vidPP.init_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(VideoPP, self).tearDown()
        self._vidPP.delete_video()
        self._vidPP.uninstall_app()

    def test_videopostprocessing_coloreffect(self):
        ''' refer TC test_VideoPostProcessing_ColorEffect
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_original()
        self._vidPP.stop_androvid_am()

    def test_videopostprocessing_color_vintage(self):
        ''' refer TC test_VideoPostProcessing_Color_Vintage
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_vintage()
        self._vidPP.diff_md5('color_vintage')
        self._vidPP.stop_androvid_am()

    def test_videopostprocessing_color_conversion_sepia(self):
        ''' refer TC test_VideoPostProcessing_Color_conversion_Sepia
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_sepia()
        self._vidPP.diff_md5('color_conversion_sepia')
        self._vidPP.stop_androvid_am()

    def test_videopostprocessing_playing_rotation(self):
        ''' refer TC test_VideoPostProcessing_Playing_Rotation
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_rotation()
        self._vidPP.diff_md5('playing_rotation')
        self._vidPP.stop_androvid_am()

    def test_videopostprocessing_scaling_up_down_1280x720(self):
        ''' refer TC test_VideoPostProcessing_Scaling_up_down_1280x720
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_720p()
        self._vidPP.diff_md5('scaling_up_down_1280x720')

    def test_videopostprocessing_scaling_up_down_160p(self):
        ''' refer TC test_VideoPostProcessing_Scaling_up_down_160p
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_160p()
        self._vidPP.diff_md5('scaling_up_down_160p')

    def test_videopostprocessing_scaling_up_down_1920x1080(self):
        ''' refer TC test_VideoPostProcessing_Scaling_up_down_1920x1080
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_1080p()
        self._vidPP.diff_md5('scaling_up_down_1920x1080')

    def test_videopostprocessing_video_edit(self):
        ''' refer TC test_VideoPostProcessing_Video_Edit
        '''
        print "[RunTest]: %s" % self.__str__()
        self._vidPP.launch_androvid_am()
        self._vidPP.save_video_as_mirror_rotation()
        self._vidPP.diff_md5('video_edit')
        self._vidPP.stop_androvid_am()
