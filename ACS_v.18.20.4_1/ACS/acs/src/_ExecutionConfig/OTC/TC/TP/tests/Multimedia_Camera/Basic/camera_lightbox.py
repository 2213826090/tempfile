# coding: utf-8
import time
import sys
from testlib.util.common import g_common_obj
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.checkIQ import CheckIQ
from testlib.multimedia.multimedia_lightbox_helper import MultiMediaLightBoxHelper, MultiMediaScrollHelper, MultiMediaRobotHelper

class CameraTest(CameraTestBase):
    """
    @summary: This test used to test camera function
    """
    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(CameraTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)

        self.multimedia_lightbox_helper = MultiMediaLightBoxHelper()
        self.multimedia_scroll_helper = MultiMediaScrollHelper()
        self.multimedia_robot_helper = MultiMediaRobotHelper()
        self.camera_common = CameraCommon()

        localPath = self.camera_common.getTmpDir()
        self.checkIQ = CheckIQ("/home/auto1/code/camera/git_CheckIQ/iqcheck/IQCheck", localPath)

        self.change_page_flag = 0

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        self.camera_common.removeDeivceFile()
        self.multimedia_scroll_helper.reset()
        self.multimedia_robot_helper.reset()
        #self.multimedia_robot_helper.move_to_default_position()
        self.multimedia_lightbox_helper.turn_off_all_light()
        if self.change_page_flag == 1:
            self.multimedia_scroll_helper.move(50, -50)
            self.multimedia_scroll_helper.set_position(0, 0)
        #time.sleep(3)
        #self.multimedia_scroll_helper.goto_page(2)

    def appPrepare(self):
        self.camera = self.camera_common.switchPlatform()

        self.host_path = self.camera_common.getTmpDir()
        self.camera_dir = self.camera_common.camera_dir
        self.camera_common.removeDeivceFile()
        self.camera_common.removeFile(self.host_path + "/*")
        self.camera.cleanMediaFiles()
        #self.camera_common.setOrientationToVertical()
        self.logger.debug("app prepare successfully")

    def test_camera_with_lightbox_operation(self, light_port_list=["Power", "CWF"], page_num=0, position=-1):
        self.multimedia_robot_helper.move_to_default_position(position)
        self.multimedia_lightbox_helper.press_light_with_list(light_port_list)
        self.multimedia_scroll_helper.reset()
        if page_num != 0:
            self.change_page_flag = 1
        self.multimedia_scroll_helper.goto_page(page_num)
        time.sleep(2)

    def test_camera_with_prepare_scroll(self, position=-1):
        self.test_camera_with_lightbox_operation(position=position)

        self.camera.startCameraApp()
        self.camera.selectMode("Camera")

        t_step = 800
        t_increment = 0
        find_border_flag = 0

        self.multimedia_scroll_helper.move(100, -100)
        for i in range(10):
            self.camera.capturePhoto()
            t_file = self.camera_common.getMediaFilesFromDevice(0, 1)[0]
            ret = self.checkIQ.detectRect(t_file)
            ret = int(ret)
            self.logger.debug("detectRect ret=%s" % ret)
            if i == 0 and ret >= 20:
                break
            elif find_border_flag == 0 and ret >= 20:
                find_border_flag = 1
            elif find_border_flag == 1 and ret >= 20:
                t_increment += t_step
            elif find_border_flag == 1 and ret < 20:
                t_increment = 0 - t_increment/2 - t_step
                break
            self.camera_common.removeDeivceFile()
            self.multimedia_scroll_helper.move(t_step, t_step/2)
        self.multimedia_scroll_helper.move(t_increment, t_increment/2)
        self.multimedia_scroll_helper.move(100, -100)
        self.multimedia_scroll_helper.set_position(0, 0)

    def test_camera_with_color_checker(self, size=-1, position=-1):
        if size == 2:
            position = 0
        self.test_camera_with_lightbox_operation(position=position)

        self.camera.startCameraApp()
        self.camera.selectMode("Camera")

        if size != -1:
            size = self.camera.getAllPhotoResolutions("Back")[size]
            self.logger.debug("size=%s" % size)
            self.camera.setPhotoResolution(size, "Back")

        self.camera.capturePhoto()

        t_file = self.camera_common.getMediaFilesFromDevice(0, 1)[0]
        ret = self.checkIQ.checkCorrupt(t_file)
        assert ret == "", "checkCorrupt ret=%s" % ret

    def test_camera_with_border(self, position=-1):
        self.test_camera_with_lightbox_operation(page_num=2, position=position)

        self.camera.startCameraApp()
        self.camera.selectMode("Camera")
        self.camera.capturePhoto()

        t_file = self.camera_common.getMediaFilesFromDevice(0, 1)[0]
        ret = self.checkIQ.findBorder(t_file)
        assert ret == "yes", "findBorder ret=%s" % ret

    def test_camera_with_move_test(self, position=-1):
        self.test_camera_with_lightbox_operation(page_num=0, position=position)
        self.multimedia_robot_helper.rotate(-100)
        self.multimedia_robot_helper.rotate(100)

        self.camera.startCameraApp()
        self.camera.selectMode("Camera")
        self.camera.capturePhoto()

        t_file = self.camera_common.getMediaFilesFromDevice(0, 1)[0]
        #ret = self.checkIQ.findBorder(t_file)
        ret = self.checkIQ.checkCorrupt(t_file)
        #assert ret == "yes", "findBorder ret=%s" % ret
        assert ret == "", "checkCorrupt ret=%s" % ret

    def lightbox_main_test(self, sub_func_name="", *arg, **keywords):
        """
        This test used to test Camera app
        The test case spec is following:
        1. appPrepare()
        2. do sub_func()
        """
        self.case_name = sys._getframe().f_back.f_code.co_name
        if sub_func_name == "":
            sub_func_name = "%s_sub_func" % self.case_name
        self.logger.debug("case_name=%s" % self.case_name)
        self.logger.debug("netflix_main_test---sub_func_name=%s" % sub_func_name)
        self.appPrepare()

        self.logger.debug("Arbitrary parameter is %s" % str(arg))
        self.logger.debug("keywords parameter is %s" % str(keywords))
        getattr(self, sub_func_name)(*arg, **keywords)

        self.logger.debug("Case %s is pass!" % self.case_name)

    def test_Camera_prepare_scroll(self):
        self.lightbox_main_test("test_camera_with_prepare_scroll")

    def test_Camera_BenchTest(self):
        self.lightbox_main_test("test_camera_with_color_checker", 2)

    def test_Camera_BenchTest_with_low_resolutions(self):
        self.lightbox_main_test("test_camera_with_color_checker", 2)

    def test_Camera_BenchTest_with_high_resolutions(self):
        self.lightbox_main_test("test_camera_with_color_checker", 1)

    def test_Camera_BorderTest(self):
        self.lightbox_main_test("test_camera_with_border")

    def test_Camera_MoveTest(self):
        self.lightbox_main_test("test_camera_with_move_test")
