
import sys
import os
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj
from testlib.graphics.extend_camera_impl import CameraExtendImpl


class CameraTest(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        setUpClass
        """
        try:
            super(CameraTest, self).setUpClass()
            CameraExtendImpl().startApp()
        except BaseException, e:
            print e
            raise

    @classmethod
    def tearDownClass(self):
        """
        tearDownClass
        """
        super(CameraTest, self).tearDownClass()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(CameraTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.photos = get_photo_implement()
        self._extendcamera = CameraExtendImpl()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self._extendcamera.launch_camera_am()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(CameraTest, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

    def testCamera(self):
        print "[RunTest]: %s" % self.__str__()
        self.photos .launch_photos_am()
        self._extendcamera.picTake()
        self.photos .launch_photos_am()
        self.photos .delete_photos_in_a_folder("Camera", 1)
        self._extendcamera.videoRecord()
        g_common_obj.assert_exp_happens()
