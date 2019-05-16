# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 11/23/2015
@author: Zhang RongX Z
'''
import time
from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.camera.camera import Camera
from testlib.graphics.set_wallpaper_impl import WallpaperImpl


class EditImagesAndSetasWallpaper(RenderAppTestBase):

    def setUp(self):
        super(EditImagesAndSetasWallpaper, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.device = g_common_obj.get_device()
        self._extendcamera = CameraExtendImpl()
        self._camera = Camera()
        self._photos = get_photo_implement()
        self._wallpaper = WallpaperImpl()
        self._photos.rm_delete_photos()
        self._photos.refresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(EditImagesAndSetasWallpaper, self).tearDown()
#         self._wallpaper.reset_wallpaper()
        self._photos.rm_delete_photos()
        self._photos.refresh_sdcard()

    def test_Imagecapturing_EditImage_SetasWallpaper(self):
        ''' refer TC test_Imagecapturing_EditImage_SetasWallpaper
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.launch_camera_am()
        self._camera.cameraSwitchTo("Camera")
        self._camera.picTake()
        self._photos.launch_photos_am()
        self._photos.open_a_picture("Camera")
        self._photos.apply_filter_to_photos("filter Ceres")
        time.sleep(5)
        for _ in range(0, 3):
            if not self.device(resourceId="com.google.android.apps.photos:id/edit").exists:
                time.sleep(3)
        self._photos.click_crop_tools()
        self._photos.rotate_90_degrees()
        self._photos.save_picture_after_rotation()
        g_common_obj.assert_exp_happens()
        time.sleep(5)
        for _ in range(0, 3):
            if not self.device(resourceId="com.google.android.apps.photos:id/edit").exists:
                time.sleep(3)
        self._photos.set_picture_as_wallpaper()
        g_common_obj.assert_exp_happens()
