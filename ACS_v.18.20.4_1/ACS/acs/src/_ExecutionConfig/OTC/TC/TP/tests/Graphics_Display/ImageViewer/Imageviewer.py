# -*- coding: utf-8 -*-
'''
@summary: Re-write by uiautomator method.
@since: 05/18/2017
@author: Rui
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.photos_impl import PARENT_EFFECTS,COLOR_CHILD_EFFECTS,LIGHT_CHILD_EFFECTS, CONFIG_FILE
from testlib.util.common import g_common_obj
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.common import adb32, launch_aosp_home, remove_aosp_launcher, pkgmgr, file_sys
import time
import random


class ImageView(UIATestBase):

    PHOTO_PATH = "/sdcard/Pictures"
    VIDEO_PATH = "/sdcard/Movies"
    ACCEPTABLE_VALUE = 5

    def setUp(self):
        super(ImageView, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        g_common_obj.root_on_device()
        self.d = g_common_obj.get_device()
        self.photosImpl = get_photo_implement()
        self.qr = QRcode()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.stop_photos_am()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageView, self).tearDown()
        self.photosImpl.rm_delete_photos()
        remove_aosp_launcher()

    def test_MoreThan10JPEGImagesOfDifferentResolution(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "jpg_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10BMPImages(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "bmp_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10BMPImagesOfDifferentResolution(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "bmp_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10GIFImagesOfDifferentResolution(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "gif_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10PNGImagesOfDifferentResolution(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "png_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10PNGImages(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "png_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10WBMPImagesOfDifferentResolution(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "wbmp_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10WEBPImagesOfDifferentResolution(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "webp_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_AddDisplayEffect_DragProgressBar(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        allEffects = PARENT_EFFECTS + LIGHT_CHILD_EFFECTS + COLOR_CHILD_EFFECTS
        useEffects = []
        loopNum = 5
        for _ in range(loopNum):
            index = random.randint(0, len(allEffects) - 1)
            useEffects.append(allEffects[index])
        useEffects.sort()
        for useEffect in useEffects:
            MinOrMax = [0, 100]
            index = random.randint(0, len(MinOrMax) - 1)
            effectVolume = MinOrMax[index]
            self.photosImpl.change_display_effect(effType=useEffect, effVol=effectVolume)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE,\
            "Failed! No effects on the saving pic."

    def test_ImageEdit_AddSaturationEffect(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        # ---------------Grag to left or right---------------
        index = random.randint(0, 1)
        if index == 0:
            effectVolume = random.randint(0, 49)
        else:
            effectVolume = random.randint(50, 100)
        self.photosImpl.change_display_effect(effType='Saturation', effVol=effectVolume)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) > 0, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_AddShadowEffect(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        # ---------------Grag to left or right---------------
        effectVolume = [0, 100][random.randint(0, 1)]
        self.photosImpl.change_display_effect(effType='Shadows', effVol=effectVolume)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_AddVignetteEffect_BlueImageEditingAndSwitching(self):
        from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_blue", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        settings_pkgName = "com.android.settings"
        g_common_obj.adb_cmd_capture_msg("am start %s" % settings_pkgName) # launch settings app previously
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Vignette', effVol=100)
        SystemUiExtendImpl().switch_recent_app(name='Settings')
        SystemUiExtendImpl().switch_recent_app(name='Photos')
        time.sleep(2)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_BMP_CancelEdit(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_001", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.change_display_effect(effVol=100)
            self.photosImpl.undo_edit()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_BMP_Zoom_ByGesture(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_001", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.pic_zoom_out(300)
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_EditPNGImage_CancelEdit(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.change_display_effect(effVol=100)
            self.photosImpl.undo_edit()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_GIF_Zoom_ByGesture(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_002", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.pic_zoom_in(300)
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_JPEG_Rotate_LeftRight(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_003", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.click_crop_tools()
            time.sleep(1)
            for _ in range(4):
                self.photosImpl.rotate_90_degrees()
                time.sleep(.5)
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_JPG_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_003", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        time.sleep(1)
        # --------Rotate left or right--------
        i = [1, 3][random.randint(0,1)]
        for _ in range(i):
            self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_JPG_Zoom_ByBoth(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_003", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.view_a_picture_fullscreen()
            self.photosImpl.pic_zoom_in(300)
            self.photosImpl.pic_zoom_out(300)
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_PNG_AddShadowEffect_2MBImage(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Shadows', effVol=100)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_PNG_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        time.sleep(1)
        # --------Rotate left or right--------
        i = [1, 3][random.randint(0, 1)]
        for _ in range(i):
            self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_PNG_SaveEdit(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effVol=100)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_PNG_Zoom_ByGesture(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.pic_zoom_out(300)
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_Rotate90degree_SaveImage(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_blue", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        time.sleep(1)
        self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) > 0, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_WBMP_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_006", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        time.sleep(1)
        # --------Rotate left or right--------
        i = [1, 3][random.randint(0, 1)]
        for _ in range(i):
            self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result_by_screenshot(self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_WEBP_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_007", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        time.sleep(1)
        # --------Rotate left or right--------
        i = [1, 3][random.randint(0, 1)]
        for _ in range(i):
            self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result_by_screenshot(self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_WEBP_Zoom_ByGesture(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_007", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.pic_zoom_in(300)
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_BWeffect_MoreThan5MB(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_morethan5mb", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        for eff in ['Whites', 'Blacks']:
            self.photosImpl.change_display_effect(effType=eff, effVol=100)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_BMP_Check4M(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_bmp4mb", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.view_a_picture_fullscreen()
        except:
            raise Exception("Test failed!")

    def test_imagedelete_press_menu_select_photo_delete(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_bmp4mb", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        bef_del = self.photosImpl.check_pic_number(self.PHOTO_PATH)
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.delete_a_picture()
        aft_del = self.photosImpl.check_pic_number(self.PHOTO_PATH)
        assert bef_del != aft_del, "Fail to delete pic!"

    def test_imageedit_blueimage_addvignetteeffect(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_blue", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Vignette', effVol=100)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_WBMP_Zoom_ByBoth(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_006", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.view_a_picture_fullscreen()
            self.photosImpl.pic_zoom_in(300)
            self.photosImpl.pic_zoom_out(300)
        except:
            raise Exception("Test failed!")

    def test_imageedit_addeffects_setwallpaper(self):
        print "[RunTest]: %s" % self.__str__()
        if "com.android.launcher3" not in pkgmgr.get_packages():
            launch_aosp_home()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Vignette', effVol=100)
        self.photosImpl.save_changes()
        self.photosImpl.click_crop_tools()
        self.photosImpl.rotate_right_45_degrees()
        self.photosImpl.save_picture_after_rotation()
        org_image = self.photosImpl.only_pull_saved_pic(self.PHOTO_PATH)
        out_image = self.qr.mark_image_qrcode(str(org_image))
        self.photosImpl.stop_photos_am()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.deploy_photo_content(directory=self.PHOTO_PATH, local_file=out_image)
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.set_picture_as_wallpaper()
        self.qr.verify_qrcode_marked_image(qrcode="9876543210",set_wallpaper=True)

    def test_imageedit_addstraighteneffect_lessthan20kb(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_blue", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        time.sleep(1)
        # --------Rotate left or right--------
        i = [1, 3][random.randint(0, 1)]
        for _ in range(i):
            self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) > 0, \
            "Failed! No effects on the saving pic."

    def test_MoreThan1000Images(self):
        from testlib.graphics.common import busybox_obj
        from testlib.util.config import TestConfig

        print "[RunTest]: %s" % self.__str__()
        # Prepare for 1000 imgs.
        self.photosImpl.deploy_photo_content("Pictures", "picture_morethan1000", self.PHOTO_PATH)
        pics = TestConfig().read(CONFIG_FILE, "Pictures").get("picture_morethan1000").split('/')[-1]
        busybox_obj.setup()
        cmd = "busybox tar xvf %s/%s -C %s" % (self.PHOTO_PATH, pics, self.PHOTO_PATH)
        chk_cmd = "\'ls %s/1000/ | grep -c png\'" % self.PHOTO_PATH
        clean_cmd = "rm -rf %s/%s" % (self.PHOTO_PATH, pics)
        busybox_obj.adb_busybox_cmd(cmd)
        rst = 0
        while rst != 1000:
            rst = int(g_common_obj.adb_cmd_capture_msg(chk_cmd))
            time.sleep(5)
        else:
            print "1000 pic prepared."
        g_common_obj.adb_cmd_capture_msg(clean_cmd)
        time.sleep(3)
        self.photosImpl.refresh_sdcard()
        time.sleep(10)
        try:
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture('1000')
            self.d.press.back()
            _index = random.randint(1, 30)
            sw_num = random.randint(1, 5)
            for i in range(sw_num):
                self.d().swipe.up()
                time.sleep(1)
            self.d(index = _index).click.wait()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_Add_Effect_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.change_display_effect(effType='Vignette', effVol=80)
            adb32.screen_rotation(1)
            adb32.screen_rotation(0)
        except:
            raise Exception("Test failed!")
        finally:
            adb32.screen_rotation(0)

    def test_ImageEdit_AddStraightenEffect(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        self.photosImpl.rotate_right_45_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) > self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_AddVignetteEffect(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Vignette', effVol=80)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_BMP_Check1K(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_bmp1k", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_BMP_Crop(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_001", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        self.photosImpl.rotate_right_45_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result_by_screenshot(self.PHOTO_PATH) > self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_BMP_SaveEdit(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_001", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Vignette', effVol=88)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result_by_screenshot(self.PHOTO_PATH) > self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_CorpUprightRectangular(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        self.photosImpl.crop_to_center(start_corner="left_bottom")
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_CropMoreThan8times_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.click_crop_tools()
            for i in range(9):
                self.photosImpl.rotate_90_degrees()
                time.sleep(.5)
            self.photosImpl.save_picture_after_rotation()
            assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) > 0, \
                "Failed! No effects on the saving pic."
        except:
            raise Exception("Test failed in round 1.")
        self.photosImpl.rm_delete_photos()

        try:
            adb32.screen_rotation(1)
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.click_crop_tools()
            for i in range(9):
                self.photosImpl.rotate_90_degrees()
                time.sleep(.5)
            self.photosImpl.save_picture_after_rotation()
            assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) > 0, \
                "Failed! No effects on the saving pic."
        except:
            raise Exception("Test failed in round 2.")
        finally:
            adb32.screen_rotation(0)

    def test_ImageEdit_JPG_1K_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_jpg1k", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_JPG_2M_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_jpg2m", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_JPG_CancelEdit(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_003", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.change_display_effect(effVol=100)
            self.photosImpl.undo_edit()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_JPG_Crop(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_003", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        self.photosImpl.crop_to_center(start_corner="left_top")
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_JPG_SaveEdit(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_003", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Vignette', effVol=22)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_PanoramicImage_AddEffect(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_panoramic", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Warmth', effVol=77)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_PersonFaceImage2MB_AddBWOrSharpnessEffects(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_face", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.change_display_effect(effType='Blacks', effVol=99)
        self.photosImpl.change_display_effect(effType='Whites', effVol=99)
        self.photosImpl.save_changes()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_PNG_Crop(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        self.photosImpl.crop_to_center(start_corner="right_top")
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_Rotate_Left(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        self.photosImpl.rotate_90_degrees()
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_Rotate_Right(self):
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.click_crop_tools()
        for i in range(3):
            self.photosImpl.rotate_90_degrees()
            time.sleep(.5)
        self.photosImpl.save_picture_after_rotation()
        assert self.photosImpl.check_saved_pic_result(picPath=self.PHOTO_PATH) >= self.ACCEPTABLE_VALUE, \
            "Failed! No effects on the saving pic."

    def test_ImageEdit_ThumbMD_All_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "jpg_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            adb32.screen_rotation(1)
        except:
            raise Exception("Test failed!")
        finally:
            adb32.screen_rotation(0)

    def test_ImageEdit_ThumbMD_One_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            adb32.screen_rotation(1)
        except:
            raise Exception("Test failed!")
        finally:
            adb32.screen_rotation(0)

    def test_ImageView_Webp_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_007", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageView_Webp_Thumbnail_LPCheck(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_webp2m", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.view_a_picture_fullscreen()
        except:
            raise Exception("Test failed!")

    def test_ImageView_Wbmp_2K_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_wbmp2k", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageView_Wbmp_300K_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_wbmp300k", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageView_Webp_1K_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_webp1k", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageView_Webp_2M_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_webp2m", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_BMP_Check8M(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_bmp8mb", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_ImageEdit_PNG_AllTransParent_Check(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_transparent", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
        except:
            raise Exception("Test failed!")

    def test_MoreThan10GIFImages(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "gif_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.view_a_picture_fullscreen()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_MoreThan10JPEGImages(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("PicturesFolder", "jpg_10resolution", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.view_a_picture_fullscreen()
            for _ in range(9):
                self.d().swipe.left(steps=10)
                time.sleep(2)
        except:
            raise Exception("Test failed!")

    def test_ImageView_Thumbnail_Mode(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_005", self.PHOTO_PATH)
            self.photosImpl.refresh_sdcard()
            self.photosImpl.launch_photos_am()
        except:
            raise Exception("Test failed!")

    def test_ImageZoom_JPEG(self):
        print "[RunTest]: %s" % self.__str__()
        try:
            self.photosImpl.deploy_photo_content("Pictures", "picture_004", self.PHOTO_PATH)
            self.photosImpl.launch_photos_am()
            self.photosImpl.open_a_picture()
            self.photosImpl.pic_zoom_out(30)
        except:
            raise Exception("Test failed!")

class BrowserView(ImageView):

    def setUp(self):
        super(BrowserView, self).setUp()
        try:
            from testlib.graphics.html5_impl import Html5Impl
            from testlib.graphics.extend_chrome_impl import ChromeExtendImpl
        except:
            raise Exception("Didn't import chrome library successfully.")
        # Init chrome
        self._Html5Impl = Html5Impl()
        self._Html5Impl.check_chrome_installed()
        self.extendchrome = ChromeExtendImpl()
        self.extendchrome.clear_data()

    def tearDown(self):
        super(BrowserView, self).tearDown()
        self.extendchrome.stop_by_am()

    def _is_image_opened(self):
        time.sleep(2) # wait for loading complete
        _is_pic_show = True
        if self.d(text="CONTINUE").exists: self.d(text="CONTINUE").click.wait() # Handle downloading event
        if self.d(textContains="was not found").exists or self.d(description="New tab").exists:
            _is_pic_show = False
        assert _is_pic_show, "Image not open."

    def test_ImageView_ByWebBrowser_JPEG(self):
        self.photosImpl.deploy_photo_content("PicturesFolder", "jpg_10resolution", self.PHOTO_PATH)
        _get_pics = file_sys.get_file_list(self.PHOTO_PATH)
        self.extendchrome.launch()
        self.extendchrome.chrome_setup()
        for p in _get_pics:
            self.extendchrome.open_website("file://" + p)
            self._is_image_opened()

    def test_ImageView_ByWebBrowser_WBMP(self):
        self.photosImpl.deploy_photo_content("PicturesFolder", "wbmp_10resolution", self.PHOTO_PATH)
        _get_pics = file_sys.get_file_list(self.PHOTO_PATH)
        self.extendchrome.launch()
        self.extendchrome.chrome_setup()
        for p in _get_pics:
            self.extendchrome.open_website("file://" + p)
            time.sleep(2)
            if self.d(text="CONTINUE").exists: self.d(text="CONTINUE").click.wait()
            time.sleep(2)
        _get_pics = file_sys.get_file_list("/sdcard/Download/")
        # not support wbmp format showing on chrome directly, open those images by photos app.
        try:
            for p in _get_pics:
                self.photosImpl.open_image_command(p)
                time.sleep(2)
        except:
            raise Exception("Fail to open image on photos.")
