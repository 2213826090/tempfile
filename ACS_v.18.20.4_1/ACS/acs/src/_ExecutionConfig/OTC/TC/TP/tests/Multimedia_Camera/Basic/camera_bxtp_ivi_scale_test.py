# coding: utf-8
from tests.Multimedia_Camera.Basic.camera_bxtp_ivi_scale_test_base import CameraTest as MainCameraTest

class CameraTest(MainCameraTest):
    def test_Camera_Scale_TP_YUYV_640x480_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["vga yuy2 0 0 1280 720 60"])

    def test_Camera_Scale_TP_YUYV_640x480_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["vga yuy2 0 0 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_640x480_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["vga yuy2 0 0 640 480 60"])

    def test_Camera_Scale_TP_YUYV_720x576_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["576p yuy2 0 0 1280 720 50"])

    def test_Camera_Scale_TP_YUYV_720x576_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["576p yuy2 0 0 1920 1080 50"])

    def test_Camera_Scale_TP_YUYV_720x576_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["576p yuy2 0 0 640 480 50"])

    def test_Camera_Scale_TP_YUYV_720x576_Progressive_to_NV21_720x576(self):
        self.check_scale_TP_with_similarity(["576p yuy2 0 0 720 576 50"])

    def test_Camera_Scale_TP_YUYV_1280x720_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["720p yuy2 0 0 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_1280x720_Progressive_to_NV21_3840x2160(self):
        self.check_scale_TP_with_similarity(["720p yuy2 0 0 3840 2160 60"])

    def test_Camera_Scale_TP_YUYV_1280x720_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["720p yuy2 0 0 640 480 60"])

    def test_Camera_Scale_TP_YUYV_1280x720_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["720p yuy2 0 0 1280 720 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 0 0 1280 720 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_to_NV21_240x135(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 0 0 240 135 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_to_NV21_3840x2160(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 0 0 3840 2160 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 0 0 640 480 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 0 0 1920 1080 60"])


    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_to_NV21_1280x720_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 1 1 1280 720 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_to_NV21_1920x1080_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 1 1 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_to_NV21_240x135_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 1 1 240 135 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_to_NV21_3840x2160_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 1 1 3840 2160 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_to_NV21_640x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p yuy2 1 1 640 480 60"])

    def test_Camera_Scale_TP_YUYV_720x576_Interlace_to_NV21_1280x720_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p yuy2 1 1 1280 720 50"])

    def test_Camera_Scale_TP_YUYV_720x576_Interlace_to_NV21_1920x1080_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p yuy2 1 1 1920 1080 50"])

    def test_Camera_Scale_TP_YUYV_720x576_Interlace_to_NV21_640x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p yuy2 1 1 640 480 50"])

    def test_Camera_Scale_TP_YUYV_720x576_Interlace_to_NV21_720x576_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p yuy2 1 1 720 576 50"])

    def test_Camera_Scale_TP_YUYV_720x480_Interlace_to_NV21_1280x720_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p yuy2 1 1 1280 720 60"])

    def test_Camera_Scale_TP_YUYV_720x480_Interlace_to_NV21_1920x1080_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p yuy2 1 1 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_720x480_Interlace_to_NV21_640x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p yuy2 1 1 640 480 60"])

    def test_Camera_Scale_TP_YUYV_720x480_Interlace_to_NV21_720x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p yuy2 1 1 720 576 60"])



    def test_Camera_Scale_TP_UYVY_1280x720_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["720p uyvy 0 0 1280 720 60"])

    def test_Camera_Scale_TP_UYVY_1280x720_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["720p uyvy 0 0 640 480 60"])

    def test_Camera_Scale_TP_UYVY_1280x720_Progressive_to_NV21_3840x2160(self):
        self.check_scale_TP_with_similarity(["720p uyvy 0 0 3840 2160 60"])

    def test_Camera_Scale_TP_UYVY_1280x720_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["720p uyvy 0 0 1920 1080 60"])

    def test_Camera_Scale_TP_UYVY_640x480_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["vga uyvy 0 0 1280 720 60"])

    def test_Camera_Scale_TP_UYVY_640x480_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["vga uyvy 0 0 1920 1080 60"])

    def test_Camera_Scale_TP_UYVY_640x480_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["vga uyvy 0 0 640 480 60"])

    def test_Camera_Scale_TP_UYVY_720x576_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["576p uyvy 0 0 1280 720 50"])

    def test_Camera_Scale_TP_UYVY_720x576_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["576p uyvy 0 0 1920 1080 50"])

    def test_Camera_Scale_TP_UYVY_720x576_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["576p uyvy 0 0 640 480 50"])

    def test_Camera_Scale_TP_UYVY_720x576_Progressive_to_NV21_720x576(self):
        self.check_scale_TP_with_similarity(["576p uyvy 0 0 720 576 50"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_to_NV21_1280x720(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 0 0 1280 720 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_to_NV21_240x135(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 0 0 240 135 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_to_NV21_3840x2160(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 0 0 3840 2160 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_to_NV21_640x480(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 0 0 640 480 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_to_NV21_1920x1080(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 0 0 1920 1080 60"])


    def test_Camera_Scale_TP_UYVY_1920x1080_Interlace_to_NV21_1280x720_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 1 1 1280 720 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Interlace_to_NV21_240x135_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 1 1 240 135 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Interlace_to_NV21_3840x2160_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 1 1 3840 2160 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Interlace_to_NV21_640x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 1 1 640 480 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Interlace_to_NV21_1920x1080_Deinterlace(self):
        self.check_scale_TP_with_similarity(["1080p uyvy 1 1 1920 1080 60"])

    def test_Camera_Scale_TP_UYVY_720x480_Interlace_to_NV21_1280x720_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 1280 720 60"])

    def test_Camera_Scale_TP_UYVY_720x480_Interlace_to_NV21_1920x1080_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 1920 1080 60"])

    def test_Camera_Scale_TP_UYVY_720x480_Interlace_to_NV21_640x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 640 480 60"])

    def test_Camera_Scale_TP_UYVY_720x480_Interlace_to_NV21_720x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 720 480 60"])

    def test_Camera_Scale_TP_UYVY_720x576_Interlace_to_NV21_1280x720_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p uyvy 1 1 1280 720 50"])

    def test_Camera_Scale_TP_UYVY_720x576_Interlace_to_NV21_1920x1080_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p uyvy 1 1 1920 1080 50"])

    def test_Camera_Scale_TP_UYVY_720x576_Interlace_to_NV21_640x480_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p uyvy 1 1 640 480 50"])

    def test_Camera_Scale_TP_UYVY_720x576_Interlace_to_NV21_720x576_Deinterlace(self):
        self.check_scale_TP_with_similarity(["576p uyvy 1 1 720 480 50"])


    def test_Camera_Scale_AVM737_720x480_to_1280x720(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 1280 720 30"])

    def test_Camera_Scale_AVM737_720x480_to_1920x1080(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 1920 1080 30"])

    def test_Camera_Scale_AVM737_720x480_to_640x480(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 640 480 30"])

    def test_Camera_Scale_AVM737_720x480_to_720x480(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 720 480 30"])

    def test_Camera_Scale_AVM737_720x480_to_176x144(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 176 144 30"])

    def test_Camera_Scale_AVM737_720x480_to_320x240(self):
        self.check_scale_TP_with_similarity(["480p uyvy 1 1 320 240 30"])




    def test_Camera_Scale_TP_UYVY_1280x720_Progressive_fps_30(self):
        self.check_scale_TP_with_fps(["720p uyvy 0 0 1920 1080 30", "720p uyvy 0 0 3840 2160 30", "720p uyvy 0 0 640 480 30", "720p uyvy 0 0 1280 720 30"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_fps_30(self):
        self.check_scale_TP_with_fps(["1080p uyvy 0 0 1280 720 30", "1080p uyvy 0 0 240 135 30", "1080p uyvy 0 0 3840 2160 30",
                                      "1080p uyvy 0 0 640 480 30", "1080p uyvy 0 0 1920 1080 30"])

    def test_Camera_Scale_TP_UYVY_640x480_Progressive_fps_60(self):
        self.check_scale_TP_with_fps(["vga uyvy 0 0 1280 720 60", "vga uyvy 0 0 1920 1080 60", "vga uyvy 0 0 640 480 60"])

    def test_Camera_Scale_TP_UYVY_720x576_Progressive_fps_50(self):
        self.check_scale_TP_with_fps(["576p uyvy 0 0 640 480 50", "576p uyvy 0 0 720 576 50", "576p uyvy 0 0 1280 720 50", "576p uyvy 0 0 1920 1080 50"])

    def test_Camera_Scale_TP_UYVY_1280x720_Progressive_fps_60(self):
        self.check_scale_TP_with_fps(["720p uyvy 0 0 1920 1080 60", "720p uyvy 0 0 3840 2160 60", "720p uyvy 0 0 640 480 60", "720p uyvy 0 0 1280 720 60"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Progressive_fps_60(self):
        self.check_scale_TP_with_fps(["1080p uyvy 0 0 1280 720 60", "1080p uyvy 0 0 240 135 60", "1080p uyvy 0 0 3840 2160 60",
                                      "1080p uyvy 0 0 640 480 60", "1080p uyvy 0 0 1920 1080 60"])

    def test_Camera_Scale_TP_UYVY_720x480_Interlace_Deinterlace_fps_30(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 640 480 60", "480p uyvy 1 1 1280 720 60", "480p uyvy 1 1 1920 1080 60", "480p uyvy 1 1 720 480 60"])

    def test_Camera_Scale_TP_UYVY_720x576_Interlace_Deinterlace_fps_25(self):
        self.check_scale_TP_with_fps(["576p uyvy 1 1 1280 720 50", "576p uyvy 1 1 1920 1080 50", "576p uyvy 1 1 640 480 50", "576p uyvy 1 1 720 576 50"])

    def test_Camera_Scale_TP_UYVY_1920x1080_Interlace_Deinterlace_fps_30(self):
        self.check_scale_TP_with_fps(["1080p uyvy 1 1 1280 720 60", "1080p uyvy 1 1 240 135 60", "1080p uyvy 1 1 3840 2160 60",
                                      "1080p uyvy 1 1 640 480 60", "1080p uyvy 1 1 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_1280x720_Progressive_fps_60(self):
        self.check_scale_TP_with_fps(["720p yuy2 0 0 1920 1080 60", "720p yuy2 0 0 3840 2160 60", "720p yuy2 0 0 640 480 60", "720p yuy2 0 0 1280 720 60"])

    def test_Camera_Scale_TP_YUYV_640x480_Progressive_fps_60(self):
        self.check_scale_TP_with_fps(["vga yuy2 0 0 1280 720 60", "vga yuy2 0 0 1920 1080 60", "vga yuy2 0 0 640 480 60"])

    def test_Camera_Scale_TP_YUYV_720x576_Progressive_fps_50(self):
        self.check_scale_TP_with_fps(["576p yuy2 0 0 640 480 50", "576p yuy2 0 0 720 576 50", "576p yuy2 0 0 1280 720 50", "576p yuy2 0 0 1920 1080 50"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_fps_60(self):
        self.check_scale_TP_with_fps(["1080p yuy2 0 0 1280 720 60", "1080p yuy2 0 0 240 135 60", "1080p yuy2 0 0 3840 2160 60",
                                      "1080p yuy2 0 0 640 480 60", "1080p yuy2 0 0 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_Deinterlace_fps_30(self):
        self.check_scale_TP_with_fps(["1080p yuy2 1 1 1280 720 60", "1080p yuy2 1 1 240 135 60", "1080p yuy2 1 1 3840 2160 60",
                                      "1080p yuy2 1 1 640 480 60", "1080p yuy2 1 1 1920 1080 60"])

    def test_Camera_Scale_TP_YUYV_720x576_Interlace_Deinterlace_fps_25(self):
        self.check_scale_TP_with_fps(["576p yuy2 1 1 640 480 50", "576p yuy2 1 1 720 576 50", "576p yuy2 1 1 1280 720 50", "576p yuy2 1 1 1920 1080 50"])

    def test_Camera_Scale_TP_YUYV_720x480_Interlace_Deinterlace_fps_30(self):
        self.check_scale_TP_with_fps(["480p yuy2 1 1 640 480 60", "480p yuy2 1 1 1280 720 60", "480p yuy2 1 1 1920 1080 60", "480p yuy2 1 1 720 480 60"])

    def test_Camera_Scale_TP_YUYV_720x576_Progressive_fps_25(self):#delete
        self.check_scale_TP_with_fps(["576p yuy2 0 0 640 480 50", "576p yuy2 0 0 720 576 50", "576p yuy2 0 0 1280 720 50", "576p yuy2 0 0 1920 1080 50"])

    def test_Camera_Scale_TP_YUYV_1280x720_Progressive_fps_30(self):
        self.check_scale_TP_with_fps(["720p yuy2 0 0 1920 1080 30", "720p yuy2 0 0 3840 2160 30", "720p yuy2 0 0 640 480 30", "720p yuy2 0 0 1280 720 30"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Progressive_fps_30(self):
        self.check_scale_TP_with_fps(["1080p yuy2 0 0 1280 720 30", "1080p yuy2 0 0 240 135 30", "1080p yuy2 0 0 3840 2160 30",
                                      "1080p yuy2 0 0 640 480 30", "1080p yuy2 0 0 1920 1080 30"])

    def test_Camera_Scale_TP_YUYV_720x480_Interlace_Deinterlace_fps_15(self):#delete
        self.check_scale_TP_with_fps(["480p yuy2 1 1 640 480 30", "480p yuy2 1 1 1280 720 30", "480p yuy2 1 1 1920 1080 30", "480p yuy2 1 1 720 480 30"])

    def test_Camera_Scale_TP_YUYV_720x576_Interlace_Deinterlace_fps_15(self):#delete
        self.check_scale_TP_with_fps(["576p yuy2 1 1 640 480 30", "576p yuy2 1 1 720 576 30", "576p yuy2 1 1 1280 720 30", "576p yuy2 1 1 1920 1080 30"])

    def test_Camera_Scale_TP_YUYV_1920x1080_Interlace_Deinterlace_fps_15(self):#delete
        self.check_scale_TP_with_fps(["1080p yuy2 1 1 1280 720 30", "1080p yuy2 1 1 240 135 30", "1080p yuy2 1 1 3840 2160 30",
                                      "1080p yuy2 1 1 640 480 30", "1080p yuy2 1 1 1920 1080 30"])



    def test_Camera_Scale_AVM737_720x480_to_1280x720_50fps(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 1280 720 50"])

    def test_Camera_Scale_AVM737_720x480_to_1920x1080_50fps(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 1920 1080 50"])

    def test_Camera_Scale_AVM737_720x480_to_640x480_50fps(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 640 480 50"])

    def test_Camera_Scale_AVM737_720x480_to_720x480_50fps(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 720 480 50"])

    def test_Camera_Scale_AVM737_720x480_to_176x144_50fps(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 176 144 50"])

    def test_Camera_Scale_AVM737_720x480_to_320x240_50fps(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 320 240 50"])

    def test_Camera_IVI_RVC_RunTime_Preview_Smooth(self):
        self.check_scale_TP_with_fps(["480p uyvy 1 1 1280 720 50"])