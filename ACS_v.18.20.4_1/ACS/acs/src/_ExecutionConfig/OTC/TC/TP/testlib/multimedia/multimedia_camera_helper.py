# coding: UTF-8
import os

from testlib.camera.CameraCommon import CameraCommon

from testlib.util.log import Logger
logger = Logger.getlogger()

class MultiMediaCameraHelper:
    
    def __init__(self, cfg_file=""):
        if cfg_file != "":
            self.cfg_file = cfg_file
        else:
            self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), "..", "Multimedia_Camera", 'tests.testplan.camera.conf')

        self.ArcSoftCamera_flag = 0
        self.switchplatform()

    def getNeedChangeResolutionDict(self):
        if "need_change_resolution_dict" not in dir(self):
            self.need_change_resolution_dict = {"SD 480p" : "480p",
                                                "HD 720p" : "720p"}
        return self.need_change_resolution_dict

    def changeResolution(self, resolution):
        if self.ArcSoftCamera_flag == 1:
            t_dict = self.getNeedChangeResolutionDict()
            if resolution in t_dict:
                return t_dict[resolution]
        return resolution

    def switchplatform(self):
        self.camera = CameraCommon().switchPlatform(self.cfg_file)
#         self.multimedia_camera_config = self.config.read(self.cfg_file, "multimedia_camera")
#         t_camera_common = CameraCommon()
#         platform = t_camera_common.getPlatform()
#         logger.debug("===platform is %s" % platform)
#         aosp = self.multimedia_camera_config.get("aosp").split(';')
#         arcsoft = self.multimedia_camera_config.get("arcsoft").split(';')
#         for i in aosp:
#             if self.multimedia_camera_config.get(i.lower()) in platform:
#                 self.camera = AOSPCamera(self.multimedia_camera_config)
#                 logger.debug("new aosp camera successfully")
#                 return
#         for i in arcsoft:
#             if self.multimedia_camera_config.get(i.lower()) in platform:
#                 self.camera = AOSPCamera(self.multimedia_camera_config)
#                 MultiMediaSetting(self.cfg_file).install_apk("quickpic_apk")
#                 logger.debug("new arcsoft camera successfully")
#                 return
#         xmlname = t_camera_common.getPlatformName()
#         print "xmlname=" + xmlname
#         if xmlname.upper() in aosp:
#             self.camera = AOSPCamera(self.multimedia_camera_config)
#             logger.debug("fronm xml file new aosp camera successfully")
#         elif xmlname.upper() in arcsoft:
#             self.camera = ArcSoftCamera(self.multimedia_camera_config)
#             logger.debug("from xml file new arcsoft camera successfully")
#             self.multimedia_setting.install_apk("quickpic_apk")
#         else:
#             self.camera = GMSCamera(self.multimedia_camera_config)

    def CameraRecord(self, time, mode="Camera", lens="Back"):
        try:
            CameraCommon().setOrientationToVertical()
            self.camera.startCameraApp()
            self.camera.selectMode(mode)
            self.camera.switchRearOrFront(lens)
            self.camera.recordVideo(1, time)
        except Exception as e:
            CameraCommon().checkCameraCrash()
            self.assertTrue(False, e)