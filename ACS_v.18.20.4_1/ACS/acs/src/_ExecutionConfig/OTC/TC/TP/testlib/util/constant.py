'''
Created on Jun 26, 2014

@author: yongga1x
'''
import logging

caseName = None
MODULE = None
Log = logging
LogPath = None
RUN_TIME = None
DEVICE_SERIAL = None
Device = None
device_storage_limited = 300
ANDROID_VERSION = "L"
BUILD_TYPE = "mrd7_150"
PAD_TYPE = "Anchor8"
timeout = 7200


PKG_CAMERA=""

CAMERA_TYPE_FRONT = "Front camera"
CAMERA_TYPE_BACK = "Back camera"
STORAGE_LOCATION_BUILT_IN = r"/storage/emulated/0"
STORAGE_LOCATION_EXTRAPOSITION = r"/storage/emulated/0"
camera_pkg_name="com.google.android.GoogleCamera"
camera_activity_name="com.android.camera.CameraActivity"
gallery_pkg_name="com.google.android.gallery3d"
gallery_activity_name="com.android.gallery3d.app.GalleryActivity"
photo_pkg_name = "com.google.android.apps.plus"

REFRESH_SDCARD = "adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://sdcard"
REFRESH_EXTERNAL_SDCARD = "adb shell am broadcast -a android.intent.action.MEDIA_MOUNTED --ez read-only false -d file://storage/sdcard1"
DELETE_DCIM_CAMERA = "adb shell rm -rf /sdcard/DCIM/Camera/*"
LOCK_UNLOCK_SCREEN = "adb shell input keyevent 82"
ON_OFF_SCREEN = "adb shell input keyevent 26"
CLEAN_OTC_TEST_DATA = "adb shell rm -rf /sdcard/otctest"
CLEAN_CAMERA_DATA = "adb shell rm -rf /sdcard/DCIM"

ON_OFF_CPU = "adb shell \"root on; echo 1 > /sys/devices/system/cpu/cpu1/online \""

CAMERA_FRONT_BACK_VERT = (488 , 1002)
CAMERA_MENU_SETTING_VERT = (398 , 1012)

#resolution
RESOLUTION_HD_720P = "HD 720p"
RESOLUTION_SD_480P = "SD 480p"
RESOLUTION_HD_1080P = "HD 1080p"

time_lapse_number = [
              "0.5",
              "1",
              "1.5",
              "2",
              "2.5",
              "3",
              "4",
              "5",
              "6",
              "10",
              "12",
              "15",
              "24"
              ]

time_lapse_unit = [
                   "seconds",
                   "minutes",
                   "hours"
                   ]