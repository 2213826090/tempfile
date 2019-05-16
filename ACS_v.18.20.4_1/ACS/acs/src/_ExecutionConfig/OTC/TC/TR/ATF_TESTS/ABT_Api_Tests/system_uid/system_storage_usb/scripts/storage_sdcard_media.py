from time import sleep
import os.path
import sys

from adb_helper.adb_utils import *
from test_utils import *
if os.getenv('ACS_EXECUTION_CONFIG_PATH') is not None:
    sys.path.insert(0, (os.getenv('ACS_EXECUTION_CONFIG_PATH') + '/TC/TP'))
else:
    sys.path.insert(0,(os.getcwd() + '/_ExecutionConfig/OTC/TC/TP'))
from testlib.util.common import g_common_obj
from testlib.androidframework.dut_manager import dut_manager
from testlib.androidframework.common import *


class Storage_sdcard_media_check(unittest.TestCase):
    '''
    Description: Check for SD card data after reboot and media files.

    Usage: Functionality checking with media content in sdcard

    To do: AdbUtil has to be changed to g_common_obj implementation
    '''

    TAG = "Storage.USB_SDCard_Check.ImagesAndVideo.files.display.in.Photos.app.after.boot"

    def setUp(self):
        print self.TAG, "setUp"
        self.d = g_common_obj.get_device(dut_manager.active_uiautomator_device_serial)

    def tearDown(self):
        print self.TAG, "tearDown"

    def test_sdcard_media_check(self):

        img_file = "wallpaper_3264x2448.jpeg"
        vid_file = "BBB_480p_1450Kbps_audio_44100_30fps_MP.mp4"
        cmd_1 = "pm clear com.android.settings"
        cmd_2 = "am start com.android.settings"
        storage = self.d(textStartsWith="Storage")
        portable = self.d(textContains="Portable")

        AdbUtils.run_adb_cmd(cmd_1)
        AdbUtils.run_adb_cmd(cmd_2)
        storage.click()

        if not portable.exists:
            self.assertTrue("SDcard storage as portable not available")

        sd_path = self.sd_card_path()
        print "SD Card Storage Path: ",sd_path
        if not sd_path:
            self.assertTrue("SD Card not mounted")

        cur_dir = os.getcwd()

        cmd_5 = "rm -rf {}/*".format(sd_path)
        AdbUtils.run_adb_cmd(cmd_5)

        vid_dir = os.path.join(os.path.expanduser("~/"), ".acs/Artifacts/VIDEO/480p")
        os.chdir(vid_dir)
        cmd_3 = "push {0} {1}".format(vid_file,sd_path)
        AdbUtils.run_adb_cmd(cmd_3,adb_shell=False)

        img_dir = os.path.join(os.path.expanduser("~/"), ".acs/Artifacts/IMAGE/JPEG")
        os.chdir(img_dir)
        cmd_4 = "push {0} {1}".format(img_file,sd_path)
        AdbUtils.run_adb_cmd(cmd_4,adb_shell=False)

        os.chdir(cur_dir)

        cmd_6 = "reboot"
        AdbUtils.run_adb_cmd(cmd_6)
        sleep(180)

        cmd_7 = "input keyevent 82"
        AdbUtils.run_adb_cmd(cmd_7)

        AdbUtils.run_adb_cmd(cmd_2)
        storage.click()
        self.d(textContains="Portable").sibling(textContains="SD Card").click()
        for i in img_file,vid_file:
            if not (self.d(text=i).exists):
                self.assertFalse("File Not Found: {}".format(i))

        '''
        cmd_5 = "am start com.google.android.apps.photos"
        AdbUtils.run_adb_cmd(cmd_5)

        if (self.d(scrollable=True).scroll.to(textContains=sd_path.split('/')[-1])):
            self.d(textContains=sd_path.split('/')[-1]).click()
        '''

    def sd_card_path(self):
        mnts = AdbUtils.run_adb_cmd('df')
        lst =  ['/storage/emulated','/storage/self']
        sdp = []
        for line in mnts.split('\n'):
            if '/storage/' in line:
                sdp.append(line.split()[0])
        for i in sdp:
            if i not in lst:
                return i
        return False



