# coding: UTF-8
import os
import time
from testlib.util.common import g_common_obj
from testlib.graphics.tools import ConfigHandle

def verifyImage(filename):
    try:
        from PIL import Image
    except:
        Image = None
    if Image is not None:
        v_image=Image.open(filename)
        v_image.verify()

class TempleRunImpl:
    """
    Implements Temple run 2 UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self.serial = self.d.server.adb.device_serial()

    def install_templerun_app(self, cfg, option = "apk_name"):
        '''
        Install Temple run 2.
        '''
        ret = os.system("adb -s %s shell pm list package | grep com.imangi.templerun2" % self.serial)
        if ret == 0:
            print "[Info] ---Temple run 2 already installed."
            return
        from testlib.util.repo import Artifactory
        config_handle = ConfigHandle()
        cfg["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        arti_obj = Artifactory(cfg.get("location"))
        ret_file = arti_obj.get(cfg.get(option))

        print "[Info] ---Install Temple run 2."
        assert os.path.isfile(ret_file)
        os.system("adb -s %s install -r %s" % (self.serial, ret_file))
        time.sleep(2)
        ret = os.system("adb -s %s shell pm list package | grep com.imangi.templerun2" % self.serial)
        assert ret == 0, "Install Temple run 2 failed!"

    def launch_templerun(self):
        '''
        Launch Temple run 2.
        '''
        print "[Info] ---Launch Temple run 2."
        os.system("adb -s %s shell am start -S com.imangi.templerun2/com.imangi.unityactivity.ImangiUnityNativeActivity" % self.serial)
        time.sleep(8)
        if self.d(text="Cancel").exists:
            self.d(text="Cancel").click.wait()
        time.sleep(8)
        self.d.press("back")

    def press_main_menu(self):
        try:
            from testlib.audio.audio_device import AudioDevice
        except:
            pass
        print "Do swipe to skip user hints screen."
        self._d =  AudioDevice()
        self._d.skip_use_hints()
        print "[Info] ---Press main menu."
        x = self.d.info['displayWidth'] / 2
        y = self.d.info['displayHeight'] - 10
        time.sleep(15)
        self.d.click(x, y)

    def start_play(self):
        print "[Info] ---Start to play."
        x = self.d.info.get("displayWidth")/2
        y = self.d.info.get("displayHeight")-10
        self.d.click(x, y)
        time.sleep(1)

    def stop_playing(self):
        print "[Info] ---Stop playing Temple Run 2."
        os.system("adb -s %s shell am force-stop com.imangi.templerun2" % self.serial)

    def close_wifi(self):
        """ Close wifi
        """
        try:
            cmd = 'svc wifi disable'
            g_common_obj.adb_cmd_capture_msg(repr(cmd))
        except:
            pass

    def open_wifi(self):
        """ Open wifi
        """
        try:
            cmd = 'svc wifi enable'
            g_common_obj.adb_cmd_capture_msg(repr(cmd))
        except:
            pass
