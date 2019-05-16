# coding: UTF-8
import os
import time
from testlib.util.common import g_common_obj

class PushBoxImpl:
    """
    Implements Push Box UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.serial = self.d.server.adb.device_serial()

    def install_pushbox_app(self, cfg, option = "pushbox"):
        '''
        Install Push Box app.
        '''
        ret = os.system("adb -s %s shell pm list package | grep com.rafaelwmartins.pushbox" % self.serial)
        if ret == 0:
            print "[Info] ---Push Box already installed."
            return
        from testlib.util.repo import Artifactory
        arti_obj = Artifactory(cfg.get("location"))
        ret_file = arti_obj.get(cfg.get(option))

        print "[Info] ---Install Push Box."
        assert os.path.isfile(ret_file)
        os.system("adb -s %s install -r %s" % (self.serial, ret_file))
        time.sleep(2)
        ret = os.system("adb -s %s shell pm list package | grep com.rafaelwmartins.pushbox" % self.serial)
        assert ret == 0, "Install Push Box failed!"

    def launch_pushbox(self):
        '''
        Launch Push Box.
        '''
        print "[Info] ---Launch Push Box."
        os.system("adb -s %s shell am start -S com.rafaelwmartins.pushbox/.IndexActivity" % self.serial)
        time.sleep(2)

    def resume_play(self):
        '''
        Resume to Push Box.
        '''
        print "[Info] ---Resume to Push Box."
        os.system("adb -s %s shell am start com.rafaelwmartins.pushbox/.IndexActivity" % self.serial)
        time.sleep(2)

    def start_play(self):
        '''
        Start to play.
        '''
        print "[Info] ---Start to play."
        self.d(text="Play").click.wait()
        self.d(text="0 / 30").click.wait()
        self.d(text="1").click.wait()

    def check_play_well(self, timeout = 100):
        '''
        Keep playing.
        '''
        print "[Info] ---Keep playing for %ds." % timeout
        current = time.time()
        while(time.time()-current < timeout):
            self.d(resourceId="com.rafaelwmartins.pushbox:id/board_foreground").swipe.right(steps=10)
            self.d(resourceId="com.rafaelwmartins.pushbox:id/board_foreground").swipe.down(steps=10)
            self.d(resourceId="com.rafaelwmartins.pushbox:id/board_foreground").swipe.left(steps=10)
            self.d(resourceId="com.rafaelwmartins.pushbox:id/board_foreground").swipe.up(steps=10)
            assert self.d(resourceId="com.rafaelwmartins.pushbox:id/board_foreground").exists
            time.sleep(1)

    def stop_playing(self):
        '''
        Stop playing Push Box.
        '''
        print "[Info] ---Stop playing Push Box."
        os.system("adb -s %s shell am force-stop com.rafaelwmartins.pushbox" % self.serial)

