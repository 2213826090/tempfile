import time
from testlib.audio import resource
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2


class RecorderImpl(object):
    pkgname = "com.intel.psi_recorder"
    apk = "newPSI_Recorder.apk"

    def __init__(self, cfg=None):
        self._adb = g_common_obj
        self.d = self._adb.get_device()

    def installed(self):
        ''' check if recorder is installed '''
        cmd = "pm list package %s" % self.pkgname
        ret = self._adb.adb_cmd_capture_msg(cmd)
        if self.pkgname in ret:
            return True
        else:
            return False

    def install(self, app='', force=False):
        '''
        install recorder app if not installed.
        if 'force' is 'True', always install the app.
        '''
        if (not force and self.installed()):  # already installed
            return
        if not app:
            app = resource.get_app(self.apk)
        cmd = "install -r %s" % app
        self._adb.adb_cmd_common(cmd)

    def launch(self):
        '''
        launch recorder
        '''
        activity = ".PSI_Recorder"
        self._adb.launch_app_am(self.pkgname, activity)
        time.sleep(5)

    def record(self, duration=5):
        '''
        record sound for a period of time
        '''
        time.sleep(3)
        self._click_btn("Record")
        time.sleep(duration)
        self._unlock()
        self._click_btn("Stop")

    def start_record(self):
        '''
        start record
        '''
        self._click_btn("Record")

    def stop(self):
        '''
        stop record/playback
        '''
        self._unlock()
        self._click_btn("Stop")

    def playback(self, playtime=5):
        ''' play the record file '''
        self._unlock()
        self._click_btn("Playback")
        time.sleep(playtime)
        self._unlock()  # in case, screen locked
        self._click_btn("Stop")

    def delete(self):
        ''' delete the record file '''
        self._unlock()
        self._click_btn("Delete")

    def quit(self):
        ''' quit the recorder '''
        self._unlock()
        self.d.press.menu()
        time.sleep(2)
        self.d(text="Exit").click()

    def force_stop(self):
        '''
        force kill recorder
        '''
        self._adb.adb_cmd("am force-stop " + self.pkgname)

    def configure(self, cfg):
        '''
        configure the parameters from 'Perference'

        'cfg' is a dict contains parameters
        parameter guide, take 'Sampling Rate' for example:
            the UI shows like "Sampling Rate: 44100Hz",
            the dict member name should be 'Sampling Rate'
            (name is string before ':')
            value is '44.1' (the option string of pop up menu)
            > recorder.configure(Sampling_Rate='44.1')
        '''
        # enter perference page
        self._unlock()
        self.d.press.menu()
        self.d(text="Preferences").click()

        def do_cfg(key, val):
            item = key.strip() + ':'
            self.d(textContains=item).click()
            # will pop up menu
            self.d(text=val.strip()).click()
            time.sleep(1)

        # 'Codecs' should be set first, otherwise, some item might be inactive
        key = 'Codecs'
        if key in cfg:
            val = cfg.pop(key)
            do_cfg(key, val)
        for key, val in cfg.items():
            do_cfg(key, val)

        self.d.press.back()
        time.sleep(1)

    def _click_btn(self, name):
        resid = "%s:id/b%s" % (self.pkgname, name)
        if self.d(resourceId=resid).exists:
            self.d(resourceId=resid).click()
        else:
            assert("button %s not exists" % resid)

    def _unlock(self):
        ''' unlock screen '''
        g_common_obj2.unlock()
