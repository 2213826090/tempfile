import time
from nose.tools import assert_equals
from testlib.util.common import g_common_obj

class TTS_Impl():

    def __init__(self):
        self.d = g_common_obj.get_device()

    def play_tts_example(self):
        setting_package = "com.android.settings"
        setting_activity = ".Settings"
        g_common_obj.launch_app_am(setting_package,setting_activity)
        self.d(resourceId = "com.android.settings:id/dashboard").scroll.vert.to(text = "Accessibility")
        self.d(text = "Accessibility").click()
        self.d(text = "Text-to-speech output").click()
        for i in range(7):
            self.d(resourceId = "com.android.settings:id/tts_engine_settings").click()
            self.d(text = "Language").click()
            self.d(resourceId = "android:id/text1",index = i).click()
            self.d.press.back()
            self.d(text = "Listen to an example").click()
            time.sleep(4)
        for i in range(4):
            self.d.press.back()
