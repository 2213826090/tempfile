import time
from nose.tools import assert_equals
from testlib.util.common import g_common_obj

class Encrypt_Impl():

    def __init__(self):
        self.d = g_common_obj.get_device()

    def decrypt_device(self):
        self.d.sleep()
        self.d.wakeup()
        self.d(resourceId = "com.android.systemui:id/lock_icon").drag.to(resourceId = "com.android.systemui:id/clock_view")
        self.d(resourceId = "com.android.systemui:id/passwordEntry").click.wait()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text("abcd")
        time.sleep(2)
        self.d.click(700,1000)

    def set_password(self):
        setting_package = "com.android.settings"
        setting_activity = ".Settings"
        g_common_obj.launch_app_am(setting_package,setting_activity)
        self.d(text = "Security").click()
        self.d(text = "Screen lock").click()
        self.d(text = "Password").click()
        if self.d(text = "No thanks").exists:
            self.d(text = "No thanks").click()
            self.d(text = "Continue").click()
        self.d(resourceId = "com.android.settings:id/password_entry").set_text("abcd")
        self.d(text = "Continue").click()
        self.d(resourceId = "com.android.settings:id/password_entry").set_text("abcd")
        self.d(text = "OK").click()
        self.d(text = "Done").click()
        self.d.press.back()
        self.d.press.back()
        self.d.press.back()

    def cancel_password(self):
        setting_package = "com.android.settings"
        setting_activity = ".Settings"
        g_common_obj.launch_app_am(setting_package,setting_activity)
        self.d(text = "Security").click()
        self.d(text = "Screen lock").click()
        self.d(resourceId = "com.android.settings:id/password_entry").set_text("abcd")
        self.d(text = "Continue").click()
        self.d(text = "None").click.wait(timeout=15000)
        time.sleep(2)
        if self.d(text="OK").exists:
            self.d(text="OK").click()
        time.sleep(2)
        self.d.press.back()
        self.d.press.back()
        self.d.press.back()
