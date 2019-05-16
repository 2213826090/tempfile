# coding: UTF-8
import os
import time
from testlib.util.common import g_common_obj

class SettingsImpl:
    """
    Implements Settings app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.serial = self.d.server.adb.device_serial()
        self.rotationStack = []

    def launch_settings(self):
        '''
        Launch Settings app.
        '''
        print "[Info] ---Launch Settings app."
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        for i in range(10):
            if self.d(text="Settings").exists:
                break
            time.sleep(1)
        assert self.d(text="Settings").exists

    def set_screen_lock(self, option, curpasswd="abcd", newpasswd="abcd"):
        '''
        Set screen lock.
        '''
        print "[Info] ---Set screen lock %s." % option
        if not self.d(text="Screen lock").exists:
            if not self.d(text = "Security").exists:
                self.d().scroll.vert.to(text = "Security")
            self.d(text = "Security").click.wait()
        time.sleep(1)
        if not self.d(text="Screen lock").exists:
            self.d().scroll.vert.to(text="Screen lock")
        self.d(text = "Screen lock").click.wait()
        if self.d(resourceId = "com.android.settings:id/password_entry").exists:
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(curpasswd)
            self.d(text = "Continue").click.wait()
        self.d(text = option).click.wait()
        if option == "Password":
            if self.d(text="No thanks").exists:
                self.d(text = "No thanks").click.wait()
                self.d(text = "Continue").click.wait()
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(newpasswd)
            self.d(text = "Continue").click.wait()
            self.d(resourceId = "com.android.settings:id/password_entry").set_text(newpasswd)
            self.d(text = "OK").click.wait()
            self.d(text = "Done").click.wait()
        assert self.d(text="Screen lock").down(text=option) != None
        self.d.press.back()

    def lock_screen(self):
        '''
        Lock screen.
        '''
        print "[Info] ---Lock screen."
        self.d.sleep()
        time.sleep(2)
        self.d.wakeup()
        for i in range(5):
            time.sleep(2)
            if self.d(description="Unlock").exists:
                return
        assert self.d(description="Unlock").exists

    def unlock_screen(self, passwd="abcd", status=True):
        '''
        Unlock screen.
        '''
        print "[Info] ---Unlock screen."
        self.d.wakeup()
        if self.d(description="Unlock").exists:
            self.d(description="Unlock").drag.to(resourceId = "com.android.systemui:id/clock_view")
        assert not self.d(description="Unlock").exists
        if self.d(resourceId = "com.android.systemui:id/passwordEntry").exists:
            self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
            self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(passwd)
            self.d.click(700,1000)
            if status:
                assert not self.d(resourceId = "com.android.systemui:id/passwordEntry").exists
            else:
                assert self.d(resourceId = "com.android.systemui:id/passwordEntry").exists \
                        or self.d(textStartsWith = "You have incorrectly typed your password").exists

    def verify_cannot_input_chars(self):
        '''
        Verify cannot input chars.
        '''
        print "[Info] ---Verify cannot input chars."
        chars="a1A!"
        if self.d(text = "OK").exists:
            self.d(text = "OK").click.wait()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").click()
        self.d(resourceId = "com.android.systemui:id/passwordEntry").set_text(chars)
        self.d.click(700,1000)
        assert self.d(textStartsWith = "Try again in").exists
        assert not self.d(text = "Wrong Password").exists

    def set_wallpaper_randomly(self):
        '''
        Set wallpaper randomly.
        '''
        from random import randrange
        self.d(text = "Display").click.wait()
        self.d(text = "Wallpaper").click.wait()
        self.d(text = "Live Wallpapers").click.wait()
        i = randrange(5)
        print "[Info] ---Set wallpaper index %d." % i
        self.d(packageName = "com.android.wallpaper.livepicker", index=i).click.wait()
        self.d(text = "Set wallpaper").click.wait()
        assert not self.d(text = "Set wallpaper").exists
        self.d.press.back()
        self.d.press.back()

    def set_airplane_mode(self, status="OFF"):
        '''
        Set airplane mode status.
        '''
        print "[Info] ---Set airplane mode status %s." % status
        if not self.d(text = "Airplane mode").exists:
            self.d(textStartsWith = "More").click.wait()
        if self.d(resourceId = "android:id/switchWidget").exists:
            if not self.d(resourceId = "android:id/switchWidget").text == status:
                self.d(resourceId = "android:id/switchWidget").click.wait()
            assert self.d(resourceId = "android:id/switchWidget", text=status, enabled=True).wait.exists(timeout=60000),\
              "switch to %s failed"%status
        elif self.d(resourceId = "android:id/switch_widget").exists:
            if not self.d(resourceId = "android:id/switch_widget").text == status:
                self.d(resourceId = "android:id/switch_widget").click.wait()
            assert self.d(resourceId = "android:id/switch_widget", text=status, enabled=True).wait.exists(timeout=60000),\
              "switch to %s failed"%status


    def enter_daydream_settings(self):
        '''
        Enter daydream settings.
        '''
        print "[Info] ---Enter daydream settings."
        if self.d(text="Display").exists:
            self.d(text="Display").click.wait()
        if self.d(text="Daydream").exists:
            self.d(text="Daydream").click.wait()

    def start_daydream(self):
        '''
        Start daydream.
        '''
        print "[Info] ---Start daydream."
        if self.d(text="OFF",className="android.widget.Switch").exists:
            self.d(className="android.widget.Switch").click.wait()
        assert self.d(text="ON",className="android.widget.Switch").exists
        self.d.press.menu()
        self.d(text="When to daydream").click.wait()
        self.d(text="While charging").click.wait()
        self.d(text="Colors").click.wait()
        self.d.press.menu()
        self.d(text="Start now").click.wait()
        time.sleep(1)
        assert self.d(packageName="com.android.dreams.basic").exists

    def stop_daydream(self):
        '''
        Stop daydream.
        '''
        print "[Info] ---Stop daydream."
        self.d.press.menu()
        assert not self.d(packageName="com.android.dreams.basic").exists
        if self.d(text="ON",className="android.widget.Switch").exists:
            self.d(className="android.widget.Switch").click.wait()
        assert self.d(text="OFF",className="android.widget.Switch").exists

    def add_and_switch_user(self):
        '''
        Add and switch to new user.
        '''
        print "[Info] ---Add and switch to new user."
        if not self.d(text="Owner").exists:
            self.d(text="Users").click.wait()
        self.d(text="Add user or profile").click.wait()
        self.d(text="User").click.wait()
        self.d(text="OK").click.wait()
        self.d(text="Set up now").click.wait()

    def go_through_guideline(self, name):
        '''
        Go through guideline.
        '''
        print "[Info] ---Go through guideline."
        self.d(description="Start").click.wait()
        self.d(text="Skip").click.wait()
        self.d(text="Skip anyway").click.wait()
#        self.d(text="Next").click.wait()
        os.system("adb -s %s shell input text '%s'" % (self.serial, name))
        time.sleep(1)
        self.d.press.back()
        time.sleep(1)
        self.d(text="Next").click.wait()
        self.d(text="More").click.wait()
        self.d(text="Next").click.wait()
        self.d(text="Finish").click.wait()
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()
        while self.d(text="OK").exists:
            self.d(text="OK").click.wait()
        if self.d(text="GOT IT").exists:
            self.d(text="GOT IT").click.wait()
        assert self.d(description="Apps").exists and not self.d(text="GOT IT").exists

    def switch_to_owner(self):
        '''
        Switch to owner.
        '''
        print "[Info] ---Switch to owner."
        for i in range(10):
            time.sleep(1)
            if self.d(text="Owner").exists:
                break
            self.d(text="Users").click.wait()
        assert self.d(text="Owner").exists
        self.d(text="Owner").click.wait()

    def delete_user(self, name):
        '''
        Delete user.
        '''
        print "[Info] ---Delete user %s." % name
        if not self.d(text="Owner").exists:
            self.d(text="Users").click.wait()
        self.d(text=name).right(description="Delete user").click.wait()
        self.d(text="Delete").click.wait()
        assert not self.d(text=name).exists

    def rotate_screen(self):
        '''
        Rotate screen.
        '''
        #2 doesn't work in uiautomator
        orientation_list = ["natural", "left", "right"]
        print "[Info] ---Rotate screen"
        for each in orientation_list:
            self.d.orientation = each
            time.sleep(1)
            assert self.d.orientation == each

    def set_orientation_nature(self):
        '''
        Set orientation nature.
        '''
        print "[Info] ---Set orientation nature"
        self.d.orientation = 'n'

    def check_home_screen(self):
        '''
        Check home screen.
        '''
        assert self.d(description="Apps").exists

    def pushRotation(self):
        self.rotationStack.append(self.d.orientation)

    def popRotation(self):
        self.d.orientation = self.rotationStack.pop()
