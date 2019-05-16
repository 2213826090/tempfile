#coding=utf8=
from testlib.util.common import g_common_obj
from testlib.util.otc_image import otcImage
import time


class SettingImpl:
    """
        @summary: class for Setting application Home UI
    """

    #--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_next(self):
            """ UI button next """
            return self.d(text="NEXT")

    def __init__(self, cfg=None):
        self.d = g_common_obj.get_device()
        self._locator = SettingImpl.Locator(self.d)
        self.cfg = cfg
        self.dut = self.d

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_setting(self):
        '''
        former name : launchSetting
        '''
        g_common_obj.launch_app_am("com.android.settings", '.Settings')
        assert self.d(text="Bluetooth").exists, "launch app failed"

    def kill_setting(self):
        '''
        former name : killSetting
        '''
        g_common_obj.stop_app_am("com.android.settings")

    def set_wall_page(self, checkWords, imgIndex=0):
        '''
        former name : setWallPage
        the checkWords of wallpage.png is "No photos found"
        '''
#         self.d(text="Display").click()
#         time.sleep(1)
#         self.d(text="Wallpaper").click()
#         time.sleep(1)
#         self.d(text="Photos").click()
#         time.sleep(1)
#         self.d(resourceId="com.google.android.apps.plus:id/tile_row").child(index=imgIndex).click()
#         time.sleep(1)
#         self.d(text="Set wallpaper").click()
#         time.sleep(1)
#         self.killSetting()
#         time.sleep()
#         wallImg = otcImage.getWidgetImage(self.d, self.d(resourceId="android:id/content"))
#         words = str(otcImage.analysisImageToString(wallImg)).replace(" ", "").replace("\n", "")
#         print "words is ", words
#         assert checkWords in  words, "analysisImageToString is " + str(words) + " Expected value is " + checkWords
        pass

    def clean_up_app_data(self, appName):
        '''
        former name : CleanUpAppData
        '''
        self.dut.press.home()
        g_common_obj.launch_app_am("com.android.settings", '.Settings')
        self.d(text="Apps").click()
        iffind = False
        for i in range(10):
            if self.d(text="All").exists:
                self.d(text="All").click()
                iffind = True
                break
            self.d(scrollable=True).scroll.horiz()
        assert iffind == True
        iffind = False
        for i in range(100):
            if self.d(text=appName).exists:
                self.d(text=appName).click()
                iffind = True
                break
            self.d(scrollable=True).scroll.vert()
        assert iffind == True
        self.d(text="Clear data").click()
        try:
            self.d(text="OK").click()
        except:
            print "there' s no prompt window"
        self.d(text="Force stop").click()
        try:
            self.d(text="OK").click()
        except:
            print "there' s no prompt window"
        for i in range(5):
            self.d.press.back()

    def enter_photo_plus_from_storage_setting(self):
        self.launchSetting()
        self.d(text="Storage").click()
        time.sleep(3)
        self.d(text="Pictures, videos").click()
        time.sleep(3)
        assert self.d(text="Photos").exists, "enter photo plus from storage setting failed"

    def recent_app(self):
        '''
        former name : recentApp
        '''
        self.d.press.recent()
        time.sleep(2)
        reID = "com.android.systemui:id/task_view_content"
        for _ in range(3):
            if self.d(resourceId=reID).exists:
                break
            time.sleep(2)
        assert self.d(resourceId=reID).exists, "enter recents container failed"

    def remove_recent_app(self, appName):
        '''
        former name : removeRecentApp
        '''
        x = self.d.info["displayWidth"]
        if not self.d(text=appName).exists:
            return False, "not found " + str(appName)
        bounds = self.d(text=appName).bounds
        print bounds
        self.d.swipe(bounds.get("left"), bounds.get("top"), x-50, bounds.get("top"))
        time.sleep(2)
        assert not self.d(text=appName).exists, "remove recent app " + str(appName) + "failed"

    def click_recent_app(self, appName):
        '''
        former name : clickRecentApp
        '''
        assert self.d(text=appName).exists, "not found " + str(appName)
        bounds = self.d(text=appName).bounds
        print bounds
        self.d.click(bounds.get("left"), bounds.get("top"))
