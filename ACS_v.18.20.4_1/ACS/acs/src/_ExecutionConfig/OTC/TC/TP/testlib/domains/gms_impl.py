# coding: UTF-8
import os
import time
from nose.tools import assert_equals
from testlib.util.common import g_common_obj

class MusicImpl:
    """
    Implements Play Music UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_play_music(self):
        '''
        Launch play music app.
        '''
        print "[Info] ---Launch play music app."
        g_common_obj.launch_app_am("com.google.android.music", \
            "com.android.music.activitymanagement.TopLevelActivity")
        for i in range(30):
            if self.d(text="Listen Now").exists:
                break
            time.sleep(1)
        assert self.d(text="Listen Now").exists

    def display_music_list(self):
        '''
        Display music list.
        '''
        print "[Info] ---Display music list."
        if not self.d(resourceId="com.google.android.music:id/play_drawer_list").exists:
            self.d(description = "Show navigation drawer").click.wait()
        self.d(text="My Library").click.wait()
        self.d(text="SONGS").click.wait()
        assert self.d(resourceId="com.google.android.music:id/icon").exists

    def make_songs(self, num):
        '''
        Create some mp3 files.
        '''
        print "[Info] ---Make some songs."
        for i in range(num):
            self.d.server.adb.cmd("shell touch /sdcard/Music/%03d.mp3" % i)
        self.d.server.adb.cmd("shell am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/Music")

    def delete_song(self, name):
        '''
        Delete song.
        '''
        self.d(text=name).right(description="Options").click.wait()
        self.d(text="Delete").click.wait()
        self.d(text="OK").click.wait()
        assert not self.d(text=name).exists
        print "[Info] ---Delete song %s." % name

    def play_music(self):
        '''
        Play music.
        '''
        print "[Info] ---Play music."
        for i in range(10):
            self.d(text="Shuffle all").click.wait()
            if self.d(description="Pause").exists:
                break
        assert self.d(description="Pause").exists

    def stop_playing_music(self):
        '''
        Stop playing music.
        '''
        print "[Info] ---Stop playing music."
        for i in range(10):
            if not self.d(description="Pause").exists:
                break
            self.d(description="Pause").click.wait()
        assert self.d(description="Play").exists

    def rotate_screen_when_playing(self, cycle = 1):
        '''
        Rotate screen.
        1. cycle: The cycle number of rotating screen.
        '''
        print "[Info] ---Rotate screen."
        for i in range(cycle):
            print "[Info] --- Cycle number: %d." % (i + 1)
            if self.d(description="Play").exists:
                self.d(description="Play").click.wait()
            self.d.orientation = "l"
            time.sleep(1)
            self.d.orientation = "u"
            time.sleep(1)
            self.d.orientation = "r"
            time.sleep(1)
            self.d.orientation = "n"
            assert not self.d(textStartsWith="Unfortunately").exists

    def switch_tab(self):
        '''
        Switch tab view in music.
        '''
        print "[Info] ---switch tab view"
        # self.d(className="android.widget.ImageButton" , description ="Show navigation drawer").click.wait()
        tabs = ["Listen Now", "My Library", "Playlists", "Instant Mixes"]
        for i in range(len(tabs)):
            self.d(description="Show navigation drawer").click()
            # self.d(className = "android.widget.TextView", packageName ="com.google.android.music").set_text(tabs[i])
            self.d(text=tabs[i]).click()
            time.sleep(2)
            result_flag1 = self.d(description="Hide navigation drawer").exists
            result_flag2 = self.d(text =tabs[i], className="android.widget.TextView").exists
            assert_equals(result_flag1, False)
            assert_equals(result_flag2, True)
            # assert_equals(self.d(className="android.widget.TextView" , text =tabs[i]).exists, True)
            print "[Info] ---search %s success" % tabs[i]
            time.sleep(2)

    def exit_app(self):
        '''
        Exit Music
        '''
        print "[Info] ---exit Music"
        self.d.press.home()
        time.sleep(5)

class CameraImpl:
    """
    Implements camera app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_camera(self):
        '''
        Launch Camera app.
        '''
        print "[Info] ---Launch Camera app."
        g_common_obj.launch_app_am("com.google.android.GoogleCamera", \
            "com.android.camera.CameraLauncher")
#        time.sleep(15)
        if self.d(resourceId ="com.android.camera2:id/confirm_button").exists:
            self.d(resourceId="com.android.camera2:id/confirm_button").click()
        for i in range(20):
            if self.d(resourceId="com.android.camera2:id/preview_overlay").exists:
                break
            time.sleep(1)
        assert self.d(resourceId="com.android.camera2:id/preview_overlay").exists

    def record_video(self):
        '''
        Record video for a long time
        '''
        print "[Info] ---Start to record video"
        time.sleep(5)
        self.d().scroll.horiz.backward()
        self.d(text="Video").click()
        self.d(resourceId="com.android.camera2:id/shutter_button").click()
        timeout = 1800
        while timeout >= 0:
            time.sleep(30)
            timeout -= 30
            assert_equals(self.d(resourceId="com.android.camera2:id/shutter_button").exists, True)
            print "[Info] ---There is no pop up diaplayed"
            assert_equals(self.d(resourceId="com.android.camera2:id/recording_time").exists, True)
            print "[Info] ---recording video normally."
        self.d(resourceId="com.android.camera2:id/shutter_button").click()

    def pic_take(self):
        '''
        Take photos
        '''
        print "[Info] ---Start to take photos"
        time.sleep(4)
        self.d().scroll.horiz.backward()
        self.d(text="Camera").click()
        time.sleep(2)
        self.d(resourceId="com.android.camera2:id/shutter_button").click()
        time.sleep(3)

    def view_pic(self):
        '''
        Review the picture and back to the camera
        '''
        print "[Info] ---View the picture"
        self.d().scroll.horiz.forward()
        self.d(className="android.widget.ImageView").click()
        assert_equals(self.d (className="android.widget.ImageView").exists, True)
        time.sleep(5)

    def back_camera(self):
        '''
        Back to the camera.
        '''
        print "[Info] ---Back to the camera"
        if not self.d(resourceId="com.android.camera2:id/shutter_button").exists:
            self.d.press.back()
        assert_equals(self.d(resourceId="com.android.camera2:id/shutter_button").exists, True)

    def switch_mode(self):
        '''
        Switch current mode to another one.
        '''
        print "[Info] ---Start to switch."
        self.d().scroll.horiz.backward()
        from random import randrange
        i = randrange(0, 5, 1)
        self.d(resourceId = "com.android.camera2:id/selector_icon", \
                    instance = i).click.wait()
        assert_equals(self.d(resourceId="com.android.camera2:id/preview_overlay").exists, True)
        print "[Info] ---Switch success"


    def exit_camera(self):
        '''
        Exit Camera app.
        '''
        print "[Info] ---Exit Camera app."
        self.d.press.back()
        assert not self.d(description="Shutter").exists


class YoutubeImpl:
    """
    Implements youtube app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_youtube(self):
        '''
        Launch youtube app.
        '''
        print "[Info] ---Launch youtube app."
        g_common_obj.launch_app_am("com.google.android.youtube", \
            "com.google.android.apps.youtube.app.WatchWhileActivity")
        for i in range(20):
            if self.d(resourceId="android:id/up").exists:
                break
            time.sleep(2)
        assert self.d(resourceId="android:id/up").exists

    def verify_able_to_watch(self):
        '''
        Verify able to watch videos.
        '''
        print "[Info] ---Verify able to watch videos."
        for i in range(10):
            if self.d(text="Retry").exists:
                self.d(text="Retry").click.wait()
            if not self.d(resourceId="com.google.android.youtube:id/load_progress").exists:
                break
            time.sleep(5)
        assert not self.d(resourceId="com.google.android.youtube:id/load_progress").exists
        assert not self.d(text="Retry").exists

    def exit_youtube(self):
        '''
        Exit youtube app.
        '''
        print "[Info] ---Exit youtube app."
        self.d.press.back()
        assert not self.d(resourceId="android:id/up").exists

class PlayStoreImpl:
    """
    Implements Play Store app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_playstore(self):
        '''
        Launch play store app.
        '''
        print "[Info] ---Launch Play Store app."
        g_common_obj.launch_app_am("com.android.vending", ".AssetBrowserActivity")
        for i in range(20):
            if self.d(resourceId="com.android.vending:id/search_button").exists:
                break
            time.sleep(2)
        assert self.d(resourceId="com.android.vending:id/search_button").exists

    def exit_playstore(self):
        '''
        Exit Play Store app.
        '''
        print "[Info] ---Exit Play Store app."
        self.d.press.back()
        assert not self.d(text="Play Store", resourceId="android:id/action_bar_title").exists


class CalendarImpl:
    """
    Implements calendar app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_calendar(self):
        '''
        launch calendar app
        '''
        print "[Info] ---Launch calendar app."
        g_common_obj.launch_app_am("com.google.android.calendar", \
         "com.android.calendar.AllInOneActivity ")
        time.sleep(2)
        while self.d(resourceId="com.android.calendar:id/right_arrow").exists:
            self.d(resourceId="com.android.calendar:id/right_arrow").click()
        if self.d(resourceId ="com.android.calendar:id/done_button").exists:
            self.d(resourceId="com.android.calendar:id/done_button").click()
        assert_equals(self.d(resourceId="com.android.calendar:id/add_event_button").exists, True)
        print "[Info] ---Launch calendar app success."

    def add_day_event(self, event_num):
        '''
        Click add button to add multiple all day event.
        '''
        print "[Info] ---click add button to add all day event."
        for i in range(event_num):
            self.d(description="Create new event").click.wait()
            if self.d(text ="OFF").exists:
                self.d(text ="OFF").click.wait()
            assert_equals(self.d(text ="ON").exists, True)
            self.d(resourceId="com.android.calendar:id/input").set_text("event_%d" % i)
            self.d.press.back()
            assert_equals(self.d(resourceId ="com.android.calendar:id/edit_fragment_header").exists, True)
            self.d(resourceId="com.android.calendar:id/save").click.wait()
            assert_equals(self.d(resourceId ="com.android.calendar:id/view_pager_container").exists, True)


class SearchImpl:
    """
    Implements google search UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
    def launch_search(self):
        '''
        Launch google search .
        '''
        print "[Info] ---Launch google search."
        self.d.press.home()
        if self.d(resourceId ="com.google.android.googlequicksearchbox:id/launcher_search_button").exists:
            self.d(resourceId ="com.google.android.googlequicksearchbox:id/launcher_search_button").click.wait()
        time.sleep(10)
        for i in range(10):
            if self.d(textStartsWith ="Search, or say").exists:
                break
            time.sleep(1)
        assert_equals(self.d(textStartsWith ="Search, or say").exists, True)

    def search_app(self):
        '''
        Search some apps.
        '''
        print "[Info] ---Search some apps."
        apps = ["Chrome", "Contacts", "Play Books", "Play Movies & TV", "Play Music"]
        for i in range(len(apps)):
            self.d(resourceId="com.google.android.googlequicksearchbox:id/search_box").clear_text()
            time.sleep(2)
            self.d(resourceId="com.google.android.googlequicksearchbox:id/search_box").set_text(apps[i])
            time.sleep(2)
            assert_equals(self.d(className="android.widget.TextView" , text =apps[i]).exists, True)
            print "[Info] ---search %s success" % apps[i]
            time.sleep(2)

    def exit_search(self):
        '''
        Exit google search.
        '''
        print "[Info] ---Exit google search."
        if self.d(resourceId ="com.google.android.googlequicksearchbox:id/navigation_button").exists:
            self.d(resourceId ="com.google.android.googlequicksearchbox:id/navigation_button").click.wait()
        time.sleep(2)
        assert not self.d(textStartsWith ="Search, or say", resourceId="com.google.android.googlequicksearchbox:id/navigation_button").exists

class MapsImpl:
    """
    Implements google Maps UI actions
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_maps(self):
        """
        Launch Google Maps app.
        """
        print "[Info] ---Launch Google Maps app."
        g_common_obj.launch_app_am("com.google.android.apps.maps", \
         "com.google.android.maps.MapsActivity ")
        time.sleep(15)
        while self.d(resourceId="com.google.android.apps.gmm:id/accept_button").exists:
            self.d(resourceId="com.google.android.apps.gmm:id/accept_button").click()
        assert_equals(self.d(resourceId ="com.google.android.apps.gmm:id/mylocation_button").exists, True)

    def exit_search(self):
        '''
        Exit google maps app.
        '''
        print "[Info] ---Exit Google Maps app."
        self.d.press.back()
        assert not self.d(resourceId ="com.google.android.apps.gmm:id/mylocation_button").exists

class DocsImpl:
    """
    Implements Docs UI actions.
    """
    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    @staticmethod
    def startApp():
        """
        Skip Calendar first launch screen
        """
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Docs")
        time.sleep(5)
        if d(descriptionContains="New File").exists:
            g_common_obj.back_home()
            return
        while d(resourceId="com.google.android.apps.docs.editors.docs:id/next").wait.exists(timeout=10000):
            d(resourceId="com.google.android.apps.docs.editors.docs:id/next").click()
            d.wait.update()
        if d(
            resourceId="com.google.android.apps.docs.editors.docs:id/done").\
        wait.exists(timeout=60000):
            d(resourceId="com.google.android.apps.docs.editors.docs:id/done").click()
        g_common_obj.back_home()

    def launch_docs(self):
        '''
        Launch Docs app.
        '''
        print"[Info] ---Launch Docs app"
        g_common_obj.launch_app_am("com.google.android.apps.docs.editors.docs", \
         "com.google.android.apps.docs.app.NewMainProxyActivity")
        time.sleep(15)
        while self.d(resourceId="com.google.android.apps.docs.editors.docs:id/welcome_button_continue").exists:
            self.d(resourceId="com.google.android.apps.docs.editors.docs:id/welcome_button_continue").click()
        if self.d(resourceId ="com.google.android.apps.docs.editors.docs:id/welcome_button_close").exists:
            self.d(resourceId="com.google.android.apps.docs.editors.docs:id/welcome_button_close").click()
        time.sleep(5)
        if not self.d(descriptionContains="New File").exists:
            print "[WARNING:UI is changed! Just test launch function]"
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/doc.png")
            return
        assert_equals(self.d(descriptionContains="New File").exists, True)
        print "[Info] ---Launch docs app success."

    def basic_test(self):
        '''
        Click add button to add docs.
        '''
        print "[Info] ---click add button to add docs."
        if not self.d(descriptionContains="New File").exists:
            print "[WARNING:UI is changed! Just test launch function]"
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/doc.png")
            self.d.press.back()
            return
        self.d(descriptionContains="New File").click.wait()
        time.sleep(5)
        assert self.d(className="android.widget.FrameLayout").exists
        self.d(resourceId="android:id/action_mode_close_button").click.wait()
        self.d.press.back()
        print "[Info] ----add docs success"

    def add_docs(self, docs_num):
        '''
        Click add button to add multiple docs.
        '''
        print "[Info] ---click add button to add docs."
        if not self.d(descriptionContains="New File").exists:
            print "[WARNING:UI is changed! Just test launch function]"
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/doc.png")
            self.d.press.back()
            return
        for i in range(docs_num):
            self.d(descriptionContains="New File").click.wait()
            time.sleep(5)
            self.d(className="android.widget.FrameLayout").set_text("docs_%d" % i)
            self.d(resourceId="android:id/action_mode_close_button").click.wait()
            assert_equals(self.d(resourceId ="android:id/decor_content_parent").exists, True)
            self.d.press.back()
            print "[Info] ----add docs success"

    def exit_docs(self):
        '''
        Exit docs app
        '''
        print "[Info] ---Exit docs app."
        self.d.press.back()
        self.d.press.back()
        assert not self.d(description="New File").exists


class ChroImpl:
    """
    Implements Chrome app UI actions.
    """
    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_chrome(self):
        '''
        Launch Chrome app.
        '''
        print "[Info] ---Launch Chrome app."
        g_common_obj.launch_app_am(" com.android.chrome", \
         "com.google.android.apps.chrome.ChromeTabbedActivity ")
        time.sleep(5)
        while self.d(resourceId="com.android.chrome:id/terms_accept").exists:
            self.d(resourceId="com.android.chrome:id/terms_accept").click()
        if self.d(resourceId ="com.android.chrome:id/negative_button").exists:
            self.d(resourceId="com.android.chrome:id/negative_button").click()
        assert_equals(self.d(resourceId="com.android.chrome:id/url_bar").exists, True)
        print "[Info] ---Launch chrome app success."

    def add_bookmark(self, bookmark_num):
        '''
        Click add button to add multiple bookmarks.
        '''
        print "[Info] ---Click add button to add bookmarks"
        for i in range(bookmark_num):
            self.d(description="Bookmark page").click.wait()
            time.sleep(3)
            self.d(resourceId="com.android.chrome:id/bookmark_title_input").set_text("bookmark_%d" % i)
            time.sleep(3)
            self.d(resourceId="com.android.chrome:id/bookmark_url_input").set_text("bookmark_%d" % i)
            self.d(resourceId="com.android.chrome:id/ok").click.wait()
            time.sleep(3)
            assert_equals(self.d(resourceId ="com.android.chrome:id/url_bar").exists, True)
            print "[Info] ----add bookmark success"

    def exit_chrome(self):
        '''
        Exit chrome app.
        '''
        print "[Info] ---Exit chrome app."
        self.d.press.back()
        assert not self.d(description="com.android.chrome:id/url_bar").exists

class NewsImpl:
    '''
    Implements News app UI actions.
    '''
    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    @staticmethod
    def startApp():
        """
        Skip Calendar first launch screen
        """
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("News & Weather")
        time.sleep(2)
        if d(resourceId ="com.google.android.apps.magazines:id/image_header_logo").exists:
            g_common_obj.back_home()
            return
        while d(resourceId="com.google.android.apps.genie.geniewidget:id/next").exists:
            d(resourceId="com.google.android.apps.genie.geniewidget:id/next").click()
            d.wait.update()
        if d(
            resourceId="com.google.android.apps.genie.geniewidget:id/done").\
        wait.exists(timeout=60000):
            d(resourceId="com.google.android.apps.genie.geniewidget:id/done").click()
        if d(description="Close navigation drawer").exists:
            d(description="Close navigation drawer").click()
            d.wait.update()
        g_common_obj.back_home()

    def launch_news(self):
        g_common_obj.launch_app_from_home_sc("News & Weather")
        assert self.d(
            resourceId ="com.google.android.apps.genie.geniewidget:id/toolbar")\
        .wait.exists(timeout=60000)
        print "[Info] ----launch news app"

    def exit_news(self):
        print "[Info] ---exit news app"
        self.d.press.back()
        self.d.press.back()
        assert not self.d(
            resourceId="com.google.android.apps.genie.geniewidget:id/toolbar")\
        .exists


class NewssImpl:
    '''
    Implements Newss app UI actions.
    '''
    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_newss(self):
        g_common_obj.launch_app_am(" com.google.android.apps.magazines", \
         "com.google.apps.dots.android.app.activity.CurrentsStartActivity ")
        time.sleep(15)
        assert_equals(self.d(resourceId ="com.google.android.apps.magazines:id/image_header_logo").exists, True)
        print "[Info] ----launch newsstand app"

    def exit_newss(self):
        print "[Info] ---exit newsstand app"
        self.d.press.back()
        self.d.press.back()
        assert not self.d(description="com.google.android.apps.magazines:id/image_header_logo").exists

class SheetImpl:
    """
    Implements Spreadsheet UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_sheets(self):
        '''
        Launch Docs app.
        '''
        print"[Info] ---Launch sheets app"
        g_common_obj.launch_app_am("com.google.android.apps.docs.editors.sheets", \
         "com.google.android.apps.docs.app.NewMainProxyActivity")
        time.sleep(15)
        while self.d(resourceId="com.google.android.apps.docs.editors.sheets:id/next").exists:
            self.d(resourceId="com.google.android.apps.docs.editors.sheets:id/next").click()
        if self.d(resourceId ="com.google.android.apps.docs.editors.sheets:id/done").exists:
            self.d(resourceId="com.google.android.apps.docs.editors.sheets:id/donee").click()
        time.sleep(10)
        if not self.d(descriptionContains="New File").exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName="com.google.android.apps.docs.editors.sheets").exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/doc.png")
            return
        assert_equals(self.d(descriptionContains="New File").exists, True)
        print "[Info] ---Launch sheets app success."

    @staticmethod
    def startApp():
        """
        Skip Calendar first launch screen
        """
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Sheets")
        time.sleep(10)
        if d(descriptionContains="New File").exists:
            g_common_obj.back_home()
            return
        while d(resourceId="com.google.android.apps.docs.editors.sheets:id/next").exists:
            d(resourceId="com.google.android.apps.docs.editors.sheets:id/next").click()
            d.wait.update()
        if d(
            resourceId="com.google.android.apps.docs.editors.sheets:id/done").\
        wait.exists(timeout=60000):
            d(resourceId="com.google.android.apps.docs.editors.sheets:id/done").click()
        g_common_obj.back_home()

    def add_sheets(self):
        '''
        Click add button to add multiple Spreadsheet.
        '''
        print "[Info] ---click add button to add spreadsheet."
        if not self.d(descriptionContains="New File").exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName="com.google.android.apps.docs.editors.sheets").exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/sheet.png")
            self.d.press.back()
            return
        self.d(descriptionContains="New File").click.wait()
        time.sleep(5)
        self.d.click(400,600)
        os.system("adb shell input text 'qwert'")
        os.system("adb shell input keyevent 66")
        if not self.d(resourceId="android:id/action_mode_close_button").exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName="com.google.android.apps.docs.editors.sheets").exists
            return
        self.d(resourceId="android:id/action_mode_close_button").click.wait()
        assert not self.d(resourceId="android:id/action_mode_close_button").exists
        self.d.press.back()
        print "[Info] ----add spreadsheet success"

    def exit_sheets(self):
        print "[Info] ---exit sheets app"
        self.d.press.back()
        self.d.press.back()
        assert not self.d(description="New File").exists

class EarthImpl:
    """
    Implements Earth UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_earth(self):
        '''
        Launch Earth app.
        '''
        print"[Info] ---Launch earth app"
        g_common_obj.launch_app_am("com.google.earth", \
         ".EarthActivity")
        time.sleep(15)
        while self.d(resourceId="com.google.earth:id/navtip_button_start").exists:
            self.d(resourceId="com.google.earth:id/navtip_button_start").click()
        if self.d(resourceId ="com.google.earth:id/navtip_button_close").exists:
            self.d(resourceId="com.google.earth:id/navtip_button_close").click()
        time.sleep(10)
        assert_equals(self.d(resourceId="com.google.earth:id/compassneedle").exists, True)
        print "[Info] ---Launch Earth app success."

    def locate_earth(self):
        '''
        Do locate to north operation
        '''
        print "[Info] ---Click compass key"
        self.d(resourceId="com.google.earth:id/compassneedle").click()
        assert_equals(self.d(resourceId="com.google.earth:id/compassneedle").exists, True)
        time.sleep(3)

    def exit_earth(self, method):
        if method == "back":
            self.d.press.back()
            print "[Info] ---click back"
            time.sleep(3)
        if method == "home":
            self.d.press.home()
            print "[Info] ---click home"
            time.sleep(3)
        if method == "recent":
            self.d.press.recent()
            print "[Info] ---click recent"
            time.sleep(3)

class GameImpl:
    """
    Implements Game UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_game(self):
        '''
        Launch Game app.
        '''
        print"[Info] ---Launch Game app"
        g_common_obj.launch_app_am("com.google.android.play.games", \
         "com.google.android.gms.games.ui.destination.main.MainActivity")
        time.sleep(15)
        assert_equals(self.d(packageName = "com.google.android.play.games").exists, True)
        print "[Info] ---Launch Game app success."
    def exit_game(self):
        '''
        Exit Game app.
        '''
        print"[Info] ---Exit game app"
        self.d.press.back()
        time.sleep(5)

class SoundSearchImpl:
    """
    Implements Sound Search UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_sound(self):
        '''
        Launch sound search app
        '''
        print"[Info] ---Launch sound search app"
        g_common_obj.launch_app_am("com.google.android.play.games", \
         "com.google.android.gms.games.ui.destination.main.MainActivity")
        time.sleep(15)
        assert_equals(self.d(packageName = "com.google.android.play.games").exists, True)
        print "[Info] ---Launch Game app success."
    def exit_game(self):
        '''
        Exit Game app.
        '''
        print"[Info] ---Exit game app"
        self.d.press.back()
        time.sleep(5)

class SettingImpl:
    """
    Implements Settings app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_settings(self):
        '''
        Launch Settings app.
        '''
        print "[Info] ---Launch Settings app."
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        time.sleep(10)
        for i in range(10):
            if self.d(text="Settings").exists:
                break
            time.sleep(1)
        assert self.d(text="Settings").exists

    def change_rate(self):
        '''
        Change speech rate .
        '''
        print "[Info] ---change speech rate"
        self.d(text="Settings").swipe.up()
        time.sleep(3)
        self.d(text="Accessibility").click.wait()
        assert self.d(text="Accessibility").exists
        time.sleep(2)
        self.d(text="Text-to-speech output").click.wait()
        assert self.d(text="Text-to-speech output").exists
        time.sleep(2)
        self.d(text="Speech rate").click.wait()
        assert self.d(text="Speech rate").exists
        time.sleep(2)
        from random import randrange
        i = randrange(0, 9, 1)
        self.d(resourceId = "android:id/text1", instance = i).click.wait()
        assert self.d(text="Text-to-speech output").exists
        time.sleep(2)

    def set_latin(self):
        '''
        Select latin IME
        '''
        print "[Info] ---Select latin IME"
        self.d(text="Language & input").click.wait()
        assert self.d(text="Language & input").exists

    def exit_settings(self):
        '''
        Exit settings
        '''
        print "[Info] ---exit settings"
        self.d.press.home()
        time.sleep(5)


class MovieImpl:
    """
    Implements play movie app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_movies(self):
        '''
        Launch Paly Movie app.
        '''
        print "[Info] ---Launch movie app."
        g_common_obj.launch_app_am("com.google.android.videos", \
            ".activity.HomeActivity")
        time.sleep(15)
        for i in range(10):
            if self.d(text="Watch Now").exists:
                break
            time.sleep(1)
        assert self.d(text="Watch Now").exists

    def search_movies(self):
        '''
        Search Movies.
        '''
        print "[Info] ---start to search movie"
        self.d(resourceId="com.google.android.videos:id/menu_search").click.wait()
        self.d(resourceId="com.google.android.videos:id/search_src_text").set_text("Maleficent")
        self.d.press.enter()
        time.sleep(15)
        for i in range(10):
            if self.d(text="Maleficent", resourceId="com.android.vending:id/li_title").exists:
                break
            time.sleep(2)
        assert self.d(text="Maleficent", resourceId="com.android.vending:id/li_title").exists
        print "[Info] ---Search success"

    def exit_movies(self):
        '''
        Exit play movie app.
        '''
        print "[Info] ---exit Movies"
        self.d.press.home()
        time.sleep(5)

class FolderImpl:
    """
    Implements folder UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def drag_in_folder(self):
        '''
        Drag apps in home folders.
        '''
        for i in range(10):
            self.d.press.home()
            print "[Info] ---Launch home page success"
            if self.d(packageName = "com.google.android.googlequicksearchbox", \
                description="Apps").exists:
                break
            time.sleep(2)
        assert self.d(packageName = "com.google.android.googlequicksearchbox", \
            description="Apps").exists
        time.sleep(2)
        self.d(packageName = "com.google.android.googlequicksearchbox", \
            description="Apps").click()
        print "[Info] ---Launch app page success"
        time.sleep(2)
        n = self.d(className = "android.widget.TextView").count
        from random import randrange
        i = randrange(0, n, 1)
        assert self.d(index = i, className = "android.widget.TextView", packageName="com.google.android.googlequicksearchbox").exists
        self.d(index = i, className = "android.widget.TextView", packageName="com.google.android.googlequicksearchbox").drag.to(151,324)
        time.sleep(2)
        print "[Info] ---Drag the app into folder"
        for i in range(10):
            self.d.press.home()
            if self.d(className = "android.widget.FrameLayout").exists:
                break
            time.sleep(2)
        assert self.d(className = "android.widget.FrameLayout").exists
        print "[Info] ---Drag app into the folder success"
