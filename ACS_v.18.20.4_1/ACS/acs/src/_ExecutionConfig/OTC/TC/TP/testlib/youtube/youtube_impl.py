#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: module for Chrome application
@since: 08/21/2014
@author: Grace yi (gracex.yi@intel.com)
"""

from testlib.util.common import g_common_obj
import time
import os


class YoutubeImpl:
    """
        class for Youtube application Home UI
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_youtube_whattowatch(self):
            """ UI button title What to Watch """
            return self.d(resourceId="android:id/action_bar_title", \
                text="What to Watch")

        @property
        def btn_youtube_whattowatch_in_list(self):
            """ UI button title What to Watch in list"""
            return self.d(resourceId="com.google.android.youtube:id/title", \
                text="What to Watch")

        @property
        def btn_youtube_firstmovie(self):
            """ UI button first movie """
            return self.d(resourceId="com.google.android.youtube:id/thumbnail", instance="2")

        @property
        def btn_choose_account(self):
            """ UI button choose account """
            return self.d(textContains="Choose an account")

        @property
        def btn_first_account(self):
            """ UI button first movie """
            return self.d(\
                resourceId="android:id/text1", instance="0")

        @property
        def btn_account_ok(self):
            """ UI button ok """
            return self.d(textContains="OK", resourceId="android:id/button2")

        @property
        def btn_ok(self):
            """ UI button ok """
            return self.d(resourceId="com.google.android.youtube:id/ok")

        @property
        def btn_next(self):
            """ UI button next """
            return self.d(text = "NEXT")

        @property
        def btn_youtube(self):
            """ UI button youbute """
            return self.d(text = "YouTube")

        @property
        def btn_like(self):
            """ UI button youbute """
            return self.d(resourceId="com.google.android.youtube:id/like_button")


        @property
        def btn_OK(self):
            """ UI button OK """
            return self.d(text = "OK")

        @property
        def btn_gmail(self):
            """ UI button gmail """
            return self.d(text = "Gmail")

        @property
        def btn_send(self):
            """ UI button send """
            return self.d(text = "Send")

        @property
        def btn_video(self):
            """ UI button video """
            return self.d(text = "Video")

        @property
        def btn_upload(self):
            """ UI button upload """
            return self.d(text = "Upload")

        @property
        def btn_uploads(self):
            """ UI button uploads """
            return self.d(text = "Uploads")

        @property
        def btn_delete(self):
            """ UI button delete """
            return self.d(text = "Delete")

        @property
        def btn_shutter(self):
            """ UI button shutter """
            return self.d(resourceId = "com.android.camera2:id/shutter_button")

        @property
        def btn_menu_home(self):
            """ UI button home """
            return self.d(resourceId = "android:id/home")

        @property
        def btn_share(self):
            """ UI button share """
            return self.d(text = "Share")

        @property
        def btn_camera_share(self):
            """ UI button share """
            return self.d(resourceId = "com.android.camera2:id/filmstrip_bottom_control_share")

        @property
        def btn_gmail_delete(self):
            """ UI button share """
            return self.d(resourceId = "com.google.android.gm:id/delete")

        @property
        def btn_choose_gmail_account(self):
            """ UI button choose gmail account"""
            return self.d(resourceId = "android:id/text1")

        @property
        def btn_conversation_list(self):
            """ UI button share """
            return self.d(text = "Conversation list")

        @property
        def btn_video_upload(self):
            """ UI button video upload """
            return self.d(resourceId = "com.google.android.youtube:id/menu_upload_activity_done")

        @property
        def btn_player(self):
            """ UI button player fragment """
            return self.d(\
                resourceId="com.google.android.youtube:id/player_fragment")

        @property
        def btn_menu_anchor(self):
            """ UI button menu anchor """
            return self.d(resourceId="com.google.android.youtube:id/contextual_menu_anchor")

        @property
        def btn_menu_button(self):
            """ UI button menu button """
            return self.d(resourceId = "android:id/button1")

        @property
        def account_name(self):
            """ UI account name """
            return self.d(resourceId="com.google.android.gm:id/from_account_name")

        @property
        def text_send_to(self):
            """ UI text send to """
            return self.d(resourceId = "com.google.android.gm:id/to")

        @property
        def text_gmail_title(self):
            """ UI text send to """
            return self.d(resourceId = "com.google.android.youtube:id/title_edit")

        @property
        def text_recording_time(self):
            """ UI text srecording time """
            return self.d(resourceId = "com.android.camera2:id/recording_time")

        @property
        def text_share_to(self):
            """ UI text share to"""
            return self.d(text = "Share to")

        @property
        def text_share_this_video_via(self):
            """ UI text share this video via"""
            return self.d(text = "Share this video via")

        @property
        def text_title(self):
            """ UI text title """
            return self.d(text = "Title")

        @property
        def text_details(self):
            """ UI text details """
            return self.d(resourceId = "com.google.android.youtube:id/details")

        @property
        def progress_bar(self):
            """ UI text details """
            return self.d(resourceId = "com.google.android.youtube:id/progressbar")

        @property
        def gmail_item(self):
            """ UI gmail item """
            return self.d(packageName = "com.google.android.gm", \
                          index = 1).child(descriptionStartsWith = \
                                           "me about Watch")

        @property
        def gmail_content_title(self):
            """ UI gmail content title """
            return self.d(textMatches = "Watch.* on YouTube.*")

        @property
        def camera_mode(self):
            """ UI camera mode """
            return self.d(resourceId = "com.android.camera2:id/mode_options_overlay")



    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self._locator = YoutubeImpl.Locator(self.d)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self.d.orientation = "n"

    def launch_from_am(self):
        """
            Launch Chrome from am
        """
        print "[INFO] Launch Chrome from am"
        self.d.watcher("SKIP_OK").when(text="OK")\
        .click(text="OK")
        g_common_obj.launch_app_am(self.cfg.get("package_name"), \
            self.cfg.get("activity_name"))
        time.sleep(15)
        if self._locator.btn_choose_account.exists:
            self._locator.btn_first_account.click()
            self._locator.btn_account_ok.click()
            time.sleep(3)
        time.sleep(2)
        if self._locator.btn_youtube_whattowatch_in_list.exists:
            self._locator.btn_youtube_whattowatch_in_list.click()
        assert self._locator.btn_youtube_whattowatch.exists, \
        "[ERROR] Launch fail"
        self.d.watcher("SKIP_OK").remove()

    def stop_from_am(self):
        """
            Stop Chrome from am
        """
        print "[INFO] Stop Chrome from am"
        g_common_obj.stop_app_am(self.cfg.get("package_name"))

    @staticmethod
    def startApp():
    # click ok in youtube
        g_common_obj.launch_app_from_home_sc("YouTube")
        d = g_common_obj.get_device()
        time.sleep(5)
        d.watcher("SKIP_UPDATE").\
        when(resourceId="com.google.android.youtube:id/later_button")\
        .click(resourceId="com.google.android.youtube:id/later_button")
        if d(text="What to Watch").exists:
            g_common_obj.back_home()
            return
        for _ in range(60):
            if d(text="What to Watch").exists and not d(text="OK").exists:
                break
            if d(text="OK").exists:
                d(text="OK").click.wait()
            if d(text="Skip").exists:
                d(text="Skip").click.wait()
            if d(text="Retry").exists:
                d(text="Retry").click.wait()
            time.sleep(2)
        d.press.back()
        if d(description="YouTube").exists:
            d(description="YouTube").click.wait()
            time.sleep(2)
        if d(text="Skip").exists:
            d(text="Skip").click.wait()
        d.watcher("SKIP_UPDATE").remove()
        g_common_obj.back_home()

    def youtube_check_playback(self, timeout=120):
        """
            check youtube playback log in logcat
        """
        key_message = "AudioFocus"
        device_dsn = g_common_obj.globalcontext.device_serial
        if device_dsn:
            cmd = "adb -s " + g_common_obj.globalcontext.device_serial + \
            " logcat -d|grep -i \"" + key_message + "\""
        else:
            cmd = "adb logcat -d|grep -i \"" + key_message + "\""
        print "[Debug] The cmd is %s" % cmd
        pipe = os.popen(cmd).read().strip()
        while len(pipe)==0:
            print "[INFO] The movie is still loading, wait 10s"
            time.sleep(10)
            timeout -= 10
            pipe = os.popen(cmd).read().strip()
            print "[Debug] %s" % pipe
            if timeout <= 0:
                print "[Warning] Did find key log in %d secs" % timeout
                break
        print "[Debug]:", pipe, len(pipe)
        if len(pipe) != 0:
            print "[WARNING: The logcat maybe changed. \
            Just check if the like button is here!]"
            assert self._locator.btn_like.exists
        assert len(pipe) != 0, "ERROR:Search %s is None" % key_message
        print("[INFO] Get the key log in logcat!")

    def youtube_first_movie_play(self, timeout=120):
        """
        @summayr: play the first movie in youtube movie list
        @return: None
        """
        g_common_obj.launch_app_from_home_sc("YouTube")
        if not self._locator.btn_youtube_firstmovie.exists:
            print "[WARNING:UI is changed! Just test launch function]"
            assert self.d(packageName=self.cfg.get("package_name")).exists
            pth = g_common_obj.get_user_log_dir()
            self.d.screenshot(pth + "/youtube.png")

            self.d.press.back()
            return
        while not self._locator.btn_youtube_firstmovie.exists:
            time.sleep(10)
            timeout -= 10
            if timeout <= 0:
                break
        assert self._locator.btn_youtube_firstmovie.exists, \
        "ERROR:There is no movie here!"
        self._locator.btn_youtube_firstmovie.click()
        time.sleep(5)
        assert self._locator.btn_player.exists, \
        "No player coming out"
        time.sleep(10)
        self.youtube_quit_by_backkey()

    def youtube_quit_by_backkey(self):
        """
            quit app by press back key
        """
        self.d.press.back()
        self.d.press.back()
        self.d.press.home()

    def share_video_to_gmail(self):
        self.launch_app_from_home("Youtube")
        time.sleep(20)
        self._locator.btn_menu_anchor.click.wait()
        time.sleep(2)
        self._locator.btn_share.click.wait()
        time.sleep(2)
        self._locator.text_share_this_video_via.swipe.up()
        time.sleep(2)
        self._locator.btn_gmail.click.wait()
        time.sleep(2)
        if self.d(resourceId="com.google.android.gm:id/welcome_tour_got_it").exists:
            self.d(resourceId="com.google.android.gm:id/welcome_tour_got_it").click.wait()
        if self.d(resourceId="com.google.android.gm:id/action_done").exists:
            self.d(resourceId="com.google.android.gm:id/action_done").click.wait()
        if self.d(resourceId="com.google.android.gm:id/manual_sync").exists:
            self.d(resourceId="com.google.android.gm:id/manual_sync").click.wait()
        assert self._locator.account_name.exists , \
        "ERROR:There is no gmail!"
        gmail_name = self._locator.account_name.text
        self._locator.text_send_to.set_text(gmail_name)
        time.sleep(2)
        self.d(resourceId="com.google.android.gm:id/send").click.wait()
        time.sleep(20)
        self.d.press.back()

    def checkGmailHasRecived(self):
        self.launch_app_from_home("Gmail")
        time.sleep(20)
        self._locator.gmail_item.click.wait()
        time.sleep(10)
        assert self._locator.gmail_content_title, \
        "ERROR: content title not found!"
        time.sleep(10)
        self._locator.btn_gmail_delete.click()
        if self._locator.btn_conversation_list.exists:
            self._locator.btn_conversation_list.click.wait()

    def cleanCameraData(self):
        g_common_obj.adb_cmd_capture_msg(\
                    "rm /storage/sdcard0/DCIM/Camera/*")

    def launch_app_from_home(self,app_name):
        """launch app from home"""
        self.d.press.home()
        if (app_name =="Camera") :
            g_common_obj.launch_app_am(self.cfg.get("camera_package"), \
                                       self.cfg.get("camera_activity"))
        elif (app_name == "Photos") :
            g_common_obj.launch_app_am(self.cfg.get("photos_package"), \
                                       self.cfg.get("photos_activity"))
        elif (app_name == "Youtube") :
            g_common_obj.launch_app_am(self.cfg.get("package_name"), \
                                       self.cfg.get("activity_name"))
        elif (app_name == "Gmail") :
            g_common_obj.launch_app_am(self.cfg.get("gmail_name"), \
                                       self.cfg.get("gmail_activity"))
        else :
            print "[ERROR]: Launch app error, app not found!"

    def getProductName(self):
        return self.d.info.get("productName")

    def take_a_video_via_camere(self):
        self.launch_app_from_home("Camera")
        time.sleep(2)
        if self._locator.btn_next.exists:
            self._locator.btn_next.click.wait()
        self._locator.camera_mode.swipe.right()
        time.sleep(2)
        self._locator.btn_video.click.wait()
        time.sleep(4)
        self._locator.btn_shutter.click.wait()
        time.sleep(2)
        assert self._locator.text_recording_time.exists, \
        "ERROR: can't find recording icon"
        time.sleep(3)
        self._locator.btn_shutter.click.wait()
        time.sleep(5)

    def share_video_to_youtube(self):
        self._locator.camera_mode.swipe.left()
        self._locator.btn_camera_share.click.wait()
        time.sleep(5)
        self._locator.text_share_to.swipe.up()
        time.sleep(2)
        self._locator.btn_youtube.click.wait()
        time.sleep(15)

    def upload_video_to_youtube(self):
        """
        This test used to test share the video to gmail function.
        The test case spec is following:
        1. Launch the "camere" and record a video.
        2. Test share the video to youtube success.
        """
        self.share_video_to_youtube()
        if self._locator.btn_choose_gmail_account.exists:
            self._locator.btn_choose_gmail_account.click.wait()
            self._locator.btn_OK.click.wait()
            time.sleep(15)
        if self._locator.btn_menu_button.exists:
            self._locator.btn_OK.click.wait()
        self._locator.text_gmail_title.set_text("Test")
        time.sleep(2)
        if self._locator.btn_OK.exists:
            self._locator.btn_OK.click.wait()
        self._locator.text_title.click.wait()
        time.sleep(8)
        self._locator.btn_video_upload.click.wait()
        if self._locator.btn_OK.exists:
            self._locator.btn_OK.click.wait()
        print "[INFO]: start upload"
        m = 1
        while True:
            if not self._locator.progress_bar.exists or m > 90:
                break
            else :
                time.sleep(10)
            m = m + 1
        print "[INFO]: end upload"
        self._locator.btn_menu_home.click.wait()
        time.sleep(1)
        self._locator.btn_uploads.click.wait()
        time.sleep(2)
        self._locator.btn_menu_anchor.click.wait()
        self._locator.btn_delete.click.wait()
        self._locator.btn_OK.click.wait()
        time.sleep(5)
        self.youtube_quit_by_backkey()