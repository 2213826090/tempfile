#coding=utf8=
import time
import os
from testlib.audio.audio_log import AudioLogger
from testlib.util.common import g_common_obj

class AudioImpl:
    """
        @summary: class for audio application Home UI
    """

    PACKAGE_NAME_PLAY_MUSIC = "com.google.android.music"
    ACTIVITY_NAME_PLAY_MUSIC = 'com.android.music.activitymanagement.TopLevelActivity'

    PACKAGE_NAME_SETTING = "com.android.settings"
    ACTIVITY_NAME_SETTING = ".Settings"
    PAD_TYPE = "Anchor8"
    LAUNCH_APP_DELAY = 3

    def __init__ (self, cfg = "", device = None):
        self.device = device if device else g_common_obj.get_test_device()
        self.d = self.device.get_device() if device else g_common_obj.get_device()
        self.cfg = cfg
        #self.dut = self.d
        self.logger = AudioLogger.getLogger()

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        orientation = self.d.info["displayRotation"]
        if width > height and orientation == 0:
            self.d.orientation = "r"
        elif width > height and orientation > 0 :
            self.d.orientation = "n"
        self.d.freeze_rotation()

    def launch_music_am(self, timeout = 20):
        self.logger.info("Launch GooglePlayMusic through AM")
        self.device.launch_app_am(self.PACKAGE_NAME_PLAY_MUSIC, self.ACTIVITY_NAME_PLAY_MUSIC)
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(self.LAUNCH_APP_DELAY)
            if self.d(text = "Allow").exists:# New M img should allow music app permission
                self.d(text = "Allow").click()
            if self.d.info.get("currentPackageName") == self.PACKAGE_NAME_PLAY_MUSIC:
                self.logger.debug("Launch Google Play Music succ in %ds"%(time.time() - start))
                return
        self.logger.error("Launch GooglePlayMusic by AM timeouted %ds"%timeout)
        raise Exception("Launch GooglePlayMusic timeout")

    def launch_music_from_home(self, timeout = 20):
        self.logger.info("Launch GooglePlayMusic from Home screen")
        self.d.press.home()
        self.device.launch_app_from_home_sc("Play Music")
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(self.LAUNCH_APP_DELAY)
            if self.d(text = "Allow").exists:# New M img should allow music app permission
                self.d(text = "Allow").click()
            if self.d.info.get("currentPackageName") == self.PACKAGE_NAME_PLAY_MUSIC:
                self.logger.debug("Launch Google Play Music succ in %ds"%(time.time() - start))
                return
        self.logger.error("Launch GooglePlayMusic by Home screen timeouted %ds"%timeout)
        raise Exception("Launch GooglePlayMusic timeout")

    def skip_music_app_guide(self):
        self.logger.info("Init Music app guide if exists")
        retry = 10
        while not self._is_home_page():
            if retry == 0:
                raise Exception("Can't finish guide: retry 10 times")
            time.sleep(3)
            if self.d(text = "Allow").exists:# New M img should allow music app permission
                self.d(text = "Allow").click()
            if (self.d(text = 'Not now').exists):
                self.d(text = 'Not now').click()
            if (self.d(text = 'Skip').exists):
                self.d(text = 'Skip').click()
            if (self.d(text = 'SKIP').exists):  # M use uppercase
                self.d(text = 'SKIP').click()
            if self.d(text="NO THANKS").exists:  # GoogleAccount login status
                self.d(text="NO THANKS").click()
            if (self.d(text = 'HELP').exists):
                if self.d(resourceId="com.google.android.music:id/action_bar").exists:
                    self.d(resourceId="com.google.android.music:id/action_bar").child(index=0).click()
                elif self.d(resourceId="com.google.android.music:id/play_header_toolbar").exists:
                    self.d(resourceId="com.google.android.music:id/play_header_toolbar").child(index=0).click()
            if self.d(text="LISTEN NOW",
                        className="android.widget.Button").exists:
                self.d(text="LISTEN NOW").click()
            if self.d(text="CONTINUE").exists:  # add new init stage
                self.d(text="CONTINUE").click()
            retry -= 1

    def _is_home_page(self):
        """
        @summary: Check if is GooglePlayMusic home page
        @return: boolean.
        """
        navigation_menu = self.d(resourceId = "com.google.android.music:id/play_drawer_list")
        show_navigation_menu = self.d(description = "Show navigation drawer")
        return show_navigation_menu.exists and not navigation_menu.exists

    def enterPlayPageFromHome(self):
        """
            @summary: enter play page from home
        """
        self.launch_music_from_home()
        self.skip_music_app_guide()

    def click_menu(self):
        """
        @summary: Call navigation menu
        """
        self.logger.debug("Call up navigation menu")
        navigation_menu = self.d(resourceId = "com.google.android.music:id/play_drawer_list")
        show_navigation_menu = self.d(description = "Show navigation drawer")
        if navigation_menu.exists:
            self.logger.debug("Already in navigation menu")
            return
        show_navigation_menu.click()
        time.sleep(1)
        assert navigation_menu.exists, "Call navigation menu failed."

    def enterAlbumsPage(self , folderName = 'otctest'):
        """
            @summary: enter Albums Page
        """
        self.logger.info("Enter Albums page")
        self.click_menu()
        wgt = self.d(text = 'My Library')
        if not wgt.exists:
            wgt = self.d(text = 'Music library')
        wgt.click.wait()
        assert self.d(text = 'ALBUMS').exists,'ALBUMS not exists, click My/Music Library not responded'
        self.d(text = 'ALBUMS').click.wait()
        assert self.d(text = folderName).exists, 'otctest not exists, click My ALBUMS not responded'
        self.d(text = folderName).click.wait()
        assert not self.d(resourceId = "com.google.android.music:id/pin_button").exists,\
                'pin button not exists, click folderName not responded'

    def enterSongsPage(self):
        """
            @summary: enter Albums Page
        """
        self.logger.info("Enter Songs Page")
        self.click_menu()
        wgt = self.d(text = 'My Library')
        if not wgt.exists:
            wgt = self.d(text = 'Music library')
        wgt.click.wait()
        time.sleep(3)
        songs_wgt = self.d(text = "SONGS", resourceId = "com.google.android.music:id/title")
        assert songs_wgt.exists,'SONGS not exists, click My/Music Library not responded'
        songs_wgt.click.wait()
        time.sleep(1)

    def playMusicLongLasting(self , audioName , playTime=None, repeatTime = 0):
        """
        @summary: play Music Long Lasting
        """
        self.logger.info("Play long lasting audioName: %s"%audioName)
        self.playMusic(audioName)
        assert self.d(resourceId =  "com.google.android.music:id/totaltime").exists,\
                'totaltime not exists, enter audio playback page failed'
        self.setRepeat(repeatTime)

    def playMusicLongLastingWithLoops(self, audioName, playTime = None, repeatTime = 0, loops = 1):
        """
        @summary: play Music Long Lasting with loops
        """
        self.logger.info("Play long lasting audioName: %s"%audioName)
        self.playMusic(audioName)
        self.setRepeat(repeatTime)
        #Play with loops
        for loop in range(int(loops)):
            assert self.d(resourceId =  "com.google.android.music:id/next", className = "android.widget.ImageButton").exists, 'not playback page'
            self.d(resourceId =  "com.google.android.music:id/next").click()
            self.checkMusicPlayBack(playTime)

    def checkMusicPlayBack(self, playTime = None, stopTime=None):
        """
            @summary: check Music Play Back
        """
        header_metadata_wgt = \
                self.d(resourceId = "com.google.android.music:id/header_metadata")
        if not self.d(resourceId = "com.google.android.music:id/play_controls").exists\
                and header_metadata_wgt.exists:
            header_metadata_wgt.click.wait()
            time.sleep(1)
        self.d(resourceId="android:id/progress").wait.exists(timeout=60000)
        assert self.d(resourceId="android:id/progress"),\
              'playing progress not exists, enter audio playback page failed'
        startTime = time.time()
        audioLength = None
        tt = None
        ct = None
        if playTime == None:
            while True:
                ct=self.d(resourceId="com.google.android.music:id/currenttime").text
                tt=self.d(resourceId="com.google.android.music:id/totaltime").text
                self.logger.debug("play time is " + str(ct) + " total time is " + str(tt))
                self.d.screen.on()
                if ct and tt :
                    self.logger.debug("%s,%s"%(ct, tt))
                    ctd = ct.split(":")
                    ttd = tt.split(":")
                    ict = 0
                    itt = 0
                    if len(ctd) == 2:
                        ict = int(ctd[0]) * 60 + int(ctd[1])
                    elif len(ctd) == 3:
                        ict = int(ctd[0]) * 3600 + int(ctd[1]) *60  + int(ctd[2])
                    if len(ttd) == 2:
                        itt = int(ttd[0]) * 60 + int(ttd[1])
                    elif len(ttd) == 3:
                        itt = int(ttd[0]) * 3600 + int(ttd[1]) * 60 + int(ttd[2])
                    if not audioLength and itt != 0:
                        audioLength = itt
                    self.logger.debug("Playback: total time %ss, current time %ss, audioLength %ss"
                                      %(str(itt), str(ict), str(audioLength)))
                    self.d.screen.on()
                    if ict > itt :
                        self.d.screen.on()
                        self.logger.debug("ict is %s, itt is %s"%(str(ict), str(itt)))
                        #get pause description
                        assert "play" in self.d(resourceId="com.google.android.music:id/pause").description,\
                                "Audio don't be Played"
                        return
                    else:
                        if itt - ict >= 4:
                            self.logger.debug(">4")
                            self.d.screen.on()
                            self.logger.debug("play button status is " +\
                                    str(self.d(resourceId="com.google.android.music:id/pause").description))
                            assert "Pause" in self.d(resourceId="com.google.android.music:id/pause").description,\
                                    "Audio don't be Played"
                        else:
                            self.d.screen.on()
                if stopTime and (time.time() - startTime > int(stopTime)):
                    self.logger.debug(audioLength)
                    self.logger.debug(time.time() - startTime)
                    self.logger.debug("Time is up,finish now")
                    return True
                if audioLength != None and (time.time() - startTime > audioLength ):
                        self.logger.debug(audioLength)
                        self.logger.debug(time.time() - startTime)
                        self.logger.debug("Time is up,finish now")
                        return True
        else:
            self.logger.debug("playTime is %s"%str(playTime))
            time.sleep(int(playTime))
            try:
                self.device.unlock_screen()
            except:
                self.device.adb_cmd('input keyevent 82')
            statusBefore = None
            statusAfter = None
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                statusBefore = self.d(resourceId = "com.google.android.music:id/pause").contentDescription
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                self.d(resourceId="com.google.android.music:id/pause").click()
                time.sleep(1)
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                statusAfter = self.d(resourceId = "com.google.android.music:id/pause").contentDescription
            assert not (statusBefore == statusAfter), "pad status display failed"

    def download_content(self, url, app_name):
        self.logger.debug("Download content: %s/%s"%(url,app_name))
        for i in range(10):
            cmd = "wget -c " + url + app_name
            os.system(cmd)
            if os.path.isfile(app_name):
                self.logger.debug("%s exists" % app_name)
                break
        assert os.path.isfile(app_name)

    def install_app(self,app_name):
        self.logger.info("Install app %s"%app_name)
        time.sleep(2)
        os.system('adb install -r ' + app_name + ' & > /dev/null 2>&1')
        for i in range(10):
            time.sleep(2)
            if self.d(text="Accept").exists:
                self.d(text="Accept").click.wait()
                break

    def launch_app_by_am(self,pkg_name, activity_name):
        self.device.launch_app_am(pkg_name, activity_name)
        time.sleep(2)

    def play_widevine_video(self):
        num = 0
        w=self.d.displayWidth
        h=self.d.displayHeight
        while True:
            if num >20:
                break
            if not self.d(text="Acquire Rights").exists:
                self.d.click(0.2031*w, 0.625*h)
                time.sleep(5)
            else:
                self.d(text="Acquire Rights").click()
                time.sleep(10)
                self.d.click(0.3094*w, 0.3478*h)
                break
            num += 1
        assert self.d(text="Acquire Rights").exists,"can't play video"

    def widevine_play_check(self,play_time):
        p_time = 0
        w=self.d.displayWidth
        h=self.d.displayHeight
        while True:
            if p_time >= play_time:
                self.logger.debug("break")
                break
            else:
                time.sleep(60)
            p_time += 60
            if self.d(text="Play").exists:
                self.d.click(0.3094*w, 0.3478*h)
            time.sleep(2)
            assert self.d(text="Acquire Rights").exists,"Video is not playing"

    # 2014-12-05 graceyix@intel.com
    def backPlayerScreenfromHome(self):
        """
        @summary: back to player screen from home
        """
        self.logger.debug("Back to play screen from home")
        self.launch_music_from_home()
        self.enterSongsPage()
        assert "Pause" in self.d(resourceId="com.google.android.music:id/pause").description,\
                'pause button contentDescription is not Pause,audio playback not started'
        #self.GoogleMusicPlayer().tablet_header_album_small().click()

    def playMusic(self , audioName):
        """
            @summary: play Music
        """
        time.sleep(2)
        self.logger.info("play audio named " + str(audioName))
        try: # new way
            resid = "com.google.android.music:id/drawer_container"  # playlist
            self.d(resourceId=resid).child_by_text(audioName,
                                                   allow_scroll_search=True)
        except:  # fallback to old way
            if not self.d(text=audioName).exists:
                self.d(scrollable=True).scroll.to(text=audioName)
                time.sleep(2)
            i = 100
            while not self.d(text=audioName).exists and i > 0:
                self.d(scrollable=True).scroll.vert.forward(steps=100)
                i -= 1
            assert self.d(text=audioName).exists,\
                    'audioName not exists, audio playback not started'
        time.sleep(2)
        if not self.d(resourceId="com.google.android.music:id/totaltime").exists:
            # UI not automatic enter song playing page
            self.d(text=audioName).click()
            time.sleep(2)
        header_metadata_wgt = \
                self.d(resourceId = "com.google.android.music:id/header_metadata")
        header_metadata_wgt.wait.exists()
        assert header_metadata_wgt.exists, \
                "Failed to play, music metadata widget did not appear after click to play."
        if not self.d(resourceId = "com.google.android.music:id/play_controls").exists:
            header_metadata_wgt.click.wait()
            time.sleep(1)
        assert "Pause" in self.d(resourceId="com.google.android.music:id/pause").description,\
                'pause button contentDescription is not Pause,  audio playback not started'
        assert self.d(resourceId="com.google.android.music:id/totaltime").exists,\
                'totaltime not exists, enter audio playback page failed'

    # 2014-12-05 graceyix@intel.com
    def changePlayStateWhileScreenLock(self):
        """
            @summary: play Music
        """
        if self.d(descriptionContains="Pause").exists:
            self.logger.debug("[INFO] PAUSE")
            self.d(descriptionContains="Pause").click()
        elif self.d(descriptionContains="Play").exists:
            self.logger.debug("[INFO] PLAY")
            self.d(descriptionContains="Play").click()
        else:
            self.logger.error("[WARNING]: No PLAY/PAUSE Button")
            return False
        return True

    # 2014-12-05 graceyix@intel.com
    def setRepeat(self, count):
        """
        @summary: set repeat
        """
        self.logger.debug("Set repeat %d"%int(count))
        for _ in range(int(count)):
            self.d(resourceId = "com.google.android.music:id/repeat").click()
            time.sleep(1)

    # 2014-12-05 graceyix@intel.com
    def playNextWhileScreenLock(self):
        """
            @summary: play Music
        """
        self.logger.debug("Play next while screen locked")
        if self.d(descriptionContains="Play").exists:
            self.logger.debug("[INFO] PAUSE")
            self.d(descriptionContains="Play").click()
        self.d(descriptionContains="Next").click()

    # 2014-12-05 graceyix@intel.com
    def playNextSong(self):
        """
            @summary: play Music
        """
        self.logger.debug("Play next")
        if self.d(descriptionContains="Next").exists:
            self.d(descriptionContains="Next").click()
        else:
            raise Exception("No next button here")

    # 2014-12-05 graceyix@intel.com
    def playMusicWhileScreenLock(self):
        """
            @summary: play Music
        """
        self.logger.debug("Play music while screen locked")
        if self.d(descriptionContains="Pause").exists:
            self.logger.debug("[INFO]: The music is playing, Skip this!")
            return True
        elif self.d(descriptionContains="Play").exists:
            self.d(descriptionContains="Play").click()
            return True
        else:
            self.logger.error("[WARNING]: No PLAY/PAUSE Button")
            return False

    # 2014-12-05 graceyix@intel.com
    def pauseMusicWhileScreenLock(self):
        """
            @summary: play Music
        """
        self.logger.debug("Pause music while screen locked")
        if self.d(descriptionContains="Play").exists:
            self.logger.debug("[INFO]: The music is paused, Skip this!")
            return True
        elif self.d(descriptionContains="Pause").exists:
            self.d(descriptionContains="Pause").click()
            return True
        else:
            self.logger.error("[WARNING]: No PLAY/PAUSE Button")
            return False

    # 2014-12-05 graceyix@intel.com
    def checkPlayWhileScreenLock(self):
        """
        @summary: check music play normallly while screen is locked
        """
        assert self.playMusicWhileScreenLock()

    def musicRepeat(self,times):
        """
            @summary: music Repeat
        """
        for i in range(times):
#             Constant.Log.info("click repeat " + str(i) + " times")
            self.d(resourceId = "com.google.android.music:id/repeat").click()
            time.sleep(1)

    def playMusicLoop(self , audioName , times):
        """
            @summary: music Repeat
        """
        print "playMusicLoop version 201408301524"

    def playMusicOrientation(self , repeatTime):
        """
            @summary: music Repeat
        """
        self.logger.debug("Change orientation while playing")
        for i in range(repeatTime):
            self.d.orientation = "l" # or "left"
            self.d.orientation = "r" # or "right"
            self.d.orientation = "n" # or "natural"
        if self.d(resourceId="com.google.android.music:id/shuffle").exists:
            return True
        else:
            self.logger.error("Widget com.google.android.music:id/shuffle not found")
            return False

    def playMusicSetVolume(self, repeatTime):
        """
            @summary: music Repeat
        """
        self.logger.debug("Set volume while playing")
        for _ in range(repeatTime):
            for j in range(15):
                self.d.press.volume_up()
            for j in range(15):
                self.d.press.volume_down()
        if self.d(resourceId="com.google.android.music:id/shuffle").exists:
            return True
        else:
            self.logger.error("Widget com.google.android.music:id/shuffle not found")
            return False

    # 2014-12-05 graceyix@intel.com
    def playMusicSetVolumeUpDown(self , repeatTime):
        """
            @summary: music Repeat
        """
        for i in range(repeatTime):
            self.logger.debug("switch volume for [%d]" % i)
            self.d.press.volume_up()
            time.sleep(1)
            self.d.press.volume_down()
            time.sleep(1)
        if self.d(resourceId="com.google.android.music:id/shuffle").exists:
            return True
        else:
            self.logger.error("Widget com.google.android.music:id/shuffle not found")
            return False

    def isMusicPlaying(self):
        for _ in range(20):  # wait at most 2s
            # while music is playing, Pause Button shows
            if self.d(descriptionContains="Pause").exists:
                return True
            time.sleep(0.1)
        else:
            self.logger.debug("Music not playing")
            return False

    # 2014-12-05 graceyix@intel.com
    def resetVolume(self, step):
        """
        @summary: reset volume to step
        """
        for _ in range(15):
            self.d.press.volume_down()
        for _ in range(int(step)):
            self.d.press.volume_up()

    def audioPlaybackHomeSwitch(self , repeatTime):
        """
            @summary: music Repeat
        """
        pass

    def audioPlaybackMuchTimes(self , repeatTime):
        """
            @summary: audio Playback MuchTimes
        """
        self.logger.debug("Play & pause %d times"%int(repeatTime))
        for i in range(repeatTime):
            self.d(resourceId="com.google.android.music:id/pause").click()
            time.sleep(.5)
        if self.d(resourceId="com.google.android.music:id/pause").exists:
            return True
        else:
            return False

    def cleanUpData(self):
        """
            @summary: clean Up 'Play Music' Data
        """
        self.logger.debug("Clean up music app data")
        self.device.stop_app_am(self.PACKAGE_NAME_PLAY_MUSIC)
        self.device.adb_cmd("pm clear %s" % self.PACKAGE_NAME_PLAY_MUSIC)

    def clickProgress(self, percent):
        """
            @summary: percent type is float, it is percent of audio.
        """
        from testlib.common.base import UiWindows
        uiProgress = UiWindows(self.d(resourceId="android:id/progress").info)
        x, y = uiProgress.getMidPoint()
        x = uiProgress.getLeft()+ percent*uiProgress.getWidth()
        self.d.click(x,y)

    def clickPause(self):
        self.d(resourceId="com.google.android.music:id/pause").click()

    def home(self):
        self.d.press.home()

    def back(self):
        self.d.press.back()

    def clickPrevious(self):
        self.logger.info("click Previous")
        self.d(resourceId="com.google.android.music:id/prev").click()

    def clickNext(self):
        self.logger.info("click Next")
        self.d(resourceId="com.google.android.music:id/next").click()

    def setOrientation(self, orientation="n"):
        self.d.orientation = orientation

    def clickShuffle(self):
        self.d(resourceId="com.google.android.music:id/shuffle").click()

    def isInPlayPage(self):
        return self.d(resourceId="com.google.android.music:id/shuffle").exists

    def openMainMenu(self):
        self.d(resourceId="com.google.android.music:id/play_header_toolbar").child(index=0).click()

    def setEffect(self, effect="Classical"):
        self.openMainMenu()
        self.d(text="Settings").click()
        self.d(text="Equalizer").click()
        time.sleep(2)
        if self.d(text="OFF").exists:
            self.d(text="OFF").click()
        self.d(resourceId="com.android.musicfx:id/eqSpinner").click()
        self.d(text=effect).click()
        self.d.press.back()
        time.sleep(1)
        self.d.press.back()

    def turnOffEffect(self):
        self.openMainMenu()
        self.d(text="Settings").click()
        self.d(text="Equalizer").click()
        time.sleep(2)
        if self.d(text="ON").exists:
            self.d(text="ON").click()

    def getCurrentTimestamp(self):
        ct = self.d(resourceId="com.google.android.music:id/currenttime").text
        return ts2sec(ct)

    def getTrackName(self):
        '''
        get current track name, must be in song page
        '''
        name = self.d(resourceId="com.google.android.music:id/trackname").text
        self.logger.info("Current Track Name: %s" % name)
        return name


def ts2sec(ts):
    '''
    convert timestamp such as '01:00' or '1:12:01' to seconds
    '''
    parts = ts.split(':')
    parts.reverse()
    seconds = 0
    for i in range(len(parts)):
        seconds += int(parts[i])*(60**i)
    return seconds
