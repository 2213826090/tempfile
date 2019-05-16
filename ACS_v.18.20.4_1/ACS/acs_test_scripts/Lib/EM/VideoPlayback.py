"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: this is module handle a simple video playback on a given monitor on the HOST side
:author: vgomberx
:since: 04/02/2015
"""

import gtk
import sys, threading, time, os
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.AcsToolException import AcsToolException


# the code are based on example at https://wiki.videolan.org/Python_bindings
class VLCWidget(gtk.DrawingArea):
    """
    Simple VLC widget.
    """
    def __init__(self, *p):
        gtk.DrawingArea.__init__(self)
        self.vlc_instance = None
        try:
            from acs_test_scripts.Lib.EM.VLCWrapper import Instance
            self.vlc_instance = Instance(p)
        except Exception as e:
            msg = "[VLCWidget]\t fail to load VideoPlayer library (VLC need to be installed in order to use this widget) : %s" % str(e)
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsToolException(AcsToolException.INSTANTIATION_ERROR, msg)
        self.player = self.vlc_instance.media_player_new()
        def handle_embed(*args):
            if sys.platform == 'win32':
                self.player.set_hwnd(self.window.handle)
            else:
                self.player.set_xwindow(self.window.xid)
            return True
        self.connect("map", handle_embed)
        self.set_size_request(320, 200)


class VideoPlayer:
    """
    Simple video player
    """
    VIDEO_PLAYER = "[HOST_VIDEO_PLAYER]\t"
    def __init__(self, monitor_no=0, video_transform=None):
        """
        :type monitor_no: int
        :param monitor_no: the monitor where to play the video,
                -1 will always target the second monitor if it exists otherwise it will target the main

        :type video_transform: string
        :param video_transform: Set the VLC video transform filter.
                                Rotate or Flip the video.
        """
        self.__logger = LOGGER_TEST_SCRIPT

        options = ["--input-repeat=-1"]
        if video_transform in ["90", "180", "270", "hflip",
                               "vflip", "transpose", "antitranspose"]:
            options.append("--video-filter=transform")
            options.append("--transform-type=%s" % str(video_transform))

        self.__vlc = VLCWidget(*options)
        self.__window = gtk.Window()
        self.__player = self.__vlc.player
        self.__vlc_instance = self.__vlc.vlc_instance
        self.__window.add(self.__vlc)
        self.__gtk_thread = None
        # configure the window of the player
        self.__move_window_to_given_monitor(monitor_no)
        self.__window.connect("destroy", gtk.main_quit)

    def __main_gtk(self):
        """
        gtk main thread
        """
        gtk.threads_init()
        self.__window.show_all()
        gtk.threads_enter()
        LOGGER_TEST_SCRIPT.debug(self.VIDEO_PLAYER + "Entering GTK main thread")
        gtk.main()
        LOGGER_TEST_SCRIPT.debug(self.VIDEO_PLAYER + "Leaving GTK main thread")
        gtk.threads_leave()
        self.__gtk_thread = None

    def play(self, file_name):
        """
        play given file in loop

        :type file_name: str
        :param file_name: video file to play
        """
        LOGGER_TEST_SCRIPT.info(self.VIDEO_PLAYER + "Trying to play %s" % file_name)
        if not os.path.exists(file_name):
            msg = self.VIDEO_PLAYER + "The following file you want to play does not exists : %s" % file_name
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsToolException(AcsToolException.INVALID_PARAMETER, msg)

        # stop any previous running media
        self.stop()
        if self.__gtk_thread is None:
            self.__gtk_thread = threading.Thread(target=self.__main_gtk)
            self.__gtk_thread.start()
            time.sleep(0.1)
        video = self.__vlc_instance.media_new(file_name)
        self.__player.set_media(video)
        self.__player.play()
        time.sleep(0.5)

    def stop(self):
        """
        stop current video playing.
        This does not close the media player
        """
        LOGGER_TEST_SCRIPT.info(self.VIDEO_PLAYER + "Trying to stop running media")
        if self.__gtk_thread is not None:
            if (self.__player.get_media() is not None) and self.__player.is_playing():
                self.__player.stop()
                time.sleep(0.5)
            else:
                LOGGER_TEST_SCRIPT.info(self.VIDEO_PLAYER + "No media is playing")
        else:
            LOGGER_TEST_SCRIPT.debug(self.VIDEO_PLAYER + "GTK thread has already been released, nothing to stop")

    def release(self):
        """
        quit the media player, causing the closure of its windows.
        After calling this this object cant be reused, you need to instantiate a new one.
        """
        LOGGER_TEST_SCRIPT.info(self.VIDEO_PLAYER + "Release the media player view")
        try:
            gtk.main_quit()
        except Exception as e:
            msg = self.VIDEO_PLAYER + "An error happen when releasing the video player : %s" % str(e)
            LOGGER_TEST_SCRIPT.error(msg)
        del self.__gtk_thread
        self.__gtk_thread = None

    def __del__(self):
        try:
            self.__player.release()
        except:
            pass

    def is_playing(self):
        """
        return if a video is playing or not

        :rtype: boolean
        :return: True if a video is playing, False otherwise
        """
        result = False
        if self.__gtk_thread is not None:
            if (self.__player.get_media() is not None) and self.__player.is_playing():
                result = True
                msg = "A video is playing"
            else:
                msg = "No video is playing"
        else:
            msg = "GTK thread has already been released, no media is playing"

        LOGGER_TEST_SCRIPT.info(self.VIDEO_PLAYER + msg)
        return result

    def __move_window_to_given_monitor(self, monitor_no):
        """
        move the player windows to given monitor
        """
        screen = self.__window.get_screen()
        # parse over the existing monitor and search for the one you want
        monitors = []
        for m in range(screen.get_n_monitors()):
            monitors.append(screen.get_monitor_geometry(m))

        # check if the monitor number is out of range
        if monitor_no > len(monitors):
            raise Exception
        # move the windows x,y to the monitor x,y
        self.__window.move(monitors[monitor_no].x, monitors[monitor_no].y)
        self.__window.maximize()
        self.__window.fullscreen()


class MediaPlayer():
    """
    A media player that handle a video play and stop.
    its aims is to kill the video player thus releasing the media view
    when stop is called.
    """
    def __init__(self):
        self.__video_player = None

    def play(self, file_name, monitor=0, video_transform=0):
        """
        launch a video player
        """
        if self.__video_player is not None:
            self.__video_player.stop()
            self.__video_player.release()
            del self.__video_player
        time.sleep(0.1)
        self.__video_player = VideoPlayer(monitor, video_transform)
        self.__video_player.play(file_name)

    def stop(self):
        """
        kill the video player
        """
        if self.__video_player is not None:
            self.__video_player.stop()
            self.__video_player.release()
            del self.__video_player
            self.__video_player = None

    def is_playing(self):
        """
        check if a video is being play
        """
        result = False
        if self.__video_player is not None:
            result = self.__video_player.is_playing()
        return result

    def __del__(self):
        if self.__video_player is not None:
            self.__video_player.release()

# keep this commented for debug purpose
# if __name__ == '__main__':
#    med = MediaPlayer()
#    print med.is_playing()
#    med.play("C:\_Dev\BBB_1080p_12Mbps_audio_44100_30fps_HP_240s.mp4", -1)
#    print med.is_playing()
#    med.play("C:\_Dev\BBB_1080p_12Mbps_audio_44100_30fps_HP_240s.mp4", -1)
#    print med.is_playing()
#    med.stop()
#    print med.is_playing()
#    med.stop()
#    print med.is_playing()
#    med.play("C:\_Dev\BBB_1080p_12Mbps_audio_44100_30fps_HP_240s.mp4", -1)
#    print med.is_playing()
#    med.stop()
#    print med.is_playing()
#    p = VideoPlayer(-1)
#    p.play("C:\_Dev\BBB_1080p_12Mbps_audio_44100_30fps_HP_240s.mp4")
