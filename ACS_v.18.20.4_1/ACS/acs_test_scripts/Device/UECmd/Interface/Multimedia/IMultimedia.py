"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file define interface for multimedia ue commands (audio, video ...)
:since: 18/01/2013
:author: ssavrimoutou
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IMultimedia():

    """
    Abstract class that defines the interface to be implemented
    by multimedia audio handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def get_media_file_duration(self, filename):
        """
        Get the duration the audio/video file.

        :type filename: str
        :param filename: Audio/Video file used to get the duration

        :rtype: float
        :return: duration in second
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_system_gallery_application(self):
        """
        Launch the gallery application

        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_video_id_from_path(self, filePath, mediaStoreName):
        """
        Get the video id from is path.

        :type filePath: str
        :param filePath: path of the video.

        :rtype: str
        :return: video id
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def play_video_from_ID(self, ID, screen_orientation=None):
        """
        Launch the video file playback.

        :type filename: str
        :param filename: can be only file stored in the Device file system

        :type screen_orientation: str
        :param screen_orientation: Possible values are portrait or landscape
            If none it will use default device screen orientation

        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_gallery_system_package(self):
        """
        Retrieve gallery system package install on the DUT.

        :rtype: tuple
        :return: a tuple containing video component found on DUT as str and gallery component found on DUT as str.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def display_image_gallery3d(self, image_path, gallery3d_icon, pictures_save_directory):
        """
        Display image using Gallery3d
        :type image_path : str
        :param image_path : Path to the image to display
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def take_screenrecord(self, screenrecord_name, save_directory="/sdcard/", verbose=False, rotate=False,
                              bit_rate=4000000, time_limit=180):
        """
        Take scrrenrecord on the DUT ans store it on sdcard

        :type screenrecord_name: str
        :param screenrecord_name: Name of screenrecord from sdcard.

        :type verbose: bool
        :param verbose: If True, display interesting information on stdout. By default False.

        :type rotate: bool
        :param rotate: If True, rotate the output 90 degrees.By default False.

        :type bit_rate: int
        :param bit_rate: Set the video bit rate, in megabits per second.  Default 4Mbps.

        :type time_limit: int
        :param time_limit: Set the maximum recording time, in seconds.  Default / maximum is 180.

        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_media_scanner(self, save_directory):
        """
        Force media scanner
        :type save_directory: str
        :param save_directory : directory where media files are saved in DUT
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def execute_icons_sequence(self, icons, pictures_save_directory, time_between_action=0.5, rotate=False):
        """
        Executes a sequence of tap on screen actions following a list of icons
        :type icons : list
        :param icons : icons to click on
        :type pictures_save_directory : str
        :param pictures_save_directory : directory where to save screen shots taken
        :type time_between_action : int
        :param time_between_action : time between tap on screen actions
        :type rotate : bool
        :param rotate : whether or not screen shots taken should be rotated.
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_run_run_3d(self):
        """
        Starts Run Run 3D
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_run_run_3d(self):
        """
        Stops Run Run 3D
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_hardware_composer(self, enable = True):
        """
        This does the same thing as going to Settings>Devloper options and enable/disable HW composer. The
        information for the command came from packages/apps/Settings/src/com/android/settings/DevelopmentSettings.java:writeDisableOverlaysOption()
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_intel_smart_video(self, enable = True):
        """
        Turn on VSP features by writing to the Intel Smart Video shared preferences file.
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_number_of_dropped_frames(self):
        """
        Get number of dropped video frames from dumpsys
        :rtype output : string
        :return output : dumpsys output
        :rtype no_total_frames : string
        :return no_total_frames : Total number of played frames
        :rtype  no_dropped_frames : string
        :return no_dropped_frames : number of dropped frames
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_youtube_cmdline_launch(self, version):
        """
        Add a permission to play Youtube from command-line
        :rtype : None
        :return : None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def open_google_photos(self):
        """
        Open google
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_screen(self, icon, type):
        """
        Check screen
        :type icon: str
        :param icon : id of icon
        :type type: str
        :param type: Type of id
        :rtype: bool
        :return: True if icon_id is in UI or False if icon_id is not in UI
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def open_google_camera(self):
        """
        Open google camera
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)