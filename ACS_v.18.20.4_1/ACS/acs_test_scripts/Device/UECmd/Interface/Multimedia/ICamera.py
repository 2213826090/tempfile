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

:organization: INTEL MCG PSI, TMT TTD AN
:summary: This file implements Camera UECmds
:since: 16/12/2013
:author: mmorchex, ahkhowaj
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class ICamera():
    """
    Abstract class that defines the interface to be implemented
    by multimedia camera(picture/video) handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.
        Nothing to be done in abstract class.
        """
        pass

    def launch_camera(self, camera):
        # This is a default implementation.
        """
        Launch camera
        :type camera: str
        :param camear: the back or front camera will be launched
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch(self):
        """
        Launch the camera 86
        This function is added for legacy reasons.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_scene_mode(self, scene_mode):
        # This is a default implementation.
        """
        Sets the scene mode
        :type scene_mode: str
        :param scene_mode: the scene mode to set
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def setup(self, flash_mode, screen_mode, camera_name):
        """
        set camera preferences

        @type flash_mode: string
        @param flash_mode: mode of flash (on, off or auto)

        @type screen_mode: string
        @param screen_mode: mode of screen (WideScreen or Standard)
        @type camera_name: string
        @param camera_name: name of the camera
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_color_effect(self, color_effect):
        # This is a default implementation.
        """
        Sets the color effect
        :type color_effect: str
        :param color_effect: the color effect mode to set
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_picture_size(self, width, height):
        # This is a default implementation.
        """
        Sets the color effect
        :type width: int
        :param width: picture width to set
        :type height: int
        :param height: picture height to set
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_flash_mode(self, flash_mode):
        # This is a default implementation.
        """
        Sets the flash mode
        :type flash_mode: str
        :param flash_mode: the flash mode to set
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_white_balance(self, white_balance):
        # This is a default implementation.
        """
        Sets the white balance mode
        :type white_balance: str
        :param white_balance: the white balance mode to set
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_exposure_mode_values(self):
        # This is a default implementation.
        """
        Get the actual, maximum and minimum exposure value
        :rtype: tuple
        :return: A tuple containing exposure value of the DUT (actual exposure, maximum exposure, minimum exposure)
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_exposure_mode(self, exposure_value):
        # This is a default implementation.
        """
        Sets exposure mode value
        :type exposure_value: int
        :param exposure_value: exposure mode value to set
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def take_raw_image_no_preview(self, v4l2n_path, download_dir, width, height):
        """
        Takes a picture without preview using V4L2n, and this requires v4l2n to be installed.
        v4l2n can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/nopreview_imaging.
        Project Home can be found at https://teamforge-amr-01.devtools.intel.com/sf/projects/v4l2n/
        :type v4l2n_path: string
        :param v4l2n_path: path to the v4l2n binary, with filename
        :type download_dir: string
        :param download_dir: directory to download the captured picture
        :type width: int
        :param width: picture width to set
        :type height: int
        :param height: picture height to set
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def take_picture(self, save_directory, pictures_number=1):
        # This is a default implementation.
        """
        Take picture
        :type save_directory : str
        :param save_directory: directory where to save picture taken on the DUT
        :type pictures_number: int
        :param pictures_number: the number of picture to take in case of burst mode, if picture_number > 1 , we are in
        burst mode.
        :rtype: str
        :return: MediaFile uri
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def record_video(self, duration, save_directory, mode=0, resolution=1):
        # This is a default implementation.
        """
        Record a video
        :type duration: int
        :param duration: duration of the video to record
        :type save_directory : str
        :param save_directory: directory where to save video recorded on the DUT
        :type mode: int
        :param mode: when mode is 1, picture is taken while video recording.
        :rtype: str or tuple
        :type resolution: int
        :param resolution: resolution of the video to record
        :return: Video recorded uri or video recorded and picture file uri
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def shutdown(self):
        # This is a default implementation.
        """
        Stop the camera
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def legacy_shutdown(self):
        """
        Stop the camera.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def download_media_file(self, remote_file_uri, local_file_uri):
        # This is a default implementation.
        """
        download media file drom DUT to host
        :type remote_file_uri: str
        :param remote_file_uri: uri of media file uri on the DUT.
        :rtype: str
        :return: media file uri on host
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_media_scanner(self, save_directory):
        # This is a default implementation.
        """
        force media scanner
        :type save_directory: str
        :param save_directory : directory where media files are saved in DUT
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def launch_system_camera_application(self):
        """
        Launch camera application.

        :rtype: None
        :return: None
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_face_detection(self):
        # This is a default implementation.
        """
        Starts the face detection
        :rtype: int
        :return: number of faces detected
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


    def get_camera_current_parameters(self):
        #This is a default implementation
        """
        Returns the current camera parameters configuration from the DUT.
        :rtype: dictionary
        :return: a map with current camera parameters with parameter:value
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def device_orientation(self):
        # This is a default implementation.
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def camera_app_setup(self, camera_app, mode):
        # This is a default implementation.
        """
        set camera defaults for device_save_directory, camera_package, camera_component

        @type camera_app: string
        @param camera_app: Camera app to be used.  Currently supports "Android_Intel_Camera_v2.2" and "Android_Google_Camera_v2.0"

        @type mode: string
        @param mode: mode of app to use.  Supports "video" and "camera".
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_camera_in_use(self) :
        # This is a default implementation.
        """
        @rtype: string
        @return: Camera currently in use.  "BACK" for world facing camera, "FRONT" for user facing camera and "UNKNOWN" if neither world or user facing camera.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def toggle_burst_mode(self) :
        # This is a default implementation.
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def select_camera_settings(self) :
        # This is a default implementation.
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def change_camera(self, cam_to_use) :
        # This is a default implementation.
        """
        Switches the camera in use.  Currently supports com.intel.camera22 camera package only.  If 25x16 display it will select the camera
        specified by cam_to_use: 'back' is facing the world, 'front' is facing the user.  For other displays with supporting coordinates in
        touchCmds it simply toggles the camera in use, without knowledge of which one is actually being selected.

        @type cam_to_use: string
        @param cam_to_use: name of the camera to select.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def upload_output_files(self, include_error_dir, local_file_dir, timeout = 180):
        # This is a default implementation.
        """
        upload output files from DUT backup directory to host and removes files from DUT directory.
        :type include_error_dir: boolean
        :param include_error_dir: if true, upload error_dir as well
        :rtype: str
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_system_camera_application(self):
        # This is a default implementation.
        """
            Stop the camera application
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def camera_application_take_picture(self, number_of_pics = 1, sleep_interval = 5):
        # This is a default implementation.
        """
            Use keyevent to take picture with camera application
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def camera_refcam_take_picture(self, number_of_pics = 1, sleep_interval = 5):
        # This is a default implementation.
        """
            Use input to take picture with Intel refcam
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def camera_application_start_stop_recording(self, record_time = 5):
        # This is a default implementation.
        """
            Use keyevent to make recording with camera application
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


    def get_focused_input_window_name(self):
        # This is a default implementation.
        '''
           Obtains the name of the window that currently has input focus.
        '''
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def move_files_to_backup_dir(self, filepaths, backup_directory = None):
        # This is a default implementation.
        """
        Moves the files in the filepath to the directory
        :type filepaths: str list
        :param filepaths: list of path where the files to be moved are
        :type backup_directory: string
        :param backup_directory: Optional.  If passed in, use this path to move the files to.
        :rtype: str
        :return: number of files found in backup_directory
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_for_camera_issues(self, errorCount = None):
        # This is a default implementation.
        '''
            Check if a known camera issue has occurred
            Increment the errorCount dictionary, if provided, and return one of the
            following codes:

            unclassifiedFail - Unclassified error
            camConnectionErr - "Unfortunately, camera has stopped"
            camStopped       - "Can't connect to the camera."
            camNotResponding - "Camera isn't responding. Do you want to close it?"
        '''
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def prepare_video_capture(self, error_counts = {}, restart_app = False, checkTriglogMsg = True):
        # This is a default implementation.
        """
        Launch and make sure we have correct camera package in focus.  Log errors into error_counts as encountered.

        :type error_counts: dict
        :param error_counts: dictionary of errors encountered and number of each occurrence.
        :type restart_app: bool
        :param restart_app: True if we will restart app, False if not.
        :type checkTriglogMsg: bool
        :param checkTriglogMsg: True if we want to reset triglog messages to be checked later, otherwise set to False.
        :rtype: tuple (int, reset_loop (bool))
        :return: int - Returns 0 if known issue or no error occurs, -1 if 'unclassifiedFail'.  bool - return True or False.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def verify_video_creation(self, error_counts = {}, videos_saved = 0):
        # This is a default implementation.
        """
        Checks to see if more videos have been created compared to current count of videos_saved.  If not, checks for errors
        to reset loop if known error has occurred or fail test if unclassifiedFail has occurred.

        :type error_counts: dict
        :param error_counts: dictionary of errors encountered and number of each occurrence.
        :type videos_saved: int
        :param videos_saved: Number of videos saved in backup directory before backing up anything new.
        :rtype: tuple - (video_found (bool), reset_loop (bool), videos_saved (int))
        :return: video_found -> True if more videos found in backup directory than videos_saved, False if not.
                 reset_loop -> True if known error has occurred and False otherwise.
                 videos_saved -> Number of videos saved in backup directory after moving any new files.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_camera_version(self, camera_app):
        # This is a default implementation.
        """
        @type camera_app: string
        @param camera_app: Camera app to be used
        @rtype: string
        @return: Camera version supported by AOS.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
