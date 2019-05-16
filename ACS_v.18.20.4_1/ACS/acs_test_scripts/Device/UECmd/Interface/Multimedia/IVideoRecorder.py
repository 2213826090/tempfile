"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements Video record UECmds
:since: 13/10/2014
:author: vdechefd
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IVideoRecorder():

    """
    Abstract class that defines the interface to be implemented by video recording sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def native_video_record(self,
                            video_path,
                            camera_name,
                            camera,
                            quality,
                            flash_mode,
                            color_effect,
                            white_balance,
                            dvs,
                            noise_reduction):
        """
        Start the video record

        :type video_path: str
        :param video_path: Path to video file to play
        :type camera_name: str
        :param camera_name: which camera app to use
        :type camera: str
        :param camera: which camera to user, could be front or back
        :type quality: str
        :param quality: quality of the record, could be high, high1080p and etc
        :type flash_mode: str
        :param flash_mode: flash mode on or off
        :type color_effect: str
        :param color_effect: which color effect to apply, could be mono, auto and etc
        :type white_balance: str
        :param white_balance: white balance setting, could be auto or etc
        :type dvs: str
        :param dvs: dvs set to true or false
        :type noise_reduction: str
        :param noise_reduction: enable noise reduction, could be on or off

        :rtype: String
        :return: The created video filename
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_video_record(self, video_path, video_file_name):
        """
        Stop the video record

        :type video_path: str
        :param video_path: Path to video file to play
        :type video_file_name: str
        :param video_file_name: Record video in continuous mode or not

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_recording(self):
        """
        Check that video recording on going

        :rtype: boolean
        :return: return True if video is running False else.
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_camera(self, package_name):
        """
        Kill Camera.

        :type package_name: str
        :param mode: have to be a effective camera name.
                     if the camera name is invalid, nothing would happen

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

#-------------------uecmd above are legacy uecmd and wont be changed -----------------------------------#

    def clean_video_storage(self, camera="default"):
        """
        empty the video storage

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def setup_camera(self, camera="default", back_quality=None, skip_wizard=None):
        """
        Setup the camera
        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :type back_quality: str
        :param back_quality: the back camera quality to be choose between : 1080P, 720P, 480P, LOW (lowest supported) , MAX (max supported)

        :type skip_wizard: str
        :param skip_wizard: some camera got a wizard the first time you open ,this allow to skip it, can be ON or OFF
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def restore_camera_setup(self, camera="default"):
        """
        Restore the camera setup from a ref file when it is present on the DUT
        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :rtype: boolean
        :return: True if the file has been restored, False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def record(self, camera="default"):
        """
        Open video view, start video recording and return the raw path of the video

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"

        :rtype: tuple
        :return: ( temporary file path , final file path)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def stop_record(self, camera="default"):
        """
        stop the video recording and leave the video view

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_video_exist(self, raw_video_path):
        """
        Check if a video file exist
        this uecmd take into account the fact that ongoing recorded video my have a name different from
        its final name like having a .tmp at the end of the file name

        :type video_path: str
        :param video_path: Path of the video file generated by video record uecmd

        :rtype: boolean
        :return: True if video exist, False otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def force_stop(self, camera="default"):
        """
        force camera application to stop

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def _list_video_storage_files(self, camera="default"):
        """
        Retrieves a list of existing video

        :type camera: str
        :param camera: the camera package name, like "camera2", "camera_google"
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
