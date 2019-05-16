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
:summary: This file implements MM Video UECmds
:since: 08/04/2010
:author: fhu2
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IImage():

    """
    Abstract class that defines the interface to be implemented
    by multimedia video handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def set_wallpaper(self, wp_type, wp_name):
        # This is a default implementation.
        """
        :type wp_name: String
        :param wp_name: the name or index of wallpaper
        :type wp_type: String
        :param wp_type: the type of wallpaper: live | static
        :rtype: None
        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "Not implemented.")

    def restore_wallpaper(self):
        # This is a default implementation.
        """
        restore to original wallpaper
        :rtype: None
        :return: None

        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "Not implemented.")

    def take_screenshot_and_pull_on_host(self, filename, output_path):
        """
        Take a screenshot and save it on the host

        :type filename: str
        :param filename: the name we will give at the screenshot.
        :type output_path: str
        :param output_path: the path where you want pull the screenshot on the host
        :rtype: str
        :return: screenshot_path the complete name of the screenshot
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def touch_template_on_screen(self, screenshot, template, tap_number=1):
        """
        This function search if the template is in screenshot. If the template is match, it realize an input tap on
        template coordinate.
        :type screenshot: str
        :param screenshot: path of the screenshot
        :type template: str
        :param template: path of the template
        :type tap_number: int
        :param tap_number: number of tap on screen
        :rtype: Bool
        :return: matching, True if match template or False if doesn't.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def pull_on_host(self, file_path, output_path):
        """
        Get a file from host
        :type file_path: str
        :param file_path : path of the file to download
        :type output_path : str
        :param output_path : directory where to pull the file
        :rtype: str
        :return: path in host of the file pulled from the DUT
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
