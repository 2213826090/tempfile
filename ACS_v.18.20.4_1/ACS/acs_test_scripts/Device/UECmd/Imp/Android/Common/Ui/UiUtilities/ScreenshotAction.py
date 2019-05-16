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
:since: 2011/09/08
:author: asebbane
"""

import os
import datetime

from UtilitiesFWK.Utilities import Global
from UiAction import UiAction


class ScreenshotAction(UiAction):

    """
    This class implements the I{screenshot} action.
    """

    def __init__(self, monkey, parameters, default_timeout):
        """
        Constructor.
        Expected parameters:
            - C{parameters[0]} : device instance
            - C{parameters[1]} : file name
            - C{parameters[2]} : file extension
                (optional if no more parameter is given,
                mandatory otherwise : may be <None>)
            - C{parameters[3]} : the timeout (optional)

        :type default_timeout: int
        :param default_timeout: the default time out value to use when
            none is specified
        :type parameters: list
        :param parameters: the list of parameters as described above.
        """
        UiAction.__init__(self, monkey, default_timeout)
        self.set_parameters(parameters["parameters"])
        self._parameters_processed = False
        self.__device_instance = parameters["device"]
        self.__device_name = self.__device_instance.get_name()
        self.__file_name_base = None
        self.__file_name_extension = None
        self.__file_name_prefix = "UIBase"
        self.__screenshot_tmp_dir = None
        self.__file_extension = None

    def _do_process_parameters(self):
        """
        Update this action attributes from the parameters
        given at instantiation time.
        """
        # Update the file name base
        self.__file_name_base = self._parameters[0]
        # Update the file extension
        extension = None
        if len(self._parameters) > 1:
            extension = self._parameters[1]
        self.__file_extension = extension
        # Update the timeout
        if len(self._parameters) > 2:
            # Check whether there is a timeout parameter
            (timeout, fail_on_timeout) = self.split_timeout(self._parameters[2])
            self.set_timeout(timeout)
            self.set_fail_on_timeout(fail_on_timeout)
        else:
            # If not, use default value
            self.set_timeout(self._default_timeout)
        self._parameters_processed = True

    def __construct_image_path(self, image_name, image_file_extension=None):
        """
        Returns a convenient file path built from the given parameters.
        :rtype: str
        :return: a file path.
        """
        current_time = datetime.datetime.now().strftime("%Y-%m-%d_%Hh%M.%S")

        if image_file_extension:
            # Build a file name with extension
            image_file_name = "%s_%s_%s_%s.%s" % (self.__device_name,
                                                  self.__file_name_prefix,
                                                  image_name,
                                                  current_time,
                                                  image_file_extension)
        else:
            # Build a file name without extension
            image_file_name = "%s_%s_%s_%s" % (self.__device_name,
                                               self.__file_name_prefix,
                                               image_name,
                                               current_time)
        # Add the directory to the file path
        screenshot_tmpdir = \
            self.__device_instance.get_report_tree().get_subfolder_path("SCREEN")
        image_path = os.path.join(
            screenshot_tmpdir,
            image_file_name)
        # Return the computed value
        return image_path

    def _do(self):
        """
        Runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        # Build the image path and store it as a result
        result = self.__construct_image_path(
            self.__file_name_base,
            self.__file_name_extension)
        self._set_result(result)
        # Actually do the screenshot
        self.__device_instance.screenshot("", self.get_result())

        return Global.SUCCESS

    def _redo(self):
        """
        Re-runs this action.
        """
        if not self.parameters_processed():
            self.process_parameters()
        return self._do()

    def can_redo(self):
        """
        Returns a C{bool} indicating whether this action can
        be re-done or not.
        :rtype: bool
        :return: C{True}
        """
        return True

    def can_undo(self):
        """
        Returns a C{bool} indicating whether this action can
        be undone or not.
        This feature may be implemented later.
        :return: C{False}
        """
        return False

