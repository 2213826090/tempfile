"""
:copyright: (c)Copyright 2012, Intel Corporation All Rights Reserved.
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
:summary: A class that aggregates actions on a DUT in a higher-level API
:author: jreynaux
:since: 07/10/2013
"""
from acs_test_scripts.Utilities.GTestFwk.Parser import GTestFwkResult
from acs_test_scripts.Utilities.GTester.PhoneWrapper import \
    AndroidDeviceWrapper as AndroidDeviceWrapperBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class AndroidDeviceWrapper(AndroidDeviceWrapperBase):

    """
    A class used to perform operations on a C{DUT}
    when such operation are not I{UE commands}.
    """

    def __init__(self, device, command_timeout="10"):
        """
        Initializes this instance.

        :param device: the I{DUT} instance
        :type device: AndroidDeviceBase

        :param command_timeout: the timeout (in seconds) to use
            when executing shell commands on the I{DUT}.
        :type command_timeout: int
        """
        AndroidDeviceWrapperBase.__init__(self, device, command_timeout)

    @classmethod
    def get_instance(cls, device):
        """
        Returns the C{AndroidDeviceWrapper} instance to use.

        :param device: the I{DUT} instance
        :type device: AndroidDeviceBase

        :rtype: AndroidDeviceWrapper
        :return: the C{AndroidDeviceWrapper} instance
        """
        if AndroidDeviceWrapper.__INSTANCE is None:
            AndroidDeviceWrapper.__INSTANCE = AndroidDeviceWrapper(device)
        return AndroidDeviceWrapper.__INSTANCE

    def make_result_directory(self, dir_path=None):
        """
        Create the C{GTestFwk} result directory on the I{DUT} if needed.
        :raise DeviceException: if the directory could not be created.
        """
        if dir_path is None:
            dir_path = GTestFwkResult.GTEST_FWK_RESULT_ROOT
        exist = False

        # Check whether the directory already exists or not
        output = self.__run_command("adb shell ls %s" % str(dir_path))
        if "No such file or directory" not in output:
            exist = True
        if exist:
            # We return the operation status
            return Global.SUCCESS, "No error."

        # Create the directory if needed
        command = "adb shell mkdir %s" % dir_path
        self.__run_command(command)
        # Check that the directory has been created
        output = self.__run_command("adb shell ls %s" % str(dir_path))
        if "No such file or directory" not in output:
            exist = True
        if exist:
            # We return the operation status
            return Global.SUCCESS, "No error."
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "The directory %s could not be created." % dir_path)
