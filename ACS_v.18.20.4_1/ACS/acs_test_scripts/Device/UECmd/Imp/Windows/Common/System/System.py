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

:organization: INTEL OPM PC&WTE
:summary: This file implements the System UEcmd for Windows device
:since: 08/16/2014
:author: wchen61
"""
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.System.ISystem import ISystem


class System(BaseV2, ISystem):

    """
    :summary: System UEcommands operations for Windows platforms using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """

        BaseV2.__init__(self, device)
        ISystem.__init__(self, device)
        self._logger = device.get_logger()

        self.retrieve_system_information_component = "com.intel.acs.agent/.SystemInfo"
        self.retrieve_system_information_category = "intel.intents.category.SYSTEMINFO"

        self._tunesound_module = "acscmd.audio.TuneSoundModule"
        self._system_info_class = "acscmd.system.SystemInformationModule"

        self._dialer_package = "com.android.dialer"
        self.dialer_component = self._dialer_package + "/.DialtactsActivity"

    def check_device_file_install(self, source_file_path, destination_file_path):
        """
        Check if the file installation has succeeded on a device

        :type file_path: str
        :param file_path: file to be installed

        :type  destination: str
        :param destination: destination on device

        :rtype: tuple (bool, str)
        :return: (Output status = True if file is installed on the device else False, output error message)

        """
        self._logger.warning("[NOT IMPLEMENTED] check_device_file_install on windows platform")
        return Global.SUCCESS, "[NOT IMPLEMENTED] check_device_file_install on windows platform"
