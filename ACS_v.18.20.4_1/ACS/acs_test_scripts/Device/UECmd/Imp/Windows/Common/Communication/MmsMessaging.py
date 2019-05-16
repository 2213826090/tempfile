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
:summary: This file implements the Messaging UEcmd for Windows device
:since: 01/14/2014
:author: Hong Fang
"""

from acs_test_scripts.Device.UECmd.Interface.Communication.IMmsMessaging import IMmsMessaging
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage
import threading
from   UtilitiesFWK.Utilities import get_method_name
import Queue
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class MmsMessaging(Base, IMmsMessaging):

    """
    @summary: Messaging UEcommands operations for Windows platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    @need('modem')
    def __init__(self, device):
        """
        Constructor.

        """
        Base.__init__(self, device)
        IMmsMessaging.__init__(self, device)
        self._logger = device.get_logger()

        self._module_name = "Intel.Acs.TestFmk.MBConnectivity"
        self._class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"
        # Retrieve default timeout used for UEcmd in sec and convert it to ms
        self._msg_timeout = self._uecmd_default_timeout * 1000

    def build_mms(self, mms_type, address, subject, mms_text, content_path, repeat_count=0):
        """
        Construct a MMS. Using the parameters passed by the TestCase.

        :type mms_type: str
        :param mms_type: Describing the type of MMS that will be build.
        Should be either text for a MMS with subject and text or picture for a
        MMS with subject, text and a picture attached.

        :type address: str
        :param address: Contains the destination phone number.

        :type subject: str
        :param subject: Contains the text used as subject for the build MMS.

        :type mms_text: str
        @:param mms_text: Contains the text used as the main message text for the
        build MMS.

        :type content_path: str
        :param content_path: Contains the path of the image that will be attached
        to the build MMS. Can be null if the mms_type is "text".

        :type repeat_count: int
        :param repeat_count: [optional] the number of times we want to repeat the message
        content in the MMS body.
        """
        # TODO : critical UEcmd
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                              "%s not implemented on Windows" % get_method_name())

    def delete_all_messages(self):
        """
        Deletes all the SMS and MMS present on the phone.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                          "%s not implemented on Windows" % get_method_name())

    def kill_mms_app(self):
        """
        Kills the MMS application.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                              "%s not implemented on Windows" % get_method_name())
