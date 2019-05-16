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
:summary: This file implements the MMS UEcmd for LLP Android device
:since: 29/10/14
:author: asebbanx
"""
import time

from acs_test_scripts.Device.UECmd.Imp.Android.KK.Communication.\
MmsMessaging import MmsMessaging as MmsMessagingCommon
from acs_test_scripts.Device.UECmd.UECmdTypes import MSG_DB_PATH


class MmsMessaging(MmsMessagingCommon):
    """
    :summary: MMS UE Commands operations for LLP Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Constructor.
        """
        MmsMessagingCommon.__init__(self, device)
        self._mms_type = None
        self._address = None
        self._subject = None
        self._mms_text = None
        self._content_path = None
        self._repeat_count = 0

    def build_mms(self, mms_type, address, subject, mms_text, content_path, repeat_count=0):
        """
        Construct a MMS. Using the parameters passed by the TestCase.
        On L this methods simply stores the provided parameters for a
        later use.
        The operation to send the MMS is actually done in C{send_mms} method.

        :type mms_type: str
        :param mms_type: Describing the type of MMS that will be build.
        Should be either text for a MMS with subject and text or picture for a
        MMS with subject, text and a picture attached.

        :type address: str
        :param address: Contains the destination phone number.

        :type subject: str
        :param subject: Contains the text used as subject for the build MMS.

        :type mms_text: str
        :param mms_text: Contains the text used as the main message text for the
        build MMS.

        :type content_path: str
        :param content_path: Contains the path of the image that will be attached
        to the build MMS. Can be null if the mms_type is "text".

        :type repeat_count: int
        :param repeat_count: [optional] the number of times we want to repeat the message
        content in the MMS body.
        """
        self._logger.info("Storing MMS information for later use.")

        self._mms_type = mms_type
        self._address = address
        self._subject = subject
        self._mms_text = mms_text
        self._content_path = content_path
        self._repeat_count = repeat_count

    def __reset_mms_info(self):
        """
        Restores this object's MMS information to their default values.
        """
        self._mms_type = None
        self._address = None
        self._subject = None
        self._mms_text = None
        self._content_path = None
        self._repeat_count = 0

    def acknowledge_mms_sending(self):
        """
        Sends an MMS with the information coming from a previous call
        to C{build_mms} method.

        This method overrides previous definitions by adding the optional parameter.
        """
        self._logger.info("Sending the MMS.")

        function = "acknowledgeMmsSending"

        self._internal_exec_v2(self._mms_module, function,
                               "--es type %s --es address %s --es mms_subject \"%s\" "
                               "--es mms_text \"%s\" --es file_path \"%s\" --ei repeat_count %s"
                               % (
                                    self._mms_type,
                                    self._address,
                                    self._subject,
                                    self._mms_text,
                                    self._content_path,
                                    str(self._repeat_count)), is_system=True)
        # Reset MMS information
        self.__reset_mms_info()
