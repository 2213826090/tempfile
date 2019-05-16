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
:summary: This file implements the Messaging UEcmd for Android device
:since: 01/04/2011
:author: asebbane
"""
from acs_test_scripts.Device.UECmd.Interface.Communication.ISmsMessaging import ISmsMessaging
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage
import time
import os
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException


class SmsMessaging(BaseV2, ISmsMessaging):
    """
    :summary: Messaging UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    @need('modem')
    def __init__(self, device):
        """
        Constructor.

        """
        BaseV2.__init__(self, device)
        ISmsMessaging.__init__(self, device)
        self._logger = device.get_logger()
        self._sms_module = "acscmd.telephony.messaging.SmsModule"

    def register_for_sms_reception(self):
        """
        Register on android.provider.Telephony.SMS_RECEIVED intent to wait for incoming MMS
        """
        self._logger.info("register on android.provider.Telephony.SMS_RECEIVED \
                           intent for SMS Reception")

        self._internal_exec_v2(self._sms_module, "registerForSmsReception", is_system=True)

    def send_sms(self, destination, message, scaddress=None, check_delivery=False):
        """
        Sends a I{SMS} to the given C{destination} with the given content C{message}.

        :type destination: str
        :param destination: the destination device number

        :type message: str
        :param message: the text message to send

        :type scaddress: str
        :param scaddress: the Service Center Address to use for this message

        :type check_delivery: bool
        :param check_delivery: Determine if the global verdict of the uecmd should
                            determined by the reception instead of sending.
        """
        self._logger.info("Sending sms ...")
        if destination in (None, ""):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Destination of SMS is empty")

        function = "sendSms"

        params = ""
        timeout = None
        # Maximum length of an adb command is 1055 char, if sms to send is greater,
        # write message in a file and push this file on DUT in order to send sms from this file
        if len(message) > 1000:
            # write SMS text in a file
            local_file = os.path.join(os.getcwd(), "_Embedded", "USERDATA", "sms.txt")
            # Create acs/src/_Embedded/USERDATA folder if not exist
            local_dir = os.path.dirname(local_file)
            if not os.path.exists(local_dir):
                os.makedirs(local_dir)
            # write message into sms.txt file
            with open(local_file, 'w') as sms_file:
                sms_file.write(message)
                sms_file.close()
            # Push this file on DUT
            remote_file = os.path.join(self._device.multimedia_path, "sms.txt").replace('\\', '/')
            self._device.push(local_file, remote_file)
            message = ""
            params = "--es msgfile %s " % remote_file
            timeout = 1000

        params += "--es number %s --es message \"%s\"" % (destination, message)

        if check_delivery:
            self._logger.debug("Verdict will be on sms delivery !")
            params += " --ez checkdelivery true"

        if scaddress is not None:
            params += " --es scaddress %s" % scaddress

        self._internal_exec_v2(self._sms_module, function, params, timeout=timeout, is_system=True)

    def set_service_center_address(self, service_center_address):
        """
        Sets the service center address.

        .. warning:: No to be implemented in Android platform
        :type service_center_address: str
        :param service_center_address: service center address

        :return: None
        """
        function = "setSmsServiceCenter"
        params = "--es scaddress {0}".format(service_center_address)
        self._internal_exec_v2(self._sms_module, function, params, is_system=True)

    def get_service_center_address(self):
        """
        Gets the service center address.

        .. warning:: Cannot be implemented on Android using Binder (adb services)
        .. todo:: Implementation with I{Smoke} application to check.
                Using android.telphony.SmsMessage.getServiceCenterAddress()

        :rtype: str
        :return: value of ServiceCenterAddress
        """
        function = "getSmsServiceCenter"
        output = self._internal_exec_v2(self._sms_module, function, is_system=True)
        return output["scaddress"]

    def wait_for_incoming_sms(self, timeout=0):
        """
        Register on SMS_Received intent to wait for incoming SMS and retrieve
        the received sms before timeout.

        :type timeout: int
        :param timeout: the time to wait until retrieving sms in second.

        :raise: Raise an error if the intent timed out

        :rtype: SmsMessage Object
        :return: sms
        """
        self._logger.info("Wait for incoming SMS")
        start_time = time.time()
        # While timeout not reach, retrieve SMS data
        while (time.time() - start_time) < int(timeout):
            try:
                # Retrieve SMS data
                output = self._internal_exec_multiple_v2(self._sms_module, "assembleReceivedSms", is_system=True)
                self._logger.info("wait_for_incoming_sms output : %s" % output)
                # If not empty, then break while loop
                if len(output) >= 1:
                    break
            except AcsBaseException as ex:
                error_msg = ex.get_error_message()
                if error_msg.find(self.UECMD_TIMEOUT_INTENT_MSG) != -1 or\
                        error_msg.find(self.UECMD_TIMEOUT_RESULT_MSG) != -1:
                    self._logger.debug("%s" % error_msg)
                else:
                    self._logger.error("%s" % error_msg)
                    raise ex
        text = ""
        sender = ""
        # Rearrange last data received to match SMS object
        for sms_data in output:
            sender = sms_data["address"]
            text += sms_data["text"]
        sms = SmsMessage(text, sender)
        # Return last SMS received
        return sms

    def delete_all_sms(self):
        """
        Deletes all I{SMS}

        :return: None
        """
        self._logger.info("Delete all sms")

        self._internal_exec_v2(self._sms_module, "deleteAllSms", is_system=True)

        # Clear also the sms on sim card
        self.__delete_all_sms_on_sim()

    def __delete_all_sms_on_sim(self):
        """
        Deletes all sms stored on sim card

        :return: None
        """
        self._logger.info("Clear all sms on sim card")

        function = "deleteAllSmsOnSim"

        output = self._internal_exec_v2(self._sms_module, function, is_system=True)
        self._logger.info(output)
