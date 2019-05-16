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

from acs_test_scripts.Device.UECmd.Interface.Communication.ISmsMessaging import ISmsMessaging
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage
import threading
from   UtilitiesFWK.Utilities import get_method_name
import Queue
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class SmsMessaging(Base, ISmsMessaging):

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
        ISmsMessaging.__init__(self, device)
        self._logger = device.get_logger()

        self._module_name = "Intel.Acs.TestFmk.MBConnectivity"
        self._class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"
        # Retrieve default timeout used for UEcmd in sec and convert it to ms
        self._msg_timeout = self._uecmd_default_timeout * 1000

    def register_for_sms_reception(self):
        """
        Register on incoming SMS event
        """
        self._logger.info("Not implemented yet on Windows")

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
        function = "SmsSend"
        args = "destination=%s message=%s time_out=%s" % ( destination, message, self._msg_timeout)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)

    def set_service_center_address(self, service_center_address):
        """
        Sets the service center address.

        :type service_center_address: str
        :param service_center_address: service center address

        :return: None
        """
        function = "SmsSetServiceCenterAddress"
        args= "service_center_address=%s time_out=%s" % ( service_center_address, 20000 )
        # Get the method and class name of the UEcommand on the embedded side
        module_name = "Intel.Acs.TestFmk.MBConnectivity"
        class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)

    def get_service_center_address(self):
        """
        Gets the service center address.

        :rtype: str
        :return: value of ServiceCenterAddress
        """
        function = "SmsGetServiceCenterAddress"
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)
        return output["values"]["service_center_address"]

    def wait_for_incoming_sms(self, timeout=0):
        """
        Register on SMS_Received intent to wait for incoming SMS and retrieve
        the received sms before timeout.

        :type timeout: int
        :param timeout: the time to wait until retrieving sms in second.

        :rtype: list of SmsMessage Object
        :return: sms array
        """
        self._logger.info("wait for incoming sms")
        function = "SmsWaitForIncoming"
        args = " time_out=%s" % (timeout*1000)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        try:
            # Launch the UEcmd on the embedded side
            output = self._internal_uecmd_exec(module_name, class_name, function, args, timeout/1000)

        except AcsBaseException as ex:
            error_msg = ex.get_error_message()
            if error_msg.find("OK") != -1 or\
                    error_msg.find("OK") != -1:
                self._logger.debug("%s" % error_msg)
            else:
                self._logger.error("%s" % error_msg)
                raise ex

        sms_list = output["values"]["list_sms"]
        count = output["values"]["count"]

        if count >= 1:
            # Fill SmsMessage object to perform comparison
            sender = "" # TBD: retrieve from UE cmd the originated number from where the SMS was received
            text = str(sms_list)
            sms = SmsMessage(text, sender)
        else:
            exception_message = "SMS not retrieved"
            self._logger.error("%s" % exception_message)
            raise DeviceException(DeviceException.SMS_EXCEPTION, exception_message)

        return sms

    def delete_all_sms(self):
        """
        Clears all I{SMS}

        :return: None
        """
        # Get the method and class name of the UEcommand on the embedded side
        module_name = "Intel.Acs.TestFmk.MBConnectivity"
        class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"

        self._logger.info("Clear all sms")

        function = "SmsDeleteAll"
        args = "time_out=30000"

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args, 30)

        #TODO: need to check if necessary to delete sms on sim card as well.
        #Clear also the sms on sim card
        #self.__delete_all_sms_on_sim()

    def __delete_all_sms_on_sim(self):
        """
        Deletes all sms stored on sim card

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                              "%s not implemented on Windows" % get_method_name())
