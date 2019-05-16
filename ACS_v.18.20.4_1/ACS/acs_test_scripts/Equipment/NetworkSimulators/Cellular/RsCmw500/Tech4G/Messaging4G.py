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
:summary: LTE messaging implementation for R&S CMW500
:since: 29/05/2013
:author: hbian
"""
import time
import acs_test_scripts.Utilities.SmsUtilities as SmsUtil
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IMessaging4G import \
    IMessaging4G


class Messaging4G(IMessaging4G, VisaObject):

    """
    LTE messaging implementation for R&S CMW500
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)

    def set_custom_sms_text(self, sms_text):
        """
        Sets custom SMS text.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        :raise TestEquipmentException: if sms_text is not str or length is greater than 160
        """
        if not isinstance(sms_text, str) and len(sms_text) > 160:
            msg = "sms_text should be a max length 160 string"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

        self.get_logger().info("Set custom sms text: %s" % sms_text)
        self._visa.send_command('CONF:LTE:SIGN:SMS:OUTG:INT "%s"' % sms_text)

    def send_sms_over_sgs(self):
        """
        Sends a SMS over sgs.
        """
        self.get_logger().info("Send sms")
        self._visa.send_command("CALL:LTE:SIGN:PSW:ACT SMS")

    def set_custom_sms_over_ims_text(self, sms_text):
        """
        Sets custom SMS text to be sent over IMS.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        """
        self.get_logger().info("Set custom SMS over IMS text to: %s" % sms_text)
        self._visa.send_command('CONFigure:DATA:CONTrol:IMS:SMS:TEXT "%s"' % sms_text)

    def set_sms_over_ims_type(self, sms_type):
        """
        Sets the SMS over IMS type.
        :type sms_type: str
        :param sms_type: the type of the SMS to send.
        """
        self.get_logger().info("Set SMS over IMS to type: %s" % sms_type)
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS:SMS:TYPE %s" % sms_type)

    def retrieve_sms_over_ims(self, timeout=0):
        """
        If a SMS was received, retrieves all info regarding that SMS over IMS (SMS type, SMS text).
        :rtype: sms object of SmsMessage Class
        :return: sms object containing the SMS text and destination number (here it will be empty).
        """
        self.get_logger().info("Check that SMS over IMS is received before timeout: %s" % timeout)
        # 1st shot
        sms_type, sms_text = (self._visa.query_command("SENSe:DATA:CONTrol:IMS:SMS:RECeived?")).split(",")
        # SMS received
        if len(sms_text) > 2:
            self.get_logger().info("SMS over IMS received by Network Simulator")
        else:
            timer = timeout
            # Wait for SMS to be received
            while (timer > 0) and (sms_text == ""):
                sms_type, sms_text = (self._visa.query_command("SENSe:DATA:CONTrol:IMS:SMS:RECeived?")).split(",")
                time.sleep(1)
                timer -= 1
        # Display the sms_type and sms_text read at last
        self.get_logger().info("SMS type is : %s, and SMS text received is : %s ", sms_type, sms_text)
        return SmsUtil.SmsMessage(sms_text, "")

    def send_sms_over_ims(self):
        """
        Send the custom SMS over IMS.
        """
        self.get_logger().info("Send SMS over IMS from Network Simulator")
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS:SMS:SEND")

    def check_sms_over_ims_delivery(self, timeout=0):
        """
        Check that the SMS over IMS has been sent.
        :type timeout: integer
        :param timeout: allowed time in seconds to reach the expected state
        @:rtype: bool
        :return: boolean to indicate if expected state was reached (true) or not (false)
        """
        self.get_logger().info(
            "Check SMS over IMS was sent by Network Simulator before timeout: %s",
            timeout)
        # 1st shot
        current_state = self._visa.query_command("SENSe:DATA:CONTrol:IMS:SMS:SEND:STATus?")
        # Sending SMS completed
        if current_state == "SCOM":
            self.get_logger().info("Send SMS over IMS completed by Network Simulator")
            return True
        # Sending SMS has not yet started or is in progress
        elif current_state == "NONE" or current_state == "SIPR":
            timer = timeout
            # Wait for SMS sending to complete
            while (timer > 0) and (current_state != "SCOM"):
                current_state = self._visa.query_command("SENSe:DATA:CONTrol:IMS:SMS:SEND:STATus?")
                time.sleep(1)
                timer -= 1
            # Sending SMS completed
            if current_state == "SCOM":
                self.get_logger().info("Send SMS over IMS completed by Network Simulator")
                return True
            else:
                # Timeout to reach completion (Test failed no TestEquipmentException raised)
                self.get_logger().info("Timeout on check SMS over IMS state: %s does not match SCOM", current_state)
                return False
        # Sending SMS failed "SFA"
        else:
            self.get_logger().info("Send SMS over IMS failed")
            return False
