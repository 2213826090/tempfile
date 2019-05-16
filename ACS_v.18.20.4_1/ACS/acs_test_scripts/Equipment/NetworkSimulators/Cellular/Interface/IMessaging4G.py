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
:summary: virtual interface of LTE messaging functionalities for cellular network
simulators
:since: 29/05/2013
:author: hbian
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException


class IMessaging4G(object):

    """
    IMessaging4G class: virtual interface of LTE messaging functionalities
    for cellular network simulators.
    """

    def set_custom_sms_text(self, sms_text):
        """
        Sets custom SMS text.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_sms_over_sgs(self):
        """
        Sends a SMS over sgs.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_custom_sms_over_ims_text(self, sms_text):
        """
        Sets custom SMS text to be sent over IMS.
        :type sms_text: str
        :param sms_text: the text of the SMS to send.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_sms_over_ims_type(self, sms_type):
        """
        Sets the SMS over IMS type.
        :type sms_type: str
        :param sms_type: the type of the SMS to send.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_sms_over_ims(self, timeout=0):
        """
        If a SMS was received, retrieves all info regarding that SMS over IMS (SMS type, SMS text).
        :rtype: sms object of SmsMessage Class
        :return: sms object containing the SMS text and destination number (here it will be empty).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_sms_over_ims(self):
        """
        Send the custom SMS text over IMS.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_sms_over_ims_delivery(self, timeout=0):
        """
        Check that the SMS over IMS has been sent.
        :type timeout: integer
        :param timeout: allowed time in seconds to reach the expected state
        @:rtype: bool
        :return: boolean to indicate if expected state was reached (true) or not (false)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
