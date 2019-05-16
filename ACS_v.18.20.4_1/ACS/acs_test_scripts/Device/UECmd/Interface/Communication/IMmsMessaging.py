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
:summary: This script implements the interface for unitary actions
for messaging features
:since: 19/07/2010
:author: asebbane
"""
from ErrorHandling.DeviceException import DeviceException


# pylint: disable=W0613


class IMmsMessaging(object):

    """
    Abstract class that defines the interface to be implemented
    by messaging handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

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
        :param mms_text: Contains the text used as the main message text for the
        build MMS.

        :type content_path: str
        :param content_path: Contains the path of the image that will be attached
        to the build MMS. Can be null if the mms_type is "text".

        :type repeat_count: int
        :param repeat_count: [optional] the number of times we want to repeat the message
        content in the MMS body.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def acknowledge_mms_sending(self):
        """
        Sends an MMS previously constructed by the build_mms function
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_incoming_mms(self, timeout, sender_number):
        """
        Waits for incoming MMS, will return successfully if before the timeout
        if the DUT receives an MMS in the thread with id thread_id the number
        of MMS in the inbox reaches the value of the expcepted_number_of_MMS
        parameter

        :type timeout: int
        :param timeout: time in seconds to wait before considering that the
        MMS will never be received

        :type sender_number: str
        :param sender_number: phone number of the sender

        :rtype: int
        :return: sent_time of the received MMS.

        :raise DeviceException:
            - If the output bundle is missing one of the expected keys.
            - If the MMS is not received before timeout.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def request_remote_send_received_mms_compare(self, mms_type):
        """
        Compares the sent MMS and the last received MMS.

        :type mms_type: str
        :param mms_type: Describes the type of MMS that will be build.
            Should be either text for a MMS with subject and text or picture
            for a MMS with subject, text and a picture attached.

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def delete_all_messages(self):
        """
        Deletes all the SMS and MMS present on the phone.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def kill_mms_app(self):
        """
        Kills the MMS application.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def register_for_mms_reception(self):
        """
        Register on android.provider.Telephony.WAP_PUSH_RECEIVED intent to wait for incoming MMS
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def send_mms(self, mms_type, mms_address, mms_subject, mms_text, mms_content_path, repeat_count=0):
        """
        Send MO MMS

        :type mms_type: str
        :param mms_type: Describing the type of MMS that will be build.
        Should be either text for a MMS with subject and text or picture for a
        MMS with subject, text and a picture attached.

        :type mms_address: str
        :param mms_address: Contains the destination phone number.

        :type mms_subject: str
        :param mms_subject: Contains the text used as subject for the build MMS.

        :type mms_text: str
        :param mms_text: Contains the text used as the main message text for the
        build MMS.

        :type mms_content_path: str
        :param mms_content_path: Contains the path of the image that will be attached
        to the build MMS. Can be null if the mms_type is "text".

        :type repeat_count: int
        :param repeat_count: [optional] the number of times we want to repeat the message
        content in the MMS body.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
