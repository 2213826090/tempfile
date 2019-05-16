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


class ISmsMessaging(object):

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

    def register_for_sms_reception(self):
        """
        Register on SMS receive event
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_service_center_address(self, service_center_address):
        """
        Sets the service center address.

        :type service_center_address: str
        :param service_center_address: service center address

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_service_center_address(self):
        """
        Gets the service center address.

        :rtype: str
        :return: value of ServiceCenterAddress
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_incoming_sms(self, timeout=0):
        """
        Returns all incoming sms after timeout or after expected sms number is reached.

        :type timeout: int
        :param timeout: the time to wait until retrieving sms in second.

        :rtype: SmsMessage Object
        :return: sms
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def delete_all_sms(self):
        """
        Deletes all I{SMS}

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
