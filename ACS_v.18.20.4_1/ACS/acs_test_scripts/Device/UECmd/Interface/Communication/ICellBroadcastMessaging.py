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
:summary: implements the interface for cell broadcast message actions
for messaging features
:since: 20/02/2013
:author: hbian
"""
from ErrorHandling.DeviceException import DeviceException


class ICellBroadcastMessaging(object):

    """
    Abstract class that defines the interface to be implemented
    by cell broadcast message handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def wait_for_incoming_cell_broadcast_sms(self, timeout):
        """
        Waits for incoming cell_broadcast_sms, will return successfully

        :type timeout: int
        :param timeout: time in seconds to wait before cell_broadcast_sms received

        :rtype: dictionary
        :return: the dictionary of sms cb include sms cb text

        :raise DeviceException: if the command timeout is reached or the device
        doesn't receive the sms cb
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clear_all_cell_broadcast_sms(self):
        """
        Clears all cell broadcast sms

        :return: None

        :raise DeviceException: if sms cb can't be removed

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
