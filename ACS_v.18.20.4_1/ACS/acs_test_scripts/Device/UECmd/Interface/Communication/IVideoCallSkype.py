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

:organization: INTEL PEG SVE DSV
:summary: This script implements unitary actions for skype video call features
:since: 07/01/2014
:author: jongyoon
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613

class IVideoCallSkype():

    """
    Abstract class that defines the interface to be implemented
    by Skype video call handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, phone):
        """
        Initializes this instances.

        Nothing to be done in abstract class.
        """
        pass

    def stop_skype_app(self):
        """
        stop Skype app

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_skype_app(self):
        """
        start Skype app

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def select_1st_contact_banner(self):
        """
        Select 1st entry on contact when banner is on screen.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def select_1st_contact_nobanner(self):
        """
        Select 1st entry on contact when no banner is on screen.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def start_call(self):
        """
        Dials a video call.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def accept_call(self):
        """
        Accepts a video call.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def reveal_buttons(self):
        """
        Reveal buttons on screen.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disconnect_call(self):
        """
        Disconnects a video call.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_logcat_trigger_message(self, keyword):
        """
        Get a logcat trigger message corresponding to given operation.

        :return :None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)