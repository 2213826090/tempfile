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
:summary: This script implements unitary actions for sip call features
:since: 06/02/2013
:author: nprecigx
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class ISipCall():

    """
    Abstract class that defines the interface to be implemented
    by voice call handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, phone):
        """
        Initializes this instances.

        Nothing to be done in abstract class.
        """
        pass

    def initialize_sipcall_module(self):
        """
        initialize sip call module

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def clean_sipcall_module(self):
        """
        clean sip module, remove all sip active profile

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def initialize_local_profile(self, phone_sip_address):
        """
        Initialize local profile

        :type phone_sip_address: str
        :param phone_sip_address: sip account name with server addesss like 'sip_account_name@sip_server'

        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def dial(self, address_to_call, check_state=True):
        """
        Dials a SIP call.

        :type address_to_call: str
        :param address_to_call: SIP address to call like 'sip_account_name@sip_server'

        :type check_state: bool
        :param check_state: check call state or not.

        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases SIP call

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def answer(self):
        """
        Answer SIP call.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sip_call_state(self):
        """
        Returns SIP call state.

        :rtype: object
        :return: A value of SIP_CALL_STATE.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_state(self, state, timeout):
        """
        Waits to reach a Sip call state until a timeout.

        :type state: UECmd.SIP_CALL_STATE
        :param state: expected state (see UECmd.SIP_CALL_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :return: None
        :raise: DeviceException
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def switch_to_bluetooth(self):
        """
        Switch audio to bluetooth.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def switch_to_bluetooth_a2dp(self):
        """
        Switch audio to bluetooth A2DP.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def switch_to_speaker(self):
        """
        Switch audio to speaker.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def switch_to_earpiece(self):
        """
        Switch audio to earpiece.

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
