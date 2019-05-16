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
:summary: This file declare SIM card UE commands
:since: 31/07/2013
:author: dgonza4x
"""
from ErrorHandling.DeviceException import DeviceException


class ISimCard(object):
    """
    :summary: SIM card UE commands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in interface.
        """
        pass

    def enable_sim_lock_system(self, required_pin):
        """
        Enable SIM lock system, asking PIN code to user for SIM card uses.

        :param required_pin: PIN code required for enabling the SIM lock system
        Must represents a number containing 4 to 8 digits
        :type required_pin: str

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def disable_sim_lock_system(self, required_pin):
        """
        Disable SIM lock system : PIN code won't be asked anymore
        at device boot.

        :param required_pin: PIN code required for disabling the SIM
        lock system.
        Must represents a number containing 4 to 8 digits
        :type required_pin: str

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def change_sim_pin(self, current_pin, new_pin):
        """
        Change the current pin code used in the SIM lock system.

        :raise DeviceException: SIM lock system is not enabled

        :param current_pin: PIN code already set. Represents a number
         containing 4 to 8 digits.
        :type current_pin: str

        :param new_pin: New PIN code to set. Represents a number
         containing 4 to 8 digits.
        :type new_pin: str

        :rtype: int
        :return: the number of tries left
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sim_lock_state(self):
        """
        Retrieve the current SIM lock system state.

        :rtype: bool
        :return: C{True} if the SIM lock system is enabled, C{False} otherwise
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def supply_pin(self, required_pin, is_password_incorrect=False):
        """
        Supply PIN code in order to unlock device at boot time.

        :param required_pin: PIN code required for unlocking phone at boot time
        Must represents a number containing 4 to 8 digits.
        :type required_pin: str

        :type is_password_incorrect: bool
        :param is_password_incorrect: Boolean indicate that we
        need to supply a incorrect PIN for test purpose.
        default value is False

        :rtype: int
        :return: the number of tries left
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def supply_puk(self, puk, new_pin):
        """
        Supply PUK code in order to unlock device already PIN locked.

        :param puk: PUK code that will be used to unlock SIM card. Represents
         a number of 8 digits.
        :type puk: str

        :param new_pin: New PIN code to set. Represents a number containing
         4 to 8 digits.
        :type new_pin: str

        :rtype: int
        :return: the number of tries left
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sim_state(self, sim_index=1):
        """
        get the SIM state

        .. note:: the following parameter applies for I{DSDS} and
            I{DSDS} only.
        :type sim_index: int
        :param sim_index: the index of the SIM (defaults to C{1})

        :rtype: int
        :return: the SIM state
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_sim_state_bfor_timeout(self, state, timeout):
        """
        Check the sim state before timeout.

        :type state: str of several state
        :param state: The possible values are:
                    - SIM_STATE_UNKNOWN
                    - SIM_STATE_ABSENT
                    - SIM_STATE_PIN_REQUIRED
                    - SIM_STATE_PUK_REQUIRED
                    - SIM_STATE_NETWORK_LOCKED
                    - SIM_STATE_READY

        :type timeout: int
        :param timeout: Time to wait attempting state

        :raise AcsConfigException: Raise INVALID_PARAMETER
        if given state is not relevant
        :raise DeviceException: Raise TIMEOUT_REACHED

        :rtype: str
        :return: the state reached if state match or raise error
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
