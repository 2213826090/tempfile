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
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Telephony.ISimCard import ISimCard
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.Utilities.SimCardUtilities import is_pin_valid, is_puk_valid
from UtilitiesFWK.Utilities import str_to_bool
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import time

class SimCard(BaseV2, ISimCard):
    """
    :summary: SIM card UE commands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    # Output text of the Android exception message when an incorrect
    # PIN is supplied to device.
    INCORRECT_PASSWORD_MSG = "PASSWORD_INCORRECT"

    # Constant Android values for SIM_STATE
    POSSIBLE_SIM_STATES = {
        "SIM_STATE_UNKNOWN": 0,
        "SIM_STATE_ABSENT": 1,
        "SIM_STATE_PIN_REQUIRED": 2,
        "SIM_STATE_PUK_REQUIRED": 3,
        "SIM_STATE_NETWORK_LOCKED": 4,
        "SIM_STATE_READY": 5}

    @need('modem')
    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in interface.
        """
        BaseV2.__init__(self, device)
        ISimCard.__init__(self, device)
        self._logger = device.get_logger()
        self.__module_name = "acscmd.telephony.SimCardModule"

    def enable_sim_lock_system(self, required_pin):
        """
        Enable SIM lock system, asking PIN code to user for SIM card uses,
        as in "Settings -> Security -> Set up SIM card lock" menu.

        :param required_pin: PIN code required for enabling the SIM lock system
        Must represents a number containing 4 to 8 digits
        :type required_pin: str

        :rtype: None
        """
        self._logger.info("Enabling SIM lock system with PIN code : " +
                          str(required_pin))
        self.__set_sim_lock_state(True, required_pin)

    def disable_sim_lock_system(self, required_pin):
        """
        Disable SIM lock system :
        PIN code won't be asked anymore at device boot.

        :param required_pin: PIN code required for disabling
        the SIM lock system. Must represents a number
        containing 4 to 8 digits
        :type required_pin: str

        :rtype: None
        """
        self._logger.info("Disabling SIM lock system using PIN code %s"
                          % required_pin)
        self.__set_sim_lock_state(False, required_pin)

    def __set_sim_lock_state(self, enable, required_pin):
        """
        Enable or disable SIM lock system, as in
        "Settings -> Security -> Set up SIM card lock" menu.

        :param enable: Enable SIM lock system when set to <i>True</i>,
        disable it when set to <i>False</i>
        :type enable: bool

        :param required_pin: PIN code required for changing the
        SIM lock system state. Must represents a number containing 4 to 8
        digits
        :type required_pin: str

        :rtype: None
        """
        method = "setSimLockState"
        method_args = "--ez enable %s --es pin %s"\
                % (str(enable), required_pin)
        self._internal_exec_v2(self.__module_name, method, method_args, is_system=True)

    def change_sim_pin(self, current_pin, new_pin):
        """
        Change the current pin code used in the SIM lock system,
        as in "Settings -> Security -> Set up SIM card lock" menu.

        :raise AcsConfigException: SIM lock system is not enabled

        :param current_pin: PIN code already set. Represents a number
        containing 4 to 8 digits.
        :type current_pin: str

        :param new_pin: New PIN code to set. Represents a number
        containing 4 to 8 digits.
        :type new_pin: str

        :rtype: int
        :return: the number of tries left
        """
        self._logger.info("Changing PIN code from %s to %s" % (current_pin, new_pin))
        if not is_pin_valid(current_pin) or not is_pin_valid(new_pin):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Invalid PIN code (must represents a number containing 4 to 8 digits)")
        # check prerequisite : sim lock feature must be enabled
        if not self.get_sim_lock_state():
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                     "SIM lock system (PIN code) must be enabled in order to change the PIN code")

        method = "changeSimPIN"
        method_args = "--es old_pin %s --es new_pin %s" % (current_pin, new_pin)
        self._internal_exec_v2(self.__module_name, method, method_args, is_system=True)
        #TODO : return the number of retries available

    def get_sim_lock_state(self):
        """
        Retrieve the current SIM lock system state, as displayed
        in "Settings -> Security -> Set up SIM card lock" menu.

        :rtype: bool
        :return: C{True} if the SIM lock system is enabled, C{False} otherwise
        """
        method = "isSimLockSystemEnabled"
        output = self._internal_exec_v2(self.__module_name, method, is_system=True)
        # retrieve state from output
        if output is None or "enabled" not in output:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to retrieve SIM lock system state")

        return str_to_bool(output["enabled"])

    def supply_pin(self, required_pin, is_password_incorrect=False):
        """
        Supply PIN code in order to unlock device at boot time.

        :param required_pin: PIN code required for unlocking
        device at boot time. Must represents a number containing
        4 to 8 digits.
        :type required_pin: str

        :type is_password_incorrect: bool
        :param is_password_incorrect: Boolean indicate that we
        need to supply a incorrect PIN for test purpose.
        default value is False

        :rtype: int
        :return: the number of tries left
        """
        self._logger.info("Supply PIN %s to the device" % required_pin)
        method = "supplyPin"
        method_args = "--es pin %s" % required_pin
        if not is_password_incorrect:
            self._internal_exec_v2(self.__module_name, method, method_args, is_system=True)
        else:
            # We test to supply wrong pin for specific tests
            try:
                self._internal_exec_v2(self.__module_name, method, method_args, is_system=True)
            except AcsBaseException as error:
                if self.INCORRECT_PASSWORD_MSG in error.get_error_message():
                    self._logger.warning("An incorrect PIN was supplied (normal behavior)")
                else:
                    raise error
        #TODO : return the number of retries available

    def supply_puk(self, puk, new_pin):
        """
        Supply PUK code in order to unlock device already PIN locked.
        Leave device with SIM lock system activated
         (with the new PIN code set).
        :param puk: PUK code that will be used to unlock SIM card.
        Represents a number of 8 digits.
        :type puk: str

        :param new_pin: New PIN code to set.
        Represents a number containing 4 to 8 digits.
        :type new_pin: str

        :rtype: int
        :return: the number of tries left
        """
        # check that puk & pin are corrects
        if is_puk_valid(puk) is False:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PUK value is incorrect : it must be a number containing 8 digits.")
        if is_pin_valid(new_pin) is False:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PIN value is incorrect : it must be a number of 4 to 8 digits")

        self._logger.info("Supply PUK %s to the device, and reset PIN code to %s." % (puk, new_pin))
        method = "supplyPuk"
        method_args = "--es puk %s --es pin %s" % (puk, new_pin)
        self._internal_exec_v2(self.__module_name, method, method_args, is_system=True)
        #TODO : return the number of retries available

    def get_sim_state(self, sim_index=1):
        """
        get the SIM state

        :rtype: int
        :return: the SIM state
        """
        method = "getSimState"
        module_class = "acscmd.telephony.SimCardModule"

        output = self._internal_exec_v2(module_class, method, is_system=True)
        sim_state = int(output["sim_state"])
        if sim_state not in self.POSSIBLE_SIM_STATES.values():
            self._logger.warning("get_sim_state(): read value from " +
                                 "Android SDK: %s. " % sim_state +
                                 "Known values in ACS Host:" +
                                 " %s" % str(self.POSSIBLE_SIM_STATES.values()))

        return sim_state

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
        state_reached = False

        if state not in self.POSSIBLE_SIM_STATES:
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "unknown state:" + state)
        str_state = str(state)

        msg = "Checks %s state before %s seconds..." % \
              (str_state, str(timeout))

        self._logger.info(msg)
        start_time = time.time()
        while (time.time() - start_time) <= timeout:
            current_state = self.get_sim_state()
            if current_state == self.POSSIBLE_SIM_STATES[state]:
                state_reached = True
                break

        if state_reached:
            return_msg = "State " + state + " has been reached !"
            self._logger.info(return_msg)
            return state
        else:
            return_msg = "State " + str_state + \
                         " has not been reached on time !"
            self._logger.error(return_msg)
            raise DeviceException(
                DeviceException.TIMEOUT_REACHED,
                return_msg)
