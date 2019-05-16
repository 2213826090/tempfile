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
import time
from acs_test_scripts.Device.UECmd.Interface.Telephony.ISimCard import ISimCard
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Utilities.SimCardUtilities import is_pin_valid, is_puk_valid
from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class SimCard(Base, ISimCard):
    """
    SIM card UE commands operations for Windows platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    # Maximum authorized attemps
    MAX_PIN_ATTEMPT = 3
    MAX_PUK_ATTEMPT = 10

    # Output text of the Windows exception message when an incorrect
    # PIN is supplied to device.
    INCORRECT_PASSWORD_MSG = "PASSWORD_INCORRECT"

    # Constant Windows values for SIM_STATE
    POSSIBLE_SIM_STATES = {
        "SIM_STATE_UNKNOWN": 0,
        "SIM_STATE_ABSENT": 1,
        "SIM_STATE_PIN_REQUIRED": 2,
        "SIM_STATE_PUK_REQUIRED": 3,
        "SIM_STATE_NETWORK_LOCKED": 4,
        "SIM_STATE_READY": 5}

    state_list = ["SIM_STATE_UNKNOWN", "SIM_STATE_ABSENT", "SIM_STATE_PIN_REQUIRED", \
        "SIM_STATE_PUK_REQUIRED", "SIM_STATE_NETWORK_LOCKED", "SIM_STATE_READY"]

    @need('modem')
    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in interface.
        """
        ISimCard.__init__(self, device)
        Base.__init__(self, device)
        self._module_name = "Intel.Acs.TestFmk.MBConnectivity"
        self._class_name =  "Intel.Acs.TestFmk.MBConnectivity.MBActivity"
        self._logger = device.get_logger()

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
        try_left = self.__set_sim_lock_state("true", required_pin)
        return try_left

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
        try_left = self.__set_sim_lock_state("false", required_pin)
        return try_left

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
        #function = "SetSimLockState"
        function = "SetSimLockState"
        args = "enable=%s required_pin=%s" % (enable, required_pin)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args )

        try_left = int(output["values"]["remaining_enter_times"])
        return try_left

    def change_sim_pin(self, current_pin, new_pin):
        """
        Change the current pin code used in the SIM lock system,
        as in "Settings -> Security -> Set up SIM card lock" menu.

        :param current_pin: PIN code already set. Represents a number
        containing 4 to 8 digits.
        :type current_pin: str

        :param new_pin: New PIN code to set. Represents a number
        containing 4 to 8 digits.
        :type new_pin: str

        :rtype: None
        """
        self._logger.info("Changing PIN code from %s to %s" % (current_pin, new_pin))
        if not is_pin_valid(current_pin) or not is_pin_valid(new_pin):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Invalid PIN code (must represents a number containing 4 to 8 digits)")
        # check prerequisite : sim lock feature must be enabled
        if not self.get_sim_lock_state():
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                     "SIM lock system (PIN code) must be enabled in order to change the PIN code")

        function = "ChangeSimPIN"
        args = "old_pin=%s new_pin=%s" % (current_pin, new_pin)
        # args = "old_pin=%s new_pin=%s" % (current_pin, new_pin)
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        output = self._internal_uecmd_exec(module_name, class_name, function, args)

        try_left = int(output["values"]["remaining_enter_times"])
        return try_left

    def get_sim_lock_state(self):
        """
        Retrieve the current SIM lock system state, as displayed
        in "Settings -> Security -> Set up SIM card lock" menu.

        :rtype: bool
        :return: C{True} if the SIM lock system is enabled, C{False} otherwise
        """
        function = "GetSimLockState"
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        output = self._internal_uecmd_exec(module_name, class_name, function )

        sim_lock_state = output["values"]["sim_lock_state"]
        # retrieve state from output
        if output is None:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to retrieve SIM lock system state")

        return sim_lock_state

    def supply_pin(self, required_pin):
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

        :rtype: None
        """
        self._logger.info("Supply PIN %s to the device" % required_pin)
        function = "SupplyPin"
        args = "pin=%s" % required_pin
        # args = "pin=%s" % required_pin
        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        output = self._internal_uecmd_exec(module_name, class_name, function, args)

        try_left = int(output["values"]["remaining_enter_times"])
        return try_left

        # TODO
        # if not is_password_incorrect:
            # Launch the UEcmd on the embedded side
        #     output = self._internal_uecmd_exec(module_name, class_name, function, 15)
        # else:
            # We test to supply wrong pin for specific tests
        #    try:
        #        # Launch the UEcmd on the embedded side
        #        output = self._internal_uecmd_exec(module_name, class_name, function, 15)
        #    except AcsBaseException as error:
        #        if self.INCORRECT_PASSWORD_MSG in error.get_error_message():
        #            self._logger.warning("An incorrect PIN was supplied (normal behavior)")
        #        else:
        #            raise error

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

        :rtype: None
        """
        # check that puk & pin are corrects
        if is_puk_valid(puk) is False:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PUK value is incorrect : it must be a number containing 8 digits.")
        if is_pin_valid(new_pin) is False:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PIN value is incorrect : it must be a number of 4 to 8 digits")

        self._logger.info("Supply PUK %s to the device, and reset PIN code to %s." % (puk, new_pin))
        function = "SupplyPuk"
        args = "puk=%s pin=%s" % (puk, new_pin)


        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)

        try_left = int(output["values"]["remaining_enter_times"])
        return try_left

    def get_sim_state(self, sim_index=1):
        """
        get the SIM state

        :rtype: int
        :return: the SIM state
        POSSIBLE_SIM_STATES = {
        "SIM_STATE_UNKNOWN": 0,
        "SIM_STATE_ABSENT": 1,
        "SIM_STATE_PIN_REQUIRED": 2,
        "SIM_STATE_PUK_REQUIRED": 3,
        "SIM_STATE_NETWORK_LOCKED": 4,
        "SIM_STATE_READY": 5}

        """
        function = "GetSimState"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        sim_state = int(output["values"]["sim_state"])
        # if sim_state not in self.POSSIBLE_SIM_STATES.values():
        if sim_state not in self.POSSIBLE_SIM_STATES.values():
            self._logger.warning("get_sim_state(): read value from DUT %s sim_state;  Known values in ACS Host: %s" \
                                 % (sim_state, str(self.POSSIBLE_SIM_STATES.values())))

        return sim_state


    def check_sim_state_bfor_timeout(self, state, timeout):
        """
        Check the network registration state before timeout.

        :type state: string or list of several state
        :param state:
        POSSIBLE_SIM_STATES = {
        "SIM_STATE_UNKNOWN": 0,
        "SIM_STATE_ABSENT": 1,
        "SIM_STATE_PIN_REQUIRED": 2,
        "SIM_STATE_PUK_REQUIRED": 3,
        "SIM_STATE_NETWORK_LOCKED": 4,
        "SIM_STATE_READY": 5}
        :type timeout: int
        :param timeout: Time to wait attempting state

        :rtype: string
        :return: the state reached if state match or raise error
        """
        state_reached = False

        if state not in ["SIM_STATE_UNKNOWN", "SIM_STATE_ABSENT", "SIM_STATE_PIN_REQUIRED", \
                         "SIM_STATE_PUK_REQUIRED", "SIM_STATE_NETWORK_LOCKED", "SIM_STATE_READY"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "unknown state:" + element)

        msg = "Checks %s state before %s seconds..." % (state, str(timeout))
        self._logger.info(msg)
        start_time = time.time()

        while (time.time() - start_time) <= timeout:
            sim_state = self.get_sim_state()
            if sim_state == self.POSSIBLE_SIM_STATES[state]:
                state_reached = True
                break

        if state_reached:
            return_msg = "State " + state + " has been reached !"
            self._logger.info(return_msg)
            return sim_state
        else:
            return_msg = "State " + state + " has not been reached on time !"
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)

    def set_sim_pin_to_default_condition(self, puk, pin):
        """
        Unlock the SIM if it is locked. Supply PUK code to unlock device sim if it is PIN locked.

        :param puk: PUK code that will be used to unblock SIM card, representing 8 digits PUK code.
        :type puk: str
        :param puk: str
        :type pin: PIN code it is a default_pin when the function is called, and use it to reset sim pin to default)

        :rtype: None
        """

        self._logger.info("starting reset sim pin to default and unlock the sim.............")
        # check that puk & pin are corrects
        if is_puk_valid(puk) is False:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PUK value is incorrect : it must be a number containing 8 digits.")
        if is_pin_valid(pin) is False:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "PIN value is incorrect : it must be a number of 4 to 8 digits")

        # Check that device contain a SIM CARD
        sim_state = self.get_sim_state()

        if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_ABSENT"]:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "SIM state is SIM_STATE_ABSENT : cancel test"
                                  " execution")

        # when sim is ready, check that device's SIM is blocked,  if yes Unblock SIM
        if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
            # when sim is blocked, we need to unblock it and reset the pin to default pin
            self.supply_puk(puk, pin)
            self.check_sim_state_bfor_timeout("SIM_STATE_PIN_REQUIRED", 30)
            sim_state = self.get_sim_state()
            # after reset the sim pin, the sim is still lock, we need to unlock it
            try_left = self.disable_sim_lock_system(pin)
            is_enabled = self.get_sim_lock_state()
            if not is_enabled and try_left == self.MAX_PIN_ATTEMPT:
                self._logger.info("Device SIM unblocked and pin reset successfully")
                return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"

        # after checking if the sim blocked, Check that SIM lock system is enabled, or after sim is blocked, if yes unlock the sim
        is_enabled = self.get_sim_lock_state()
        if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_PIN_REQUIRED"] and is_enabled:
            # Disabling SIM lock system with default pin
            try_left = self.disable_sim_lock_system(pin)
            sim_state = self.get_sim_state()
            is_enabled = self.get_sim_lock_state()
            if try_left == self.MAX_PUK_ATTEMPT and stim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                self._logger.info("sim blocked, puk attemp counter left %d" % try_left)
                self.supply_puk(puk, pin)
                self.check_sim_state_bfor_timeout("SIM_STATE_PIN_REQUIRED", 30)
                sim_state = self.get_sim_state()
                # after reset the sim pin, the sim is still lock, we need to unlock it
                self.disable_sim_lock_system(pin)
                is_enabled = self.get_sim_lock_state()
                if not is_enabled:
                    self._logger.info("Device SIM unblocked and pin reset successfully")
                    return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"
                else:
                    self._logger.info("Device SIM could not be reset successfully")
                    return Global.FAILURE, "Device SIM could not be reset successfully"
            # Check that SIM lock system is disabled
            elif is_enabled and try_left != self.MAX_PIN_ATTEMPT:
                # after first try sim is still locked and attemp counter was reduced, which means the pin entered was wrong, we need to set it beck to defaul pin via PUK
                sim_state = self.get_sim_state()
                while is_enabled and try_left != 0 and sim_state != self.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                    try_left = self.disable_sim_lock_system(pin) # keep enter wrong pin to block it
                    is_enabled = self.get_sim_lock_state()
                    sim_state = self.get_sim_state()
                if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                    # now sim is blocked, need to unblock it and reset the pin to default pin
                    try_left = self.supply_puk(puk, pin)
                    self._logger.info("current sim_state is %s, and PUK enter counter remaining is %d" % (
                        self.state_list[sim_state], try_left))
                    self.check_sim_state_bfor_timeout("SIM_STATE_PIN_REQUIRED", 30)
                    sim_state = self.get_sim_state()
                    # after reset the sim pin with puk, the sim should still be locked, we need to unlock it
                    if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_PIN_REQUIRED"]:
                        self.disable_sim_lock_system(pin)
                        self.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)
                        is_enabled = self.get_sim_lock_state()
                        if not is_enabled:
                            self._logger.info("Device SIM unblocked and pin reset successfully")
                            return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"
                        else:
                            self._logger.info("Device SIM can not be unlocked during sim pin reset")
                            return Global.FAILURE, "Device SIM can not be unlocked during sim pin reset"
            elif is_enabled and try_left == self.MAX_PIN_ATTEMPT: # enter the correct pin during enable pin, we use the same pin to disable it
                try_left = self.disable_sim_lock_system(pin)
                sim_state = self.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)
                if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_READY"] and try_left == self.MAX_PIN_ATTEMPT:
                    self._logger.info("Device SIM unblocked and pin reset successfully")
                    return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"
                else:
                    self._logger.info("Device SIM could not be reset to default pin successfully")
                    return Global.FAILURE, "Device SIM could not be reset to default pin successfully"
            else: # pin was disabled
                self._logger.info("Device SIM unblocked and pin reset successfully")
                return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"

        else: # here we handle sim is at SIM_STATE_READY, and etc
            # when sim pin is not blocked or not enabled, we need to enable pin for changing it back to default pin
            try_left = self.enable_sim_lock_system(pin)
            is_enabled = self.get_sim_lock_state()
            if is_enabled and try_left == self.MAX_PIN_ATTEMPT: # meaning we used the correct pin (default pin), then we need to use the same pin to disable the sim lock and make the sim back to default condition
                try_left = self.disable_sim_lock_system(pin)
                sim_state = self.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)
                if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_READY"] and try_left == self.MAX_PIN_ATTEMPT:
                    self._logger.info("Device SIM unblocked and pin reset successfully")
                    return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"
                else:
                    self._logger.info("Device SIM could not be reset to default pin successfully")
                    return Global.FAILURE, "Device SIM could not be reset to default pin successfully"
            elif not is_enabled and try_left != self.MAX_PIN_ATTEMPT: # meaning the pin entered is wrong, keep enter the wrong pin to block it and then reset it
                self._logger.info("entered wrong pin, sim_state is %s, pin enter remaining counter is %d" % (
                    state_list[sim_state], try_left))
                while not is_enabled and try_left != 0 and sim_state != self.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                    self._logger.info("current sim_state is %s, and pin enter remaining  counter is %d" % (
                        state_list[sim_state], try_left))
                    try_left = self.enable_sim_lock_system(pin)
                    is_enabled = self.get_sim_lock_state()
                    sim_state = self.get_sim_state()
            if sim_state == self.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                # now sim is blocked, need to unblock it and reset the pin to default pin
                self._logger.info("entered wrong pin for 3 times, sim_state is %s, PUK enter remaining counter is %d" % (
                    state_list[sim_state], try_left))
                # we do not allows enter wrong puk
                self.supply_puk(puk, pin)
                self.check_sim_state_bfor_timeout("SIM_STATE_PIN_REQUIRED", 30)
                # after reset the sim pin, the sim is still lock, we need to unlock it
                self.disable_sim_lock_system(pin)
                self.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)
                is_enabled = self.get_sim_lock_state()
                if not is_enabled:
                    self._logger.info("Device SIM unblocked and pin reset successfully")
                    return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"
                else:
                    self._logger.info("Device SIM can not be unlocked during sim pin reset")
                    return Global.FAILURE, "Device SIM can not be unlocked during sim pin reset"

        self._logger.info("Device SIM unblocked and pin reset successfully")
        return Global.SUCCESS, "Device SIM unblocked and pin reset successfully"
