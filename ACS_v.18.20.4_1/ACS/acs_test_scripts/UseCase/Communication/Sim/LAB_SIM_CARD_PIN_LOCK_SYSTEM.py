"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements LIVE_SIM_CARD_PIN_LOCK_SYSTEM usecase
@since: 31/07/2013
@author: dgonza4x
"""
import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.SimCardUtilities import is_pin_valid, is_puk_valid
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from random import randrange

class LabSimCardPinLockSystem(UseCaseBase):
    """
    Usecase asserting that we can enable, disable and change PIN
    (SIM lock feature) on Device
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Run the inherited '__init__' method
        UseCaseBase.__init__(self, tc_name, global_config)

        # retrieve SIM card UE commands
        self.sim_card_api = self._device.get_uecmd("SimCard")
        # retrieve Networking UE commands
        self.networking_api = self._device.get_uecmd("Networking")
        # retrieve Modem UE commands
        self.__modem_api = self._device.get_uecmd("Modem")

        # retrieve testcase parameters
        self.__puk_code = self._tc_parameters.get_param_value("PUKCode")
        self.__default_pin = self._tc_parameters.get_param_value("defaultPINCode")
        self.__new_pin = self._tc_parameters.get_param_value("NEW_PIN")
        self.__check_after_reboot = \
                str_to_bool(self._tc_parameters.get_param_value("DOUBLE_CHECK_AFTER_REBOOT"))

        if not is_pin_valid(self.__new_pin):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                        "NEW_PIN testcase parameter is incorrect : it must "
                        "be a number containing 4 to 8 digits.")

    def set_up(self):
        """
        Setup the device :
        - check that NEW_PIN parameter is correct
        - check that SIM state is 'SIM_STATE_READY'
        - check that default PIN code retrieved from phone is correct
        - disable flight mode
        - cancel test if SIM lock feature is
        already enabled, preventing board to get locked (PUK required).
        """
        # Run the inherited 'set_up' method
        UseCaseBase.set_up(self)

        # use default pin and PUK set sim pin back to defaault condition
        self.sim_card_api.set_sim_pin_to_default_condition (self.__puk_code, self.__default_pin)

        # turn on mobile broadband
        self.networking_api.reset_mobile_broadband_mode(1)

        # wait for sim is ready
        sim_state  = self.sim_card_api.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)
        if sim_state != self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_READY"]:
            self._logger.info("SIM could not reach READY state")
            return Global.FAILURE, "SIM could not reach READY state"


        # Setup check has been successful
        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Steps checking that we can ENABLE SIM lock feature:
        - enable SIM lock system with given PIN parameter
        - check that SIM lock system is enabled
        - if user want to check feature after reboot:
            - reboot the device
            - check that device is unregistered on any cell (device PIN locked)
            - supply PIN in order to unlock device
            - check that SIM lock system feature is still enabled
            - check that device is "registered" (or on going to be registered)
             on a cell

        Steps checking that we can CHANGE PIN code:
        - Change PIN code
        - Check that SIM lock system is still enabled
        - Change PIN back to default one (living device in original state)

        Steps checking that we can DISABLE SIM lock feature:
        - disable SIM lock system with given PIN parameter
        - check that SIM lock system is disabled
        - if user want to check feature after reboot:
            - reboot the device
            - check that SIM lock system is still disabled
            - check that device is "registered" (or on going to be registered)
             on a cell
        """
        # Run the inherited 'run_test' method
        UseCaseBase.run_test(self)

        # 1. Enable SIM lock system when we know the correct pin, as I have set sim back to default pin), and Check that SIM lock system is enabled
        try_left = self.sim_card_api.enable_sim_lock_system(self.__default_pin)
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled and try_left != self.sim_card_api.MAX_PIN_ATTEMPT:
            return Global.FAILURE, "Enabling SIM lock system has failed"
        self._logger.info("SIM is lock properly with default pin")


        # make this false, as tablet would not check pin until shutdown, and we have no connection to shutdown on dut after shutdown, so it could not be implemented for automation
        # for reboot code below is not verified if it works
        """
        if self.__check_after_reboot :
            self.get_logger().info(
                    "Checking that SIM lock is still enabled after a reboot")
            # Reboot the device
            self._device.reboot(wait_settledown_duration=True)

            # Check that the device state is unregistered
            self.__modem_api.check_cdk_state_bfor_timeout("unregistered",
                    self._device.get_uecmd_timeout())

            # Unlock SIM
            self.sim_card_api.supply_pin(self.__current_pin)

            # Check that SIM lock system is still enabled
            self._logger.info("Check that SIM lock system is still enabled")
            is_enabled = self.sim_card_api.get_sim_lock_state()
            if not is_enabled:
                return (Global.FAILURE,
                        "SIM lock System is not "\
                        "persistent after a device reboot")

            # if self.__check_after_reboot = true, we need to add code to provide an available network to check ["searching", "registered", "roaming"]
            # Check that the device is able to camp on a cell
            self.__modem_api.check_cdk_state_bfor_timeout(
                    ["searching", "registered", "roaming"],
                    self._device.get_uecmd_timeout())
        """

        # 2. Change PIN code from default (current) to new PIN, no need to check if the current pin is correct, as I have already know the pin is default pin
        self._logger.info("changing the sim pin from 4 digit default pin to 8 digits pin")
        try_left = self.sim_card_api.change_sim_pin(self.__default_pin, self.__new_pin)
        if try_left == self.sim_card_api.MAX_PIN_ATTEMPT:
            self._logger.info("pin changed sucessfully with 8 digits new pin %s, the pin trying counter is %s" % (self.__new_pin, try_left))
        # Check that SIM lock system is still enabled after change the pin
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled:
            return (Global.FAILURE, "failure: SIM lock system should not be disabled after having PIN changed to new pin")
        else:
            self._logger.info ("after SIM pin changed properly by using new pin %s, now we are removing the lock, as it is still locked after having PIN changed" % self.__new_pin)
            try_left = self.sim_card_api.disable_sim_lock_system(self.__new_pin)
            # Check that SIM lock system is disabled
            is_enabled = self.sim_card_api.get_sim_lock_state()
            if is_enabled and try_left != self.sim_card_api.MAX_PIN_ATTEMPT:
                return (Global.FAILURE, "SIM lock could not be disabled with new pin")
            else:       # sim was disabled properly
                # testing pin trying counter entering wrong pin once to reduce the remaining counter, by enable the pin with wrong pin
                self._logger.info ("using the new pin %s to unlock the sim is successful, and Changing pin function works properly" % self.__new_pin)

        # 3. testing pin entering counter, by entering wrong pin to see the counter changes.
        self._logger.info ("after SIM pin is removed successfully with new pin, now we are testing pin enter counter via entering wrong pin")
        bad_pin = str(randrange(10000)).zfill(((len(str(self.__new_pin)) - 3) % 5) + 4)
        try_left = self.sim_card_api.enable_sim_lock_system(bad_pin)
        is_enabled  = self.sim_card_api.get_sim_lock_state()
        if try_left == self.sim_card_api.MAX_PIN_ATTEMPT - 1 and not is_enabled:
            self._logger.info("entered a wrong pin %s, and pin enter remaining counter is %d" % (bad_pin, try_left))
        else:
            return (Global.FAILURE, "entered a wrong pin the %s, and pin enter remaining counter is %d" % (bad_pin, try_left))


        # 4. test if the pin goes back to MAX_PIN_ATTEMP before via entering a right pin
        try_left = self.sim_card_api.enable_sim_lock_system(self.__new_pin)
        is_enabled  = self.sim_card_api.get_sim_lock_state()
        if try_left == self.sim_card_api.MAX_PIN_ATTEMPT and is_enabled:
            self._logger.info("success: entered a right pin %s, and pin enter remaining counter is sent back to %d (%d is MAX_PIN_ATTEMP)" %  (self.__default_pin, try_left, self.sim_card_api.MAX_PIN_ATTEMPT))
        else:
            return (Global.FAILURE, "failure: entered a right pin the %s, and pin enter remaining counter is %d" % (self.__default_pin, try_left))

        # 5. testing puk counter. We only try one wrong puk code, as we do not want to reach max trying for puk
        if is_enabled:
            self._logger.info("to test puk trying counter, it using a wrong pin until pin is blocked to test the puk re-try counter")
            sim_state  = self.sim_card_api.get_sim_state()
            try_left == self.sim_card_api.MAX_PIN_ATTEMPT
            try_times = 1
            while sim_state != self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                bad_pin = str(randrange(10000)).zfill(((len(str(self.__new_pin)) - 3) % 5) + 4)
                try_left = self.sim_card_api.disable_sim_lock_system(bad_pin)
                sim_state  = self.sim_card_api.get_sim_state()
                self._logger.info("pin trying counter is %s after entered the wrong pin at the trying #%d" % (try_left, try_times))
                try_times = try_times + 1
            if try_left == 10 and sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                self._logger.info("pin MAX_PIN_ATTEMPT has reached, and sim is blocked and puk try counter is %d" % try_left)
                self._logger.info("testing puk counter via entering puk code wrong once")
                bad_puk = str(int(self.__puk_code) + int(self.__puk_code))
                try_lef = self.sim_card_api.supply_puk(bad_puk, self.__default_pin)
                sim_state  = self.sim_card_api.get_sim_state()
                try_times = 1
                if sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                    self._logger.info("current sim_state is %s, and puk remaining  counter is %d at the #%d try_times" % (self.sim_card_api.state_list[sim_state], try_left, try_times))
                    # unblock the sim with right puk and default pin
                    try_left = self.sim_card_api.supply_puk(self.__puk_code, self.__default_pin)
                    sim_state  = self.sim_card_api.get_sim_state()
                    if try_left == 3 and sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PIN_REQUIRED"]:
                        self._logger.info("current sim_state is %s, and puk remaining  counter is %d after unblock the sim pin" % (self.sim_card_api.state_list[sim_state], try_left))
                    else:
                        self._logger.info("current sim_state is %s, and puk remaining  counter is %d after unblock the sim pin" % (self.sim_card_api.state_list[sim_state], try_left))
                        return Global.FAILURE, "Unable to unblock SIM after entering %d times" % self.sim_card_api.MAX_PIN_ATTEMPT
            else:
                self._logger.info("Unable to block sim after %d tries" % self.sim_card_api.MAX_PIN_ATTEMPT)
                return Global.FAILURE, "Unable to block device SIM after %d tries" % self.sim_card_api.MAX_PIN_ATTEMPT


        """
        if self.__check_after_reboot:
            self.get_logger().info("Checking that SIM lock is still disabled after a reboot")
            # Reboot the device
            self._device.reboot(wait_settledown_duration=True)

            # if self.__check_after_reboot = true, we need to add code to provide an available network to check ["searching", "registered", "roaming"]
            # Check that the device is able to camp on a cell
            self.__modem_api.check_cdk_state_bfor_timeout(
                    ["searching", "registered", "roaming"],
                    self._device.get_uecmd_timeout())

            # Check that SIM lock system is still disabled
            self._logger.info("Check that SIM lock system is still disabled")
            is_enabled = self.sim_card_api.get_sim_lock_state()
            if is_enabled:
                return (Global.FAILURE,
                        "SIM lock System is not "\
                        "persistent after a device reboot")
        """
        # test has been successful
        return Global.SUCCESS, "No error"


    def  tear_down(self):
        """
        Set the device to its original state :
        - disable SIM lock system if needed.
        """
        # Run the inherited 'tear_down' method
        UseCaseBase.tear_down(self)

        # just in case any exception occured during testing, we need to use default pin and PUK set sim pin back to defaault condition
        self.sim_card_api.set_sim_pin_to_default_condition(self.__puk_code, self.__default_pin)

        # turn off mobile broadband
        self.networking_api.reset_mobile_broadband_mode(0)

        # tear down has been successful
        return Global.SUCCESS, "No error"
