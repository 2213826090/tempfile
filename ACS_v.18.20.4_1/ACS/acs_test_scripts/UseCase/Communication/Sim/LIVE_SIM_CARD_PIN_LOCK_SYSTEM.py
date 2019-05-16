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
:summary: This file implements LIVE_SIM_CARD_PIN_LOCK_SYSTEM usecase
:since: 31/07/2013
:author: dgonza4x
"""
import time
from acs_test_scripts.Utilities.SimCardUtilities import is_pin_valid
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveSimCardPinLockSystem(UseCaseBase):
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
        self.__current_pin = self._device.get_default_pin_code()
        self.__new_pin = self._tc_parameters.get_param_value("NEW_PIN")
        self.__check_after_reboot = str_to_bool(
                self._tc_parameters.get_param_value(
                        "DOUBLE_CHECK_AFTER_REBOOT"))

        # This attribute will be used to know which PIN the usecase
        # should use in order to disable SIM lock system in tear_down
        self.__current_pin_code = None

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

        # Reset used pin for disabling SIM lock system for B2B purpose
        self.__current_pin_code = None

        # Check that current and new pin are correct
        if not is_pin_valid(self.__current_pin):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                        "Current PIN code is incorrect : it must be a number"
                        " containing 4 to 8 digits. You must override device "
                        "parameter 'defaultPINCode' in the bench config.")
        if not is_pin_valid(self.__new_pin):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                        "NEW_PIN testcase parameter is incorrect : it must "
                        "be a number containing 4 to 8 digits.")

        # Check that SIM state is SIM_STATE_READY
        sim_state = self.sim_card_api.get_sim_state()
        if sim_state != \
                self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_READY"]:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                   "SIM state is not ready : cancel test"
                   " execution for preventing the SIM to"
                   " be locked (PUK required)")

        # Ensure that flight mode is disabled
        self.networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # Assert that Device initial state is correct
        # in order to prevent SIM lock
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if is_enabled:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                        "SIM lock system already enabled : "
                        "cancel test for preventing the SIM to "
                        "be locked (PUK required)")

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

        # Enable SIM lock system
        self.sim_card_api.enable_sim_lock_system(self.__current_pin)

        # Check that SIM lock system is enabled
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled:
            return Global.FAILURE, "Enabling SIM lock system has failed"

        # SIM lock system enabled
        self.__current_pin_code = self.__current_pin

        if self.__check_after_reboot:
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

            # Check that the device is able to camp on a cell
            self.__modem_api.check_cdk_state_bfor_timeout(
                    ["searching", "registered", "roaming"],
                    self._device.get_uecmd_timeout())

        # Change PIN code from default (current) to new PIN
        self.sim_card_api.change_sim_pin(self.__current_pin, self.__new_pin)

        # If something goes wrong, we know at this point
        # that we should use NEW_PIN in tear_down
        # for disabling PIN lock system
        self.__current_pin_code = self.__new_pin

        # Check that SIM lock system is still enabled
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled:
            return (Global.FAILURE,
                    "SIM lock system should be still"\
                    " enabled after having PIN changed")

        # Change PIN code from new to default PIN (keep device to its initial
        # PIN after usecase, and without enabling again the SIM lock system)
        self.sim_card_api.change_sim_pin(self.__new_pin, self.__current_pin)

        # If something goes wrong, we know at this point
        # that we should use CURRENT_PIN in tear_down
        # for disabling PIN lock system
        self.__current_pin_code = self.__current_pin

        # Disable SIM lock system
        self.sim_card_api.disable_sim_lock_system(self.__current_pin)

        # Check that SIM lock system is disabled
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if is_enabled:
            return Global.FAILURE, "Disabling SIM lock system has failed"

        if self.__check_after_reboot:
            self.get_logger().info(
                    "Checking that SIM lock is still disabled after a reboot")
            # Reboot the device
            self._device.reboot(wait_settledown_duration=True)

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

        # test has been successful
        return Global.SUCCESS, "No error"

    def  tear_down(self):
        """
        Set the device to its original state :
        - disable SIM lock system if needed.
        """
        # Run the inherited 'tear_down' method
        UseCaseBase.tear_down(self)

        # Disable SIM lock system if needed
        # check parameter self.__current_pin_code
        if self.__current_pin_code is None:
            raise DeviceException(
                       DeviceException.PROHIBITIVE_BEHAVIOR,
                       "Disable SIM lock system cancelled for preventing"
                       "SIM to be locked (PUK required)")

        # Disable SIM lock system if needed
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if is_enabled:
            self.sim_card_api.disable_sim_lock_system(self.__current_pin_code)
            # Check that SIM lock system is disabled
            is_enabled = self.sim_card_api.get_sim_lock_state()
            if is_enabled:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                            "Disabling SIM lock system has failed")

        # tear down has been successful
        return Global.SUCCESS, "No error"
