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
:summary: This file implements LIVE_SIM_CARD_PUK_LOCK_SYSTEM usecase
:since: 13/09/2013
:author: jreynaux
"""
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.SimCardUtilities import is_pin_valid, is_puk_valid
from UtilitiesFWK.Utilities import Global
from random import randrange
import time


class LiveSimCardPukLockSystem(UseCaseBase):
    """
    Usecase asserting that we can enable, enable PIN
    (SIM lock feature) and lock SIM (PUK request) on Device
    """
    # Maximum authorized attemps
    MAX_PIN_ATTEMPT = 3

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

        # retrieve device parameters
        self.__pin_code = self._device.get_default_pin_code()
        self.__puk_code = self._device.get_sim_puk_code()

    def set_up(self):
        """
        Setup the device :
        Device contains a SIM card, and SIM PIN check is activated on it.
        Device SIM is not blocked
        """
        # Run the inherited 'set_up' method
        UseCaseBase.set_up(self)

        # Check that current pin is correct
        if not is_pin_valid(self.__pin_code):
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                        "Current PIN code is incorrect : it must be a number"
                        " containing 4 to 8 digits. You must override device "
                        "parameter 'defaultPINCode' in the bench config.")

        # Check that current puk is correct
        if not is_puk_valid(self.__puk_code):
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                        "Current PUK code is incorrect : it must be a number"
                        " containing 8 digits. You must override device "
                        "parameter 'simPUKCode' in the bench config.")

        # Check that device contain a SIM CARD
        sim_state = self.sim_card_api.get_sim_state()
        if sim_state == \
                self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_ABSENT"]:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                   "SIM state is not present : cancel test"
                   " execution")

        # Check that device's SIM is not locked by PUK
        sim_state = self.sim_card_api.get_sim_state()
        if sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
            # Unlock SIM
            self.sim_card_api.supply_puk(self.__puk_code, self.__pin_code)

        # Ensure that flight mode is disabled
        self.networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        self.sim_card_api.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)

        # Make sure that Device initial state is correct
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled:
            self._logger.info("SIM lock system not yet enabled, enabling it")
            self.sim_card_api.enable_sim_lock_system(self.__pin_code)

        is_enabled = self.sim_card_api.get_sim_lock_state()

        if not is_enabled:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                        "SIM lock system not enabled : "
                        "cancel test because initial condition not "
                        "respected")

        # Setup check has been successful
        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Performs the three tests on SIM:
        1/ SIM PIN check (nominal scenario)
        2/ Lock the SIM PIN
        3/ Unlock it with SIM PUK
        """
        # Run the inherited 'run_test' method
        UseCaseBase.run_test(self)

        # 1/ SIM PIN check (nominal scenario)
        self._logger.info("Step 1: SIM PIN check (nominal scenario)...")
        (status, output) = self.do_sim_pin_check()

        # Stop on first failure to void device's sim brick
        if status != Global.SUCCESS:
            return status, output

        # 2/ Lock the SIM PIN:
        self._logger.info("Step 2: Lock SIM PIN ...")
        (status, output) = self.do_lock_sim_pin()

        # Stop on first failure to void device's sim brick
        if status != Global.SUCCESS:
            return status, output

        # 3/ Unlock it with SIM PUK
        self._logger.info("Step 3: Unlock SIM PIN with PUK ...")
        (status, output) = self.do_unlock_sim_puk()

        # test has been successful
        return status, output

    def tear_down(self):
        """
        Set the device to its original state :
        """
        # Run the inherited 'tear_down' method
        UseCaseBase.tear_down(self)

        # Disable SIM lock system if needed
        # check parameter self.__current_pin_code
        if self.__pin_code is None:
            raise DeviceException(
                       DeviceException.PROHIBITIVE_BEHAVIOR,
                       "Disable SIM lock system cancelled for preventing"
                       "SIM to be locked (PUK required)")

        # Disable SIM lock system if needed
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if is_enabled:
            self.sim_card_api.disable_sim_lock_system(self.__pin_code)
            # Check that SIM lock system is disabled
            is_enabled = self.sim_card_api.get_sim_lock_state()
            if is_enabled:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                            "Disabling SIM lock system has failed")

        # tear down has been successful
        return Global.SUCCESS, "No error"

    def do_sim_pin_check(self):
        """
        SIM PIN check (nominal scenario):
        - User boots the device: device ask for SIM PIN entry
        - User enters the SIM PIN code: phone interface appears, and device could
        register to cellular services

        :rtype: tuple
        :return: Global Status and Output log
        """
        # Reboot the device
        self._device.reboot(wait_settledown_duration=True)

        # Check that the device state is unregistered
        self._logger.debug("Check for state \"unregistered\" ")
        self.__modem_api.check_cdk_state_bfor_timeout("unregistered",
                self._device.get_uecmd_timeout())

        # Unlock SIM
        self.sim_card_api.supply_pin(self.__pin_code)

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

        return Global.SUCCESS, "No error"

    def do_lock_sim_pin(self):
        """
        Lock the SIM PIN:
        - User reboots the device: device ask for SIM PIN entry
        - User enters an incorrect PIN code: device report incorrect entry, and ask
        again for SIM PIN entry
        - loop previous step until the SIM PIN is locked, and then the device ask for
        the SIM PUK (SIM PIN unlock code)
        Depending on implementation, the use should not access to other device services
        (Android default)

        :rtype: tuple
        :return: Global Status and Output log
        """
        # Reboot the device
        self._device.reboot(wait_settledown_duration=True)

        # Check that the device state is unregistered
        self._logger.debug("Check for state \"unregistered\" ")
        self.__modem_api.check_cdk_state_bfor_timeout("unregistered",
                self._device.get_uecmd_timeout())

        self._logger.debug("Supply wrong PIN many time until lock SIM")
        sim_state = self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_READY"]
        try_count = 0
        while (sim_state !=
                   self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]
                or try_count < self.MAX_PIN_ATTEMPT):
            # Enter WRONG PIN
            pin = self._generate_bad_pin()
            self.sim_card_api.supply_pin(pin, is_password_incorrect=True)
            time.sleep(self._wait_btwn_cmd)
            sim_state = self.sim_card_api.get_sim_state()
            try_count += 1

        # Handle error case
        if try_count >= self.MAX_PIN_ATTEMPT and sim_state != \
                self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
            return Global.FAILURE, "Unable to lock device SIM after %d tries" % try_count

        # Check that the device state is unregistered
        self._logger.debug("Check for state \"unregistered\" ")
        self.__modem_api.check_cdk_state_bfor_timeout("unregistered",
                self._device.get_uecmd_timeout())

        return Global.SUCCESS, "Device SIM locked successfully"

    def do_unlock_sim_puk(self):
        """
        Unlock it with SIM PUK:
        - User enters the correct SIM PUK code: device ask for SIM PIN entry
        - User enters the correct SIM PIN code: phone interface appears, and device
        could register to cellular services

        :rtype: tuple
        :return: Global Status and Output log
        """
        # Reboot the device
        self._device.reboot(wait_settledown_duration=True)

        # Check that the device state is unregistered
        self._logger.debug("Check for state \"unregistered\"")
        self.__modem_api.check_cdk_state_bfor_timeout("unregistered",
                self._device.get_uecmd_timeout())

        self._logger.debug("Check for SIM state \"SIM_STATE_PUK_REQUIRED\"")
        self.sim_card_api.check_sim_state_bfor_timeout("SIM_STATE_PUK_REQUIRED", 30)

        self.sim_card_api.supply_puk(self.__puk_code, self.__pin_code)

        # Check that the device state is registered
        self._logger.debug("Check for state \"registered\"")
        self.__modem_api.check_cdk_state_bfor_timeout("registered",
                self._device.get_uecmd_timeout())

        return Global.SUCCESS, "Device SIM unlocked successfully"

    def _generate_bad_pin(self):
        """
        Generate a random pin code of 4 digits.

        .. note:: Use a modulo formula to ensure that the new pin
        is not the same than the one given in bench config

        :rtype: str
        :return: a random pin code
        """
        return str(randrange(10000)).zfill(((len(str(self.__pin_code)) - 3) % 5) + 4)
