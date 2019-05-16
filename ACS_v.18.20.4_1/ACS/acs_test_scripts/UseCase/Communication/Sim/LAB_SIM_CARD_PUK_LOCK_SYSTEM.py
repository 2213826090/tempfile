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
@summary: This file implements LIVE_SIM_CARD_PUK_LOCK_SYSTEM usecase
@since: 13/09/2013
@author: jreynaux
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
from acs_test_scripts.UseCase.Networking.LAB_EGPRS_BASE import LabEgprsBase



class LabSimCardPukLockSystem(LabEgprsBase):
    """
    Usecase asserting that we can enable, enable PIN
    (SIM lock feature) and lock SIM (PUK request) on Device
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Run the inherited '__init__' method
        # UseCaseBase.init__(self, tc_name, global_config)
        LabEgprsBase.__init__(self, tc_name, global_config)


        # retrieve SIM card UE commands
        self.sim_card_api = self._device.get_uecmd("SimCard")
        # retrieve Networking UE commands
        self.networking_api = self._device.get_uecmd("Networking")
        # retrieve Modem UE commands
        self.__modem_api = self._device.get_uecmd("Modem")

        # retrieve device parameters
        # self.__pin_code = self._device.get_default_pin_code()
        self.__puk_code = self._tc_parameters.get_param_value("PUKCode")
        # get default pin  from test case .xml
        self.__default_pin = self._tc_parameters.get_param_value("defaultPINCode")

    def set_up(self):
        """
        Setup the device :
        Device contains a SIM card, and SIM PIN check is activated on it.
        Device SIM is not blocked
        """
        # Run the inherited 'set_up' method
        # UseCaseBase.set_up(self)
        LabEgprsBase.set_up(self)

        #  we need to use default pin and PUK set sim pin back to defaault condition
        self.sim_card_api.set_sim_pin_to_default_condition(self.__puk_code, self.__default_pin)

        # wait for sim is ready
        self.sim_card_api.check_sim_state_bfor_timeout("SIM_STATE_READY", 30)

        # Make sim pin on Device to initial state
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled:
            self._logger.info("SIM lock system not yet enabled, enabling it")
            self.sim_card_api.enable_sim_lock_system(self.__default_pin)

        # check if it sim pin is enabled
        is_enabled = self.sim_card_api.get_sim_lock_state()

        if not is_enabled:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                        "SIM lock system can not be enabled : "
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

        # 1/ SIM PIN check - enter the pin when sim is locked (nominal scenario)
        self._logger.info("Step 1: do SIM PIN check......")
        is_enabled = self.sim_card_api.get_sim_lock_state()
        sim_state = self.sim_card_api.get_sim_state()
        if is_enabled and sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PIN_REQUIRED"]:
            self._logger.info("sim pin is enabled, dut will not ask for the pin to register until next shutdown, actually we do not need to supply pin to regitere to network without shutdown")
            (status, output) = self.do_sim_pin_check()
            self._logger.info("success: sim pin is enabled properly !!")
        # Stop on first failure to void device's sim brick
        if status != Global.SUCCESS:
            return status, output


        # 2/ block the SIM PIN:
        self._logger.info("Step 2: block SIM PIN test by entering wrong pin 3 times...")
        (status, output) = self.do_block_sim_pin()
        sim_state   = self.sim_card_api.get_sim_state()
        if sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
            self._logger.info("sim pin is blocked after entering wrong pin %d times" % self.sim_card_api.MAX_PIN_ATTEMPT)

        if status != Global.SUCCESS:
            return status, output

        # 3/ Unblock it with SIM PUK
        self._logger.info("Step 3: Unlock SIM PIN with PUK ...")
        (status, output) = self.do_unblock_sim_puk()

        # test has been successful
        return status, output

    def tear_down(self):
        """
        Set the device to its original state :
        """
        # Run the inherited 'tear_down' method
        # UseCaseBase.tear_down(self)
        LabHspaBase.tear_down(self)

        # just in case any exception occured during testing, we need to use default pin and PUK set sim pin back to defaault condition
        self.sim_card_api.set_sim_pin_to_default_condition(self.__puk_code, self.__default_pin)


        # turn off mobile broadband
        self.networking_api.reset_mobile_broadband_mode(0)

        # tear down has been successful
        return Global.SUCCESS, "No error"

    def do_sim_pin_check(self):
        """
        SIM PIN check (nominal scenario):
        - User boots the device: device ask for SIM PIN entry
        - User enters the SIM PIN code: phone interface appears, and device could
        register to cellular services

        @rtype: tuple
        @return: Global Status and Output log
        """
        # Check that the device state is unregistered whhen the pin is  not entered
        # for tablet dut does not check pin for registere until next shutdown, not even with restart)
        """
        self._logger.debug("Check for state \"unregistered\" ")
        self.__modem_api.check_cdk_state_bfor_timeout("unregistered",
                self._device.get_uecmd_timeout())
        """

        # Unlock SIM
        self.sim_card_api.supply_pin(self.__default_pin)

        # Check that SIM is still enabled after unlocking
        self._logger.info("Check after pin is entered, sim pin is still enabled")
        is_enabled = self.sim_card_api.get_sim_lock_state()
        if not is_enabled:
            return (Global.FAILURE,
                    "SIM is not unlocked properly")

        # Check that the device is able to camp on a cell
        self.__modem_api.check_cdk_state_bfor_timeout(
                ["searching", "registered", "roaming"],
                self._device.get_uecmd_timeout())

        return Global.SUCCESS, "No error"

    def do_block_sim_pin(self):
        """
        Lock the SIM PIN:
        - User reboots the device: device ask for SIM PIN entry
        - User enters an incorrect PIN code: device report incorrect entry, and ask
        again for SIM PIN entry
        - loop previous step until the SIM PIN is locked, and then the device ask for
        the SIM PUK (SIM PIN unlock code)
        Depending on implementation, the use should not access to other device services
        (Android default)

        @rtype: tuple
        @return: Global Status and Output log
        """

        self._logger.info("Supply wrong PIN many time until block SIM")
        sim_state  = self.sim_card_api.get_sim_state()
        self._logger.info("before entering wrong pin, sim_state is %s" % self.sim_card_api.state_list[sim_state])
        while sim_state != self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
            # Enter WRONG PIN
            bad_pin = self.generate_bad_pin(self.__default_pin)
            remaining_counter = self.sim_card_api.supply_pin(bad_pin)
            time.sleep(self._wait_btwn_cmd)
            sim_state  = self.sim_card_api.get_sim_state()
            self._logger.info("current sim_state is %s, and wrong pin entered %s and pin/puk trying remaining counter is %d" % (self.sim_card_api.state_list[sim_state], bad_pin, remaining_counter))

        # Handle error case
        if remaining_counter != 10 and sim_state != \
            self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
            return Global.FAILURE, "Unable to lock device SIM after %d tries" % self.sim_card_api.MAX_PIN_ATTEMPT

        # handle success block sim case
        elif remaining_counter == 10 and sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                self._logger.info("pin MAX_PIN_ATTEMPT has reached, and sim is blocked and puk try counter is %d" % remaining_counter)
                self._logger.info("testing puk counter via entering puk code wrong once")
                bad_puk = str(int(self.__puk_code) + int(self.__puk_code))
                try_left = self.sim_card_api.supply_puk(bad_puk, self.__default_pin)
                sim_state  = self.sim_card_api.get_sim_state()
                try_times = 1
                if sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PUK_REQUIRED"]:
                    self._logger.info("current sim_state is %s, and puk remaining  counter is %d at the #%d try_times" % (self.sim_card_api.state_list[sim_state], try_left, try_times))
                else:
                    self._logger.info("Unable to block sim after %d tries" % self.sim_card_api.MAX_PIN_ATTEMPT)
                    return Global.FAILURE, "Unable to block device SIM after %d tries" % self.sim_card_api.MAX_PIN_ATTEMPT

        return Global.SUCCESS, "Device SIM blocked successfully"

    def do_unblock_sim_puk(self):
        """
        Unlock it with SIM PUK:
        - User enters the correct SIM PUK code: device ask for SIM PIN entry
        - User enters the correct SIM PIN code: phone interface appears, and device
        could register to cellular services

        @rtype: tuple
        @return: Global Status and Output log
        """

        self._logger.debug("unblock SIM  when sim state is \"SIM_STATE_PUK_REQUIRED\"")
        """
        self.sim_card_api.check_sim_state_bfor_timeout("SIM_STATE_PUK_REQUIRED", 30)

        self.sim_card_api.supply_puk(self.__puk_code, self.__default_pin)

        # Check that the device state is registered
        self._logger.debug("Check for state \"registered\"")
        self.__modem_api.check_cdk_state_bfor_timeout("registered",
                self._device.get_uecmd_timeout())
        """

        # unblock the sim with right puk and default pin
        try_left = self.sim_card_api.supply_puk(self.__puk_code, self.__default_pin)
        sim_state  = self.sim_card_api.get_sim_state()
        if try_left == 3 and sim_state == self.sim_card_api.POSSIBLE_SIM_STATES["SIM_STATE_PIN_REQUIRED"]:
            self._logger.info("current sim_state is %s, and puk remaining  counter is %d after unblock the sim pin" % (self.sim_card_api.state_list[sim_state], try_left))
        else:
            self._logger.info("current sim_state is %s, and puk remaining  counter is %d after unblock the sim pin" % (self.sim_card_api.state_list[sim_state], try_left))
            return Global.FAILURE, "Unable to unblock SIM after entering %d times" % self.sim_card_api.MAX_PIN_ATTEMPT

        return Global.SUCCESS, "Device SIM unlocked successfully"

    def generate_bad_pin(self, pin_code):
        """
        Generate a random pin code of 4 digits.

        @note: Use a modulo formula to ensure that the new pin
        is not the same than the one given in bench config

        @rtype: str
        @return: a random pin code
        """
        return str(randrange(10000)).zfill(((len(str(pin_code)) - 3) % 5) + 4)
