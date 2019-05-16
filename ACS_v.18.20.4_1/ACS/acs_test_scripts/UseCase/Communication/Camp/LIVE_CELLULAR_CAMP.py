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
:summary: Use Case Cellular Live Camp
:since: 19/11/2013
:author: lvacheyx
"""
# Module name follows ACS conventions
# pylint: disable=C0103

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.Utilities.RegistrationUtilities import check_mnc_mcc_range, check_mnc_mcc
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveCellularCamp(UseCaseBase):
    """
    Live Cellular Camp base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Initialize basic attributes for the Use Case
        self._is_operator_checked = False
        self._is_operator_2_checked = False
        self._deactivate_data = None
        self._initial_pdp_context_status = None
        self._initial_pref_network = None
        self._initial_flight_mode_state = None
        self._network_pref = None

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read Home Public Land Mobile Network (HPLMN) Coverage
        # from test case xml file and accept all the different
        # written for True and False values
        hplmn_coverage_value = \
            self._tc_parameters.get_param_value("HPLMN_COVERAGE")
        if hplmn_coverage_value not in (None, ''):
            self._hplmn_coverage = (str(hplmn_coverage_value).lower() == "true")

        # Read the parameter indicating whether we shall deactivate
        # data during the test or not.
        deactivate_data = self._tc_parameters.get_param_value("DEACTIVATE_DATA")
        if deactivate_data not in ("", None):
            self._deactivate_data = str_to_bool(deactivate_data)
        else:
            self._deactivate_data = False

        # Read Mobile Country Code (MCC) from test case xml file
        self._mcc = int(self._tc_parameters.get_param_value("MCC"))

        # Read Mobile Country Code (MCC_2) from test case xml file
        self._mcc2 = int(self._tc_parameters.get_param_value("MCC_2","0"))

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = int(self._tc_parameters.get_param_value("MNC"))

        # Read Mobile Network Code (MNC_2) from test case xml file
        self._mnc2 = int(self._tc_parameters.get_param_value("MNC_2","0"))

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read PREFERRED_NETWORK_TYPE from xml parameters
        self._network_pref = self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE")
        if self._network_pref:
            self._network_pref = self._network_pref.upper()

        # Read the parameter indicating whether we shall check
        # data during the test or not.
        self.__check_data = self._tc_parameters.get_param_value("CHECK_DATA", False, "str_to_bool")

        # Read the parameter indicating whether we shall reboot
        # the DUT instead of setting flight mode to off then to on
        self.__reboot = self._tc_parameters.get_param_value("REBOOT", False, "str_to_bool")

        # Instantiate generic UECmd for camp
        self._modem_api = self._device.get_uecmd("Modem")
        # retrieve SIM card UE commands
        self.sim_card_api = self._device.get_uecmd("SimCard")
        self._sim_states = self.sim_card_api.POSSIBLE_SIM_STATES

        # Instantiate generic UECmd for networking
        self._networking_api = self._device.get_uecmd("Networking")

        # Instantiate the PhoneSystem UE Command category
        self._phone_system = self._device.get_uecmd("PhoneSystem")

        self._expected_reg_state = None

    #------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        # Call UseCase base set_up function
        UseCaseBase.set_up(self)

        # User knows if the phone is within HPLMN coverage
        # If the phone is within coverage
        if self._hplmn_coverage:
            # Registration expected result shall be 'Registered'
            self._expected_reg_state = "registered"
        # Else roaming in the 'Roaming'
        else:
            self._expected_reg_state = "roaming"

        hpmln_message = "Expected network registration state to be %s while " \
                        "HPMLN coverage is %s" % (
                            str(self._expected_reg_state),
                            str(self._hplmn_coverage))
        self._logger.info(hpmln_message)

        # Check that parameters CHECK_DATA and DEACTIVATE_DATA are correctly
        # set in TC
        if self.__check_data and self._deactivate_data:
            error_msg = "parameters CHECK_DATA and DEACTIVATE_DATA should not have the same value ,please update TC"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Deactivate data if requested
        if self._deactivate_data:
            # First store the initial PDP context status
            # pylint: disable=W0212
            self._initial_pdp_context_status = \
                self._networking_api._get_pdp_context_status()
            # pylint: enable=W0212
            # Check whether we had to change the PDP context status
            if self._initial_pdp_context_status in ("0", "1"):
                # Then deactivate PDP context status
                self._networking_api.deactivate_pdp_context()

        # Activate data if requested
        if self.__check_data:
            # First store the initial PDP context status
            self._initial_pdp_context_status = self._networking_api._get_pdp_context_status()
            time.sleep(self._wait_btwn_cmd)

            # Check whether we had to change the PDP context status
            if self._initial_pdp_context_status not in ("0", "1"):
                # Then activate PDP context status
                self._networking_api.activate_pdp_context()

        # Check initial flight mode state
        self._initial_flight_mode_state = self._networking_api.get_flight_mode()

        # must turn flight mode on to set prefered data class
        if self._initial_flight_mode_state == "off":
            self._networking_api.set_flight_mode("on")

        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No error."

    #------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base run_test function
        UseCaseBase.run_test(self)

        # Initialize a global test message
        test_message = "No errors."

        self._initial_pref_network = self._dut_config.get("defaultPreferredNetwork")
        time.sleep(self._wait_btwn_cmd)

        # Activate preferred network
        if self._network_pref is None:
            # If there is no Network preference in the testcase.
            self._logger.warning("No preferred network set in the testcase")
        elif self._networking_api.is_preferred_network_type_valid(self._network_pref):
            # Setting the DUT preferred network type to the one specified
            # in the TC.
            self._networking_api.set_preferred_network_type(self._network_pref)
            time.sleep(self._wait_btwn_cmd)
            # Check the DUT is camped on a compatible network with the selected
            # preferred network.

            # need to turn on dut and wait for register to check rat
            self._networking_api.set_flight_mode("off")

            self._modem_api.check_rat_with_pref_network(self._network_pref, self._registration_timeout)
            time.sleep(self._wait_btwn_cmd)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown network type: %s" % self._network_pref)

        if self.__reboot:
            self._device.reboot()
        else:
            # Enable flight mode and wait
            self._networking_api.set_flight_mode("on")
            time.sleep(self._wait_btwn_cmd)

            # Check the reg state is unregistered
            self._logger.info(
                "Check current network registration status is not registered...")
            self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

            # Disable flight mode and wait
            self._networking_api.set_flight_mode("off")
            time.sleep(self._wait_btwn_cmd)

        # Check registration state
        self._logger.info(
            "Expected network registration state is %s; Check current "
            "network registration state..." % str(self._expected_reg_state))

        self._modem_api.check_cdk_state_bfor_timeout(self._expected_reg_state,
                                                     self._registration_timeout)

        # Check the DUT is camped on a compatible network with the selected
        # preferred network.
        if self._network_pref is not None:
            self._modem_api.check_rat_with_pref_network(self._network_pref, self._registration_timeout)
            time.sleep(self._wait_btwn_cmd)

        # Get MCC & MNC (and MCC_2 & MNC_2) from phone
        cellular_info = self._modem_api.get_cellular_operator_info()
        self._logger.debug("CELLULAR INFO: %s" % str(cellular_info))

        # Now that phone has registered, let's check that operator
        # codes if required
        if self._is_operator_checked:
            # Retrieve the SIM 1 state for information
            sim_state = self.sim_card_api.get_sim_state(1)
            sim_state = self.__find_sim_state(sim_state)
            test_message = "Reported states: [SIM1]: %s" % str(sim_state)
            self._logger.info("[SIM1] SIM state: %s" % str(sim_state))
            # At this point check whether we are testing a DSDS phone
            if self._is_operator_2_checked:
                # If that is the case we will also provide a SIM name
                # so that possible error messages are more explicit
                sim_name = "SIM_1"
            else:
                sim_name = None

            # Check the MNC / MCC values
            check_mnc_mcc_range(self._mnc, self._mcc)

            # Compare retrieved and expected values for MNC / MCC
            check_mnc_mcc(
                cellular_info["MNC"],
                cellular_info["MCC"],
                self._mnc,
                self._mcc,
                sim_name)

        # Check camp on SIM_2 if requested.
        if self._is_operator_2_checked:
            # Retrieve the SIM 2 state for information
            sim_state = self.sim_card_api.get_sim_state(2)
            sim_state = self.__find_sim_state(sim_state)
            test_message += ", [SIM2]: %s" % str(sim_state)
            self._logger.info("[SIM2] SIM state: %s" % str(sim_state))
            # If that is the case we will also provide a SIM name
            # so that possible error messages are more explicit
            sim_name = "SIM_2"

            # Check the MNC / MCC values
            check_mnc_mcc_range(self._mnc2, self._mcc2)

            # Compare retrieved and expected values for MNC / MCC
            check_mnc_mcc(
                cellular_info["MNC_2"],
                cellular_info["MCC_2"],
                self._mnc2,
                self._mcc2,
                sim_name)

        # At this point, no exception has been raised,
        # Check data status if required
        if self.__check_data:
            if self._networking_api._get_pdp_context_status() not in ("0", "1"):
                test_message = "Data is not activated"
                return Global.FAILURE, test_message

        # return a successful verdict
        test_message += "."
        return Global.SUCCESS, test_message

    #------------------------------------------------------------------------------

    def tear_down(self):
        """
        Dispose the test.
        """
        # Call UseCase base run_test function
        UseCaseBase.tear_down(self)

        # Activate data if it has been disabled previously
        # (restore previous phone state).
        if self._deactivate_data:
            # Check whether we had to change the PDP context status
            if self._initial_pdp_context_status in ("0", "1"):
                # Activate PDP context
                self._networking_api.activate_pdp_context()

        if self.__check_data:
            # Check whether we had to change the PDP context status
            if self._initial_pdp_context_status not in ("0", "1"):
                # Deactivate PDP context
                self._networking_api.deactivate_pdp_context()

        # Set the initial flight mode
        self._networking_api.set_flight_mode(self._initial_flight_mode_state)
        time.sleep(self._wait_btwn_cmd)

        # Set initial network state
        if self._initial_pref_network is not None:
            self._networking_api.set_preferred_network_type(self._initial_pref_network)
            time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No error."

    def __find_sim_state(self, sim_state_int):
        """
        Returns a str representation of the given I{SIM}
        state given as int.

        :type sim_state_int: int
        :param sim_state_int: the I{SIM} state as integer

        :rtype: str
        :return: the I{SIM} state as str
        """
        sim_state = "Unsupported value returned from DUT."
        for key, value in self._sim_states.items():
            if value == sim_state_int:
                sim_state = key
                break
        return sim_state
