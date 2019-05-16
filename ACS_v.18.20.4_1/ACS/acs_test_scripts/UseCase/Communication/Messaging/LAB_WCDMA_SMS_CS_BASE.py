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
:summary: Use Case WCDMA SMS Base for SMOKE and BAT tests
:since: 19/08/2010
:author: dgo
"""

import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabWcdmaSmsCsBase(UseCaseBase):

    """
    Use Case WCDMA SMS base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")
        self._apn = self._network.get_param_value("APN")
        self._ssid = self._network.get_param_value("SSID")

        # Retrieve valid bench name for 3G capability
        self._bench_name = get_nw_sim_bench_name("3G", global_config, self._logger)
        # bench_name should be either NETWORK_SIMULATOR1 or NETWORK_SIMULATOR2
        # In both cases, we need to retrieve the NS number (position in the bench config, either 1 or 2)
        self._ns_number = int(self._bench_name[-1])

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        # Retrieve the model of the equipment
        self._ns_model = self._ns_node.get_param_value("Model")

        # Read BAND from xml UseCase parameter file
        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")

        # NS_CELL_REL
        self._ns_cell_rel = 7

        # Read CELL_POWER from xml UseCase parameter file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read DATA_CODING_SCHEME from xml UseCase parameter file
        self._data_coding_sheme = \
            self._tc_parameters.get_param_value("DATA_CODING_SCHEME")

        # Read SMS_TEXT from xml UseCase parameter file
        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT")

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        dcs = DataCodingScheme(self._data_coding_sheme)
        dcs.decode()

        # Read Home Public Land Mobile Network (HPLMN) Coverage
        # from test case xml file and accept all the different
        # written for True and False values
        self._hplmn_coverage = \
            self._tc_parameters.get_param_value("HPLMN_COVERAGE", "True")

        # Read Mobile Country Code (MCC) from test case xml file
        self._mcc = \
            int(self._tc_parameters.get_param_value("MCC", "1"))

        # Read Mobile Network Code (MNC) from test case xml file
        self._mnc = \
            int(self._tc_parameters.get_param_value("MNC", "1"))

        # Get number of bits per character setted in DCS
        self._nb_bits_per_char = dcs.compute_character_size()

        character_set = dcs.get_character_set()
        if character_set == "7BITS":
            self._content_type = "CTEX"
        else:
            self._content_type = "CDAT"

        # Instantiate Messaging UECmd for SMS UseCases
        self._messaging_api = self._device.get_uecmd("SmsMessaging")

        # Instantiate Modem UECmd for checking phone registration
        self._modem_api = self._device.get_uecmd("Modem")

        # Instantiate generic UECmd for camp
        self._networking_api = self._device.get_uecmd("Networking")

        # Create cellular network simulator and retrieve 3G messaging API
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_messaging_3g = self._ns_cell_3g.get_messaging()
        self._ns_data_3g = self._ns_cell_3g.get_data()

        # init wanted registration parameters
        self._wanted_reg_state = "None"

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # Determinates the kind of registration state to wait
        # by comparing test case MCC/MNC parameters to sim MCC/MNC
        sim_info = self._modem_api.get_sim_operator_info()
        if sim_info["MCC"] != self._mcc or\
                sim_info["MNC"] != self._mnc:
            self._wanted_reg_state = "roaming"
            self._networking_api.set_roaming_mode("ON")
        else:
            self._wanted_reg_state = "registered"

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Enable flight mode
        self._networking_api.set_flight_mode("on")

        hpmln_message = "Expected network registration state to be %s while " \
                        % self._wanted_reg_state

        self._logger.info(hpmln_message)

        # Connect to equipment
        self._ns.init()

        # Set the equipment Application Format = "WCDMA"
        self._ns.switch_app_format("WCDMA")

        # Perform full preset
        self._ns.perform_full_preset()

        # Deactivate HSUPA and HSDPA capabilities
        self._ns_data_3g.set_edch_cell_capability("OFF")
        self._ns_data_3g.set_hsdpa_cell_capability("OFF")

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Set Mobile Country Code (MCC)
        self._ns_cell_3g.set_mcc(self._mcc)
        # Set Mobile Network Code (MNC)
        self._ns_cell_3g.set_mnc(self._mnc)

        hpmln_message = "Expected network registration state to be %s while " \
            "HPMLN coverage is %s, MCCODE is %s and MNCODE is %s" % (
                str(self._wanted_reg_state),
                str(self._hplmn_coverage),
                str(self._mcc),
                str(self._mnc))

        self._logger.info(hpmln_message)

        # 3G default cell setup
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           "3G",
                           self._band_name,
                           self._ns_cell_rel,
                           self._logger)

        # Set cell power using CELL_POWER value
        self._ns_cell_3g.set_cell_power(self._cell_power)

        # Set cell on
        self._ns_cell_3g.set_cell_on()

        # Disable airplane mode
        self._networking_api.set_flight_mode("off")

        # Check registration status before time out using
        # registrationTimeout value from Device_Catalog.xml
        time.sleep(self._wait_btwn_cmd)
        dut_imsi = self._modem_api.get_imsi(self._registration_timeout)

        RegUtil.check_dut_registration_before_timeout(self._ns_cell_3g,
                                                      self._networking_api,
                                                      self._logger,
                                                      dut_imsi,
                                                      self._registration_timeout)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._modem_api.check_cdk_state_bfor_timeout(self._wanted_reg_state,
                                                     self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        time.sleep(self._wait_btwn_cmd)
        # Check Data Connection State => PDP_ACTIVE before timeout
        self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout,
                                                     blocking=False)

        # Get RAT from Equipment
        network_type = self._ns_data_3g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call use case base Setup function
        UseCaseBase.run_test(self)

        if self._wanted_reg_state == "roaming":
            # Dectivate Roaming mode
            self._networking_api.set_roaming_mode("OFF")
            time.sleep(self._wait_btwn_cmd)

            # Check Data Connection State => ATTAched before timeout
            self._ns_data_3g.check_data_connection_state("ATTACHED",
                                                         self._registration_timeout,
                                                         blocking=False)

            # Check that DUT is registered on the good RAT
            state = self._modem_api.get_network_registration_status()
            self._logger.info("the network registration is in %s state" % state)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        # Clear old SMS on 8960
        self._ns_messaging_3g.clear_message_data()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        try:
            # Activate the flight mode.
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Disconnect from equipment
        self._ns.release()

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"
