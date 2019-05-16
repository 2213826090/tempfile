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
:summary: Use Case EGPRS SMS Base for SMOKE and BAT tests
:since: 10/02/2011
:author: ssavrimoutou
"""

import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager as EM
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabEgprsSmsPsBase(UseCaseBase):

    """
    Use Case EGPRS SMS base class.
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

        # Retrieve valid bench name for 2G capability
        self._bench_name = get_nw_sim_bench_name("2G", global_config, self._logger)

        # Read NETWORK_SIMULATOR from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters(self._bench_name)

        # Read the CELL_BAND value from UseCase xml Parameter
        self._band_name = self._tc_parameters.get_param_value("CELL_BAND")

        # Read the CELL_POWER value from UseCase xml Parameter
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read the BCH_ARFCN value from UseCase xml Parameter
        self._bch_arfcn = \
            int(self._tc_parameters.get_param_value("BCH_ARFCN"))

        # Read the PDTCH_ARFCN value from UseCase xml Parameter
        self._pdtch_arfcn = \
            int(self._tc_parameters.get_param_value("PDTCH_ARFCN"))

        # Read the DATA_CODING_SCHEME from UseCase xml Parameter
        self._data_coding_sheme = \
            str(self._tc_parameters.get_param_value("DATA_CODING_SCHEME"))

        # Read the SMS_TEXT from UseCase xml Parameter
        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT")

        # Read the SMS_TRANSFER_TIMEOUT from UseCase xml Parameter
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        # Get number of bits per character set
        dcs = DataCodingScheme(self._data_coding_sheme)
        dcs.decode()

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

        # Create cellular network simulator
        eqt_man = EM()
        self._ns = eqt_man.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_messaging_2g = self._ns_cell_2g.get_messaging()
        self._ns_data_2g = self._ns_cell_2g.get_data()

        # Instantiate generic UECmd for camp
        self._networking_api = self._device.get_uecmd("Networking")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # Clear all data connections
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Connect to equipment
        self._ns.init()

        # Set the equipment Application Format = "GSM/GPRS"
        self._ns.switch_app_format("GSM/GPRS")

        # Perform full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns.configure_amplitude_offset_table()

        # Set serving cell to EGPRS
        self._ns_cell_2g.set_cell_service("EGPRS")

        # Set cell power using CELL_POWER value
        self._ns_cell_2g.set_cell_power(self._cell_power)

        # Select the band
        self._ns_cell_2g.set_band(self._band_name)

        # Set broadcast channel ARFCN using BCH_ARFCN value
        self._ns_cell_2g.set_bcch_arfcn(self._bch_arfcn)

        # Set packet data channel ARFCN using PDTCH_ARFCN value
        self._ns_cell_2g.set_pdtch_arfcn(self._pdtch_arfcn)

        # Set cell on
        self._ns_cell_2g.set_cell_on()

        # Check Data Connection State => ATTACHED before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("ATTACHED",
                                                               self._ns_cell_2g,
                                                               self._networking_api,
                                                               self._logger,
                                                               self._registration_timeout,
                                                               flightmode_cycle=True,
                                                               blocking=False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid)

        # Check Data Connection State => PDP Active before timeout
        self._ns_data_2g.check_data_connection_state("PDP_ACTIVE",
                                                     self._registration_timeout,
                                                     blocking=False)

        # Get RAT from Equipment
        network_type = self._ns_data_2g.get_network_type()

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

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        # Clear old SMS on Network simulator
        self._ns_messaging_2g.clear_message_data()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # Disconnect from equipment
        self._ns.release()

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"
