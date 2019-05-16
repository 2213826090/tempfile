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
:since: 10/02/2011
:author: jre
"""

import time
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.EquipmentUtilities import get_nw_sim_bench_name


class LabWcdmaSmsPsBase(UseCaseBase):

    """
    Use Case WCDMA SMS over PS base class.
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
        self._band = self._tc_parameters.get_param_value("CELL_BAND")

        # NS_CELL_REL
        self._ns_cell_rel = 7

        # Read CELL_POWER from xml UseCase parameter file
        self._cell_power = \
            int(self._tc_parameters.get_param_value("CELL_POWER"))

        # Read DL_UARFCN from xml UseCase parameter file
        self._dl_uarfcn = \
            int(self._tc_parameters.get_param_value("DL_UARFCN"))

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

        # Create cellular network simulator and retrieve 3G mesaging
        # and 3G data interfaces
        self._ns = self._em.get_cellular_network_simulator(self._bench_name)
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_messaging_3g = self._ns_cell_3g.get_messaging()
        self._ns_data_3g = self._ns_cell_3g.get_data()

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

        # Set the equipment Application Format = "WCDMA"
        self._ns.switch_app_format("WCDMA")

        # Perform full preset
        self._ns.perform_full_preset()

        # Deactivate HSUPA and HSDPA capabilities
        self._ns_data_3g.set_edch_cell_capability("OFF")
        self._ns_data_3g.set_hsdpa_cell_capability("OFF")

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # 3G default cell setup
        RegUtil.setup_cell(self._ns_number,
                           self._ns_model,
                           "3G",
                           self._band,
                           self._ns_cell_rel,
                           self._logger)

        # Set Frequency points using frequency list
        self._ns.configure_amplitude_offset_table()

        # Set cell power using CELL_POWER value
        self._ns_cell_3g.set_cell_power(self._cell_power)

        # Set serving cell to PACKET
        self._ns_cell_3g.set_cell_service("PACKET")

        # Set downlink UARFCN using DL_UARFCN value
        self._ns_cell_3g.set_band_and_dl_arfcn(
            "BAND" + str(self._band), self._dl_uarfcn)

        # Set cell on
        self._ns_cell_3g.set_cell_on()

        # Check Data Connection State => ATTACHED before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("ATTACHED",
                                                               self._ns_cell_3g,
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

        # Set cell off
        self._ns_cell_3g.set_cell_off()

        # Disconnect from equipment
        self._ns.release()

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"
