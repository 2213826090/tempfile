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
:summary: This file is the base Use Case for LTE Mobility
:since: 04/06/2013
:author: lvacheyx
"""
from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_BASE import LabMobilityBase
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from ErrorHandling.AcsConfigException import AcsConfigException


class LabMobilityLteBase(LabMobilityBase):

    """
    Usecase base for mobility use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabMobilityBase.__init__(self, tc_name, global_config)

        # Read the xml Template for NS1 Configuration
        # Read NS1_MIMO from test case xml file
        self._ns1_mimo_temp = self._tc_parameters.get_param_value("NS1_MIMO")
        if self._ns1_mimo_temp not in (None, ''):
            self._ns1_mimo = (str(self._ns1_mimo_temp).lower() == "true")
        # Read NS1_SIGNAL_MODE from test case xml file
        self._ns1_signal_mode = \
            str(self._tc_parameters.get_param_value("NS1_SIGNAL_MODE"))

        # Read NS1_PHYSICAL_CELL_ID from test case xml file
        self._ns1_physical_cell_id = \
            str(self._tc_parameters.get_param_value("NS1_PHYSICAL_CELL_ID"))
        # Read NS1_CELL_ID from test case xml file
        self._ns1_cell_id = \
            str(self._tc_parameters.get_param_value("NS1_CELL_ID"))
        # Read NS1_CELL_POWER_RFO1 from test case xml file
        self._ns1_cell_power_rf1 = \
            str(self._tc_parameters.get_param_value("NS1_CELL_POWER_RFO1"))
        # Read NS1_CELL_POWER_RFO2 from test case xml file
        self._ns1_cell_power_rf2 = \
            str(self._tc_parameters.get_param_value("NS1_CELL_POWER_RFO2"))

        # Read NS1 Mobile Country Code (MCC) from test case xml file
        self._ns1_mcc = \
            int(self._tc_parameters.get_param_value("NS1_MCC"))

        # Read NS1 Mobile Network Code (MNC) from test case xml file
        self._ns1_mnc = \
            int(self._tc_parameters.get_param_value("NS1_MNC"))
        # Read NS1_CELL_BAND from test case xml file
        self._ns1_cell_band = \
            int(self._tc_parameters.get_param_value("NS1_CELL_BAND"))
        # Read NS1_DL_EARFCN from test case xml file
        self._ns1_lte_dl_earfcn = \
            int(self._tc_parameters.get_param_value("NS1_DL_EARFCN"))
        # Read NS1_SCENARIO_PATH from test case xml file
        self._ns1_scenario_path = \
            str(self._tc_parameters.get_param_value("NS1_SCENARIO_PATH"))

        # Read the xml Template for NS2 Configuration
        # Read NS2_MIMO from test case xml file
        self._ns2_mimo_temp = self._tc_parameters.get_param_value("NS2_MIMO")
        if self._ns2_mimo_temp not in (None, ''):
            self._ns2_mimo = (str(self._ns2_mimo_temp).lower() == "true")
        # Read NS2_SIGNAL_MODE from test case xml file
        self._ns2_signal_mode = \
            str(self._tc_parameters.get_param_value("NS2_SIGNAL_MODE"))

        # Read NS2_PHYSICAL_CELL_ID from test case xml file
        self._ns2_physical_cell_id = \
            str(self._tc_parameters.get_param_value("NS2_PHYSICAL_CELL_ID"))
        # Read NS2_CELL_ID from test case xml file
        self._ns2_cell_id = \
            str(self._tc_parameters.get_param_value("NS2_CELL_ID"))
        # Read NS2_CELL_POWER_RFO1 from test case xml file
        self._ns2_cell_power_rf1 = \
            str(self._tc_parameters.get_param_value("NS2_CELL_POWER_RFO1"))
        # Read NS2_CELL_POWER_RFO2 from test case xml file
        self._ns2_cell_power_rf2 = \
            str(self._tc_parameters.get_param_value("NS2_CELL_POWER_RFO2"))

        # Read NS2 Mobile Country Code (MCC) from test case xml file
        self._ns2_mcc = \
            int(self._tc_parameters.get_param_value("NS2_MCC"))

        # Read NS2 Mobile Network Code (MNC) from test case xml file
        self._ns2_mnc = \
            int(self._tc_parameters.get_param_value("NS2_MNC"))
        # Read NS2_CELL_BAND from test case xml file
        self._ns2_cell_band = \
            int(self._tc_parameters.get_param_value("NS2_CELL_BAND"))
        # Read NS2_DL_EARFCN from test case xml file
        self._ns2_lte_dl_earfcn = \
            int(self._tc_parameters.get_param_value("NS2_DL_EARFCN"))
        # Read NS2_SCENARIO_PATH from test case xml file
        self._ns2_scenario_path = \
            str(self._tc_parameters.get_param_value("NS2_SCENARIO_PATH"))

        if self._ns1_model == "AGILENT_E6621A" and self._ns2_model == "AGILENT_E6621A":
            self._logger.debug("Bench configuration is supported by the Usecase using %s as NS1 \
                                and %s as NS2" % self._ns1_model, self._ns2_model)

        else:
            # Set NS1 APIs instance
            self._ns1_cell = self._ns1.get_cell_4g()
            self._ns1_data = self._ns1_cell.get_data()

            # Set the equipment application format depending on ACTIVE_CELL_TECH.
            # For 4G switch to ""LTE FDD""
            self._ns1.switch_app_format("LTE FDD")

            # Set NS2 APIs instance
            self._ns2_cell = self._ns2.get_cell_4g()
            self._ns2_data = self._ns2_cell.get_data()

            # Set the equipment application format depending on IDLE_CELL_TECH.
            # For 4G switch to ""LTE FDD""
            self._ns2.switch_app_format("LTE FDD")

            self._error.Msg = "Bench configuration is not supported by the Usecase"
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, self._error.Msg)

#-----------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobilityBase Setup function
        LabMobilityBase.set_up(self)

        # Set cell off
        self._ns1_cell.set_cell_off()

        # Set cell off
        self._ns2_cell.set_cell_off()

        # Set EPC off
        self._ns1_data.set_epc_off()
        self._ns2_data.set_epc_off()

        # Call specific configuration functions
        RegUtil.setup_cell_lte(self._ns1,
                             self._ns1_mcc,
                             self._ns1_mnc,
                             self._ns1_ip_dut,
                             self._ns1_signal_mode,
                             self._ns1_cell_id,
                             self._ns1_physical_cell_id,
                             self._ns1_mimo,
                             self._ns1_cell_power_rf1,
                             self._ns1_cell_power_rf2,
                             self._ns1_scenario_path,
                             self._ns1_cell_band,
                             self._ns1_lte_dl_earfcn,
                             self._apn)

        # Call specific configuration functions
        RegUtil.setup_cell_lte(self._ns2,
                             self._ns2_mcc,
                             self._ns2_mnc,
                             self._ns2_ip_dut,
                             self._ns2_signal_mode,
                             self._ns2_cell_id,
                             self._ns2_physical_cell_id,
                             self._ns2_mimo,
                             self._ns2_cell_power_rf1,
                             self._ns2_cell_power_rf2,
                             self._ns2_scenario_path,
                             self._ns2_cell_band,
                             self._ns2_lte_dl_earfcn,
                             self._apn)

        # Set EPC on
        self._ns1_data.set_epc_on()
        self._ns2_data.set_epc_on()

        # Set the APN
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        return Global.SUCCESS, "No errors"
