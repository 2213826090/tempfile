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
:summary: This file is the base Use Case for 3GSM Mobility
:since: 04/06/2013
:author: lvacheyx
"""
import time
from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_BASE import LabMobilityBase
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from ErrorHandling.AcsConfigException import AcsConfigException


class LabMobility3gsmBase(LabMobilityBase):

    """
    Usecase base for mobility use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Added Messaging management
        LabMobilityBase.__init__(self, tc_name, global_config)

        # Read NS1_CELL_TECH from test case xml file (In str)
        self._ns1_cell_tech = \
            str(self._tc_parameters.get_param_value("NS1_CELL_TECH"))

        # Read NS2_CELL_TECH from test case xml file (In str)
        self._ns2_cell_tech = \
            str(self._tc_parameters.get_param_value("NS2_CELL_TECH"))

        # Read NS1_CELL_BAND from test case xml file (In str)
        self._ns1_cell_band = str(self._tc_parameters.get_param_value("NS1_CELL_BAND"))

        # Read NS2_CELL_BAND from test case xml file (In str)
        self._ns2_cell_band = str(self._tc_parameters.get_param_value("NS2_CELL_BAND"))

        # NS1_CELL_REL
        self._ns1_cell_rel = 7

        # NS2_CELL_REL
        self._ns2_cell_rel = 7

        # Set APIs instances for NS1
        if self._ns1_cell_tech == "2G":
            self._ns1_cell = self._ns1.get_cell_2g()
            self._ns1_vc = self._ns1_cell.get_voice_call()
            self._ns1_data = self._ns1_cell.get_data()
            self._ns1_messaging = self._ns1_cell.get_messaging()
        elif self._ns1_cell_tech == "3G":
            self._ns1_cell = self._ns1.get_cell_3g()
            self._ns1_vc = self._ns1_cell.get_voice_call()
            self._ns1_data = self._ns1_cell.get_data()
            self._ns1_messaging = self._ns1_cell.get_messaging()
        else:
            self._error.Msg = "Unknown Cell Radio Access Technology"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        # Set APIs instances for NS2
        if self._ns2_cell_tech == "2G":
            self._ns2_cell = self._ns2.get_cell_2g()
            self._ns2_vc = self._ns2_cell.get_voice_call()
            self._ns2_data = self._ns2_cell.get_data()
            self._ns2_messaging = self._ns2_cell.get_messaging()
        elif self._ns2_cell_tech == "3G":
            self._ns2_cell = self._ns2.get_cell_3g()
            self._ns2_vc = self._ns2_cell.get_voice_call()
            self._ns2_data = self._ns2_cell.get_data()
            self._ns2_messaging = self._ns2_cell.get_messaging()
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown Cell Radio Access Technology")

        # Read CATEGORY from testcase xml parameters
        self._category = \
            str(self._tc_parameters.get_param_value("CATEGORY", "WCDMA"))
        # Read fast dormancy parameter
        self._ns1_fast_dormancy = \
            self._ns1_node.get_param_value("Fast_Dormancy", "disable")
        self._ns2_fast_dormancy = \
            self._ns2_node.get_param_value("Fast_Dormancy", "disable")

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobilityBase Setup function
        LabMobilityBase.set_up(self)

        if self._ns1_cell_tech == "2G":
            # Set the equipment application format depending on NS1_CELL_TECH.
            # If 2G switch to ""GSM/GPRS""
            self._ns1.switch_app_format("GSM/GPRS")
        elif self._ns1_cell_tech == "3G":
            # Set the equipment application format depending on NS1_CELL_TECH.
            # If 3G switch to ""WCDMA""
            self._ns1.switch_app_format("WCDMA")

        if self._ns2_cell_tech == "2G":
            # Set the equipment application format depending on NS2_CELL_TECH.
            # If 2G switch to ""GSM/GPRS""
            self._ns2.switch_app_format("GSM/GPRS")
        elif self._ns2_cell_tech == "3G":
            # Set the equipment application format depending on NS2_CELL_TECH.
            # If 3G switch to ""WCDMA""
            self._ns2.switch_app_format("WCDMA")

        # Ensure flight mode off so that GSM sim operator info can be retrieved
        self._networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        if self._category != "GSM":
            # Read maxDlWcdmaRab, maxUlWcdmaRab, maxDlHspaRab, maxUlHspaRab from DeviceCatalog.xml
            if self._category == "WCDMA":
                max_dl_rab = str(self._dut_config.get("maxDlWcdmaRab"))
                max_ul_rab = str(self._dut_config.get("maxUlWcdmaRab"))
            elif self._category == "HSPA":
                max_dl_rab = str(self._dut_config.get("maxDlHspaRab"))
                max_ul_rab = str(self._dut_config.get("maxUlHspaRab"))

            # Read the UL_RAB value from UseCase xml Parameter
            # Check if this value is set to MAX or not
            ul_rab_config = self._tc_parameters.get_param_value("UL_RAB", "MAX")

            # Read the DL_RAB value from UseCase xml Parameter
            # Check if this value is set to MAX or not
            dl_rab_config = self._tc_parameters.get_param_value("DL_RAB", "MAX")

            if ul_rab_config == "MAX":
                self._ul_rab = max_ul_rab
            else:
                self._ul_rab = ul_rab_config

            if dl_rab_config == "MAX":
                self._dl_rab = max_dl_rab
            else:
                self._dl_rab = dl_rab_config

            if self._category == "HSPA":
                # Get the HSDPA category (only if user
                # wants to use HSDPA.
                if(self._dl_rab is not None) and self._category:
                    # Retrieve all after "HSDPA_CAT"
                    self._hsdpa_cat = self._dl_rab[9:]
                else:
                    self._logger.debug("HSDPA Cat parameter not found "
                                       "on TC parameters, retrieve from DeviceModel")
                    self._hsdpa_cat = \
                        int(self._dut_config.get("maxDlHspaRab"))

                self._logger.debug("HSDPA CAT: " + str(self._hsdpa_cat))
                # Get the HSUPA category (only if user
                # wants to use HSUPA.
                if self._ul_rab.find("HSUPA_CAT") != -1:
                    # Retrieve all after "HSUPA_CAT"
                    self._hsupa_cat = self._ul_rab[9:]
                    self._logger.debug("HSUPA CAT: " + str(self._hsupa_cat))
                else:
                    self._hsupa_cat = None

                # Initialize further parameters
                self._rbt_channel_type = "HSPA"

        # Disable data and PDP context
        self._networking_api.deactivate_pdp_context()

        # Enable flight mode
        self._networking_api.set_flight_mode("on")

        # Perform full preset on NS1 cell
        self._ns1.perform_full_preset()

        # Perform full preset on NS2 cell
        self._ns2.perform_full_preset()

        # Set cell off for the 2 network simulators
        self._ns1_cell.set_cell_off()
        self._ns2_cell.set_cell_off()

        # Set IP addresses for the 2 network simulators
        self._ns1.set_ip_addresses(self._ns1_ip_lan1,
                                   self._ns1_ip_lan2,
                                   self._ns1_ip_subnet_mask,
                                   self._ns1_ip_default_gateway,
                                   self._ns1_ip_dut,
                                   self._ns1_ip_dns1,
                                   self._ns1_ip_dns2)

        self._ns2.set_ip_addresses(self._ns2_ip_lan1,
                                   self._ns2_ip_lan2,
                                   self._ns2_ip_subnet_mask,
                                   self._ns2_ip_default_gateway,
                                   self._ns2_ip_dut,
                                   self._ns2_ip_dns1,
                                   self._ns2_ip_dns2)

        self.set_external_connection(self._ns1,
                                     self._ns2,
                                     self._ns1_ip_lan2,
                                     self._ns2_ip_lan2)

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns1.configure_amplitude_offset_table()

        # Set Frequency points using frequency list
        # Set Amplitude Offset correction using offset list
        # Turning amplitude offset state to ON
        self._ns2.configure_amplitude_offset_table()

        # If the fast dormancy parameter is set to disable do nothing to
        # avoid regressions.
        if self._ns1_fast_dormancy.lower() == "enable" and (self._ns1_cell_tech == "3G"):
            # Setting fast dormancy support according to the parameter
            # present in the bench config.
            self._ns1_data.\
                set_fast_dormancy_support("enable")
        if self._ns2_fast_dormancy.lower() == "enable" and (self._ns2_cell_tech == "3G"):
            # Setting fast dormancy support according to the parameter
            # present in the bench config.
            self._ns2_data.\
                set_fast_dormancy_support("enable")

        # Call specific configuration functions
        RegUtil.setup_cell(1,
                           self._ns1_model,
                           self._ns1_cell_tech,
                           self._ns1_cell_band,
                           self._ns1_cell_rel,
                           self._logger)
        RegUtil.setup_cell(2,
                           self._ns2_model,
                           self._ns2_cell_tech,
                           self._ns2_cell_band,
                           self._ns2_cell_rel,
                           self._logger)

        # Deactivate E-DCH Cell and HSDPA Cell capabilities
        # if cell tech is 3G
        if self._ns1_cell_tech == "3G":
            self._ns1_data.set_edch_cell_capability("OFF")
            self._ns1_data.set_hsdpa_cell_capability("OFF")
        # if cell tech is 2G
        else:
            # Set the multislot configuration
            self._ns1_data.set_multislot_config(self._multislot)

        if self._ns2_cell_tech == "3G":
            self._ns2_data.set_edch_cell_capability("OFF")
            self._ns2_data.set_hsdpa_cell_capability("OFF")
        # if cell tech is 2G
        else:
            # Set the multislot configuration
            self._ns2_data.set_multislot_config(self._multislot)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        try:
            # Call LabMobilityBase tear_down function
            LabMobilityBase.tear_down(self)

            # Clear all data connections
            self._networking_api.deactivate_pdp_context()
            time.sleep(self._wait_btwn_cmd)

            # Activate the flight mode.
            self._networking_api.set_flight_mode("on")
        except:
            pass

        # Set cell off
        self._ns1_cell.set_cell_off()

        # Set cell off
        self._ns2_cell.set_cell_off()

        # Disconnect from external 8960
        self._ns1.disconnect_from_external_device()

        # Disconnect from external 8960
        self._ns2.disconnect_from_external_device()

        # Verify ethernet disconnection
        self._ns2.check_external_device_disconnection(1, 5)

        # Release equipment NS1 resource
        self._ns1.release()

        # Release equipment NS2 resource
        self._ns2.release()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def _set_gprs_bearer(self, cell, data, eqt_id=1):
        """
        Set GPRS radio bearer for 3g cell
        :type cell: str
        :param cell: Network Simulator Cell API
        :type data: str
        :param data: Network Simulator Data API
        :type eqt_id: int
        :param eqt_id: Equipment number
        """

        # Set the initial PS Data RRC State to DCH
        data.set_initial_ps_data_rrc_state("DCH")

        if self._category == "HSPA":
            # Activate E-DCH and HSDPA cell capabilities
            data.set_edch_cell_capability("ON")
            data.set_hsdpa_cell_capability("ON")
            # Get the HSUPA MS reported category (only if user
            # wants to use HSUPA.
            if (self._ul_rab.find("HSUPA_CAT") != -1):
                data.set_gprs_radio_access_bearer("HSUPA", "HSDPA")
                self._ul_rab = None
            else:
                data.set_gprs_radio_access_bearer(self._ul_rab, "HSDPA")

            # Set the category configuration to AUTO
            data.set_category_control_mode("ON")

            # Set channel type, the default value is "HSPA"
            cell.set_rbt_channel_type(self._rbt_channel_type)

            # Call specific configuration functions
            self._setup_high_categories(eqt_id)

        else:
            # Sets the GPRS Radio Access Bearer
            data.set_gprs_radio_access_bearer(self._ul_rab, self._dl_rab)
