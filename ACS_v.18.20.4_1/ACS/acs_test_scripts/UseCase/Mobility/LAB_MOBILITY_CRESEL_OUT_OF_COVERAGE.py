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
:summary: Several cell reselections 2G/3G UC
:since: 24/10/2013
:author: mbrisbax
"""

import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from UtilitiesFWK.Utilities import Global


class LabMobilityCreselOutOfCoverage(LabMobility3gsmBase):

    """
    Several cell reselections 2G/3G UC
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobilityBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)
        # Initialize variables
        self._wanted_reg_state = None

        # Read NS1 parameters from testcase xml parameters
        self._ns_camped_power = \
            float(self._tc_parameters.get_param_value("NS1_CELL_POWER"))

        self._ns1_cell_service = \
            str(self._tc_parameters.get_param_value("NS1_CELL_SERVICE"))

        self._ns1_lac = \
            int(self._tc_parameters.get_param_value("NS1_LAC"))

        self._ns1_rac = \
            int(self._tc_parameters.get_param_value("NS1_RAC"))

        # Read NS2 parameters from testcase xml parameters
        self._ns2_cell_power = \
            float(self._tc_parameters.get_param_value("NS2_CELL_POWER"))

        self._ns2_cell_service = \
            str(self._tc_parameters.get_param_value("NS2_CELL_SERVICE"))

        self._ns2_lac = \
            int(self._tc_parameters.get_param_value("NS2_LAC"))

        self._ns2_rac = \
            int(self._tc_parameters.get_param_value("NS2_RAC"))

        self._cresel_power = \
            float(self._tc_parameters.get_param_value("CRESEL_POWER"))

        # Read CRESEL_TIME_OUT from testcase xml parameters
        self._cresel_timeout = \
            float(self._tc_parameters.get_param_value("CRESEL_TIME_OUT"))

        # Read CRESEL_TIME_OUT from testcase xml parameters
        self._cresel_timeout = \
            float(self._tc_parameters.get_param_value("CRESEL_TIME_OUT"))

        # Read CRESEL_TIME_OUT from testcase xml parameters
        self._cresel_nocoverage_time = \
            float(self._tc_parameters.get_param_value("CRESEL_NO_COVERAGE_TIME"))

        # Read NS1_PSC from testcase xml parameters
        self._ns1_psc = int(self._tc_parameters.get_param_value("NS1_PSC", "1"))

        # Read NS2_PSC from testcase xml parameters
        self._ns2_psc = int(self._tc_parameters.get_param_value("NS2_PSC", "15"))

        # Set parameters to go from cell 1 to cell 2
        self._1_to_2_cell_parms = (self._ns1_cell,
                                       self._ns2_cell,
                                       self._ns2_data,
                                       self._ns2_cell_service,
                                       self._cresel_power,
                                       self._ns2_model,
                                       self._cresel_timeout,
                                       self._cresel_nocoverage_time)

        # Set parameters to go from cell 2 to cell 1
        self._2_to_1_cell_parms = (self._ns2_cell,
                                   self._ns1_cell,
                                   self._ns1_data,
                                   self._ns1_cell_service,
                                   self._cresel_power,
                                   self._ns1_model,
                                   self._cresel_timeout,
                                   self._cresel_nocoverage_time)
        # Set primary cell to first cell
        self._cell_in_use = 1

#-----------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobility3gsmBase Setup function
        LabMobility3gsmBase.set_up(self)

        # Set default variables to switch equipments
        # during several cell reselections
        # Set cell service using ns1_CELL_SERVICE parameter
        # Set Cell Power using ns1_CELL_POWER parameter
        self._ns1_cell.set_cell_service(self._ns1_cell_service)
        self._ns1_cell.set_cell_power(self._ns_camped_power)

        # Set cell service using TARGET_CELL_SERVICE parameter
        # Set Cell Power using TARGET_CELL_POWER parameter
        self._ns2_cell.set_cell_service(self._ns2_cell_service)
        self._ns2_cell.set_cell_power(self._ns2_cell_power)

        # Set ns1_LAC and ns2_LAC parameters
        self._ns1_cell.set_lac(self._ns1_lac)
        self._ns2_cell.set_lac(self._ns2_lac)

        # Set ns1_RAC and ns2_RAC parameters
        self._ns1_cell.set_rac(self._ns1_rac)
        self._ns2_cell.set_rac(self._ns2_rac)

        # Set scrambling code for 3G cells
        if self._ns1_cell_tech == "3G":
            self._ns1_cell.set_scrambling_code(self._ns1_psc)
        if self._ns2_cell_tech == "3G":
            self._ns2_cell.set_scrambling_code(self._ns2_psc)

        # Set cell on
        self._ns1_cell.set_cell_on()
        self._ns2_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check if DUT is attached to cell %d (DUT check)",
                          self._cell_in_use)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set primary cell to first cell
        self._cell_in_use = 1

        # Check Data Connection State => ATTACHED before timeout
        self._logger.info("Check if DUT is attached to cell %d (NW check)",
                          self._cell_in_use)
        self._ns1_data.check_data_connection_state("ATTACHED",
                                                   self._cresel_timeout,
                                                   blocking=False)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        # Check Data Connection State => PDP_ACTIVE before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("PDP_ACTIVE",
                                                               self._ns1_cell,
                                                               self._networking_api,
                                                               self._logger,
                                                               self._registration_timeout,
                                                               flightmode_cycle=False,
                                                               blocking=False)

        # Check that DUT is registered on the good RAT
        self._logger.info("Check if DUT is attached to cell %d with the good RAT",
                          self._cell_in_use)
        self._modem_api.check_network_type_before_timeout(self._ns1_data.get_network_type(),
                                                          self._registration_timeout)

        return (Global.SUCCESS,
                "No errors")

#-----------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        # Go out of coverage then switch to other cell
        if self._cell_in_use == 1:
            self._cell_in_use = 2
            cell_parms = self._1_to_2_cell_parms
        else:
            self._cell_in_use = 1
            cell_parms = self._2_to_1_cell_parms

        self._logger.info("Go out of coverage then switch to cell %d",
                           self._cell_in_use)
        self.go_out_of_coverage_and_cresel(*cell_parms)

        return Global.SUCCESS, "No errors"
