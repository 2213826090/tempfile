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
:since: 12/09/2011
:author: ccontreras
"""

import time
from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from UtilitiesFWK.Utilities import Global


class LabMobilityCreselIdle(LabMobility3gsmBase):

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

        self._ns1_arfcn = \
            int(self._tc_parameters.get_param_value("NS1_ARFCN"))

        self._ns1_lac = \
            int(self._tc_parameters.get_param_value("NS1_LAC"))

        self._ns1_rac = \
            int(self._tc_parameters.get_param_value("NS1_RAC"))

        # Read NS2 parameters from testcase xml parameters
        self._ns2_cell_power = \
            float(self._tc_parameters.get_param_value("NS2_CELL_POWER"))

        self._ns2_cell_service = \
            str(self._tc_parameters.get_param_value("NS2_CELL_SERVICE"))

        self._ns2_arfcn = \
            int(self._tc_parameters.get_param_value("NS2_ARFCN"))

        self._ns2_lac = \
            int(self._tc_parameters.get_param_value("NS2_LAC"))

        self._ns2_rac = \
            int(self._tc_parameters.get_param_value("NS2_RAC"))

        self._cresel_power = \
            float(self._tc_parameters.get_param_value("CRESEL_POWER"))

        # Read DECREMENTATION_STEP_POWER from testcase xml parameters
        self._decrementation_step_power = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_POWER"))

        # Read DECREMENTATION_STEP_TIMER from testcase xml parameters
        self._decrementation_step_timer = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_TIMER"))

        # Read CRESEL_LIMIT_POWER from testcase xml parameters
        self._cresel_limit_power = \
            float(self._tc_parameters.get_param_value("CRESEL_LIMIT_POWER"))

#-----------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobility3gsmBase Setup function
        LabMobility3gsmBase.set_up(self)

        # Set default variables to switch equipments
        # during several cell reselections
        # Set NS1 Equipment APIs
        ns1_cell = self._ns1_cell

        # Set NS2 Equipment APIs
        ns2_cell = self._ns2_cell

        # Set Cell Band  and ARFCN using ns1_CELL_BAND
        # and ns1_ARFCN parameters
        # Set cell service using ns1_CELL_SERVICE parameter
        # Set Cell Power using ns1_CELL_POWER parameter
        ns1_cell.configure_basic_cell_parameters(
            self._ns1_cell_service, self._ns1_cell_band,
            self._ns1_arfcn, self._ns_camped_power)

        # Set Cell Band  and ARFCN using TARGET_CELL_BAND
        # and TARGET_ARFCN parameters
        # Set cell service using TARGET_CELL_SERVICE parameter
        # Set Cell Power using TARGET_CELL_POWER parameter
        ns2_cell.configure_basic_cell_parameters(
            self._ns2_cell_service, self._ns2_cell_band,
            self._ns2_arfcn, self._ns2_cell_power)

        # Set ns1_LAC and ns2_LAC parameters
        ns1_cell.set_lac(self._ns1_lac)
        ns2_cell.set_lac(self._ns2_lac)

        # Set ns1_RAC and ns2_RAC parameters
        ns1_cell.set_rac(self._ns1_rac)
        ns2_cell.set_rac(self._ns2_rac)

        # Set cell on
        ns1_cell.set_cell_on()
        ns2_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Check Data Connection State => ATTACHED before timeout
        self._ns1_data.check_data_connection_state("ATTACHED",
                                           self._registration_timeout,
                                           blocking=False)

        return Global.SUCCESS, "No errors"

#-----------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        msg = "Begin to decrease Camped cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
        msg += "cell reselection isn't performed."
        self._logger.info(
            msg,
            self._ns_camped_power,
            self._cresel_limit_power,
            self._decrementation_step_timer,
            self._decrementation_step_power)

        # Decrease cell power on NS1 and wait for DUT to be camped on NS2
        # and log the result in msg
        self.decrease_cell_power_while_idle(self._ns1_cell,
                                            self._ns_camped_power,
                                            self._ns2_cell,
                                            self._ns2_data,
                                            self._ns2_cell_service,
                                            self._decrementation_step_power,
                                            self._decrementation_step_timer,
                                            self._cresel_limit_power,
                                            self._cresel_power,
                                            self._ns2_model)

        # Display Cell reselection result
        self._logger.info("Cell Reselection is successful")

        msg = "Begin to decrease Camped cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
        msg += "cell reselection isn't performed."
        self._logger.info(
            msg,
            self._ns_camped_power,
            self._cresel_limit_power,
            self._decrementation_step_timer,
            self._decrementation_step_power)

        # Decrease cell power on NS2 and wait for DUT to be camped on NS1
        # and log the result in msg
        self.decrease_cell_power_while_idle(self._ns2_cell,
                                            self._ns_camped_power,
                                            self._ns1_cell,
                                            self._ns1_data,
                                            self._ns1_cell_service,
                                            self._decrementation_step_power,
                                            self._decrementation_step_timer,
                                            self._cresel_limit_power,
                                            self._cresel_power,
                                            self._ns1_model)

        # Display Cell reselection result
        self._logger.info("Cell Reselection is successful")

        return Global.SUCCESS, "No errors"
