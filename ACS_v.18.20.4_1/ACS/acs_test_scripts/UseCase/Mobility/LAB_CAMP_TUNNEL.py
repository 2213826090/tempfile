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
:summary: Several cell tunneling 2G/2G UC, back from out of coverage
:since: 04/02/2013
:author: sjamaoui
"""
from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from UtilitiesFWK.Utilities import Global
import time


class LabCampTunnel(LabMobility3gsmBase):

    """
    Lab Camp Tunnel testing cell changes
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobilityBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)
        # activeize variables
        self._wanted_reg_state = None

        # Read NS1 parameters from testcase xml parameters
        self._ns1_cell_power = \
            float(self._tc_parameters.get_param_value("NS1_CELL_POWER"))

        self._ns1_cell_service = \
            str(self._tc_parameters.get_param_value("NS1_CELL_SERVICE"))

        self._ns1_limit_power = \
            float(self._tc_parameters.get_param_value("NS1_LIMIT_POWER"))

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

        self._ns2_limit_power = \
            float(self._tc_parameters.get_param_value("NS2_LIMIT_POWER"))

        self._ns2_arfcn = \
            int(self._tc_parameters.get_param_value("NS2_ARFCN"))

        self._ns2_lac = \
            int(self._tc_parameters.get_param_value("NS2_LAC"))

        self._ns2_rac = \
            int(self._tc_parameters.get_param_value("NS2_RAC"))

        # Read DECREMENTATION_STEP_POWER from testcase xml parameters
        self._decrementation_step_power = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_POWER"))

        # Read DECREMENTATION_STEP_TIMER from testcase xml parameters
        self._decrementation_step_timer = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_TIMER"))

        # Set INCREMENTATION_STEP_POWER
        self._incrementation_step_power = \
            float(self._tc_parameters.get_param_value("INCREMENTATION_STEP_POWER"))

        # Sets INCREMENTATION_STEP_TIMER
        self._incrementation_step_timer = \
            float(self._tc_parameters.get_param_value("INCREMENTATION_STEP_TIMER"))

#-----------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobility3gsmBase Setup function
        LabMobility3gsmBase.set_up(self)

        return Global.SUCCESS, "No errors"

#-----------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        # Set default variables to switch equipments
        # during several tunneling

        # Set NS1 Equipment APIs
        ns1_cell = self._ns1_cell
        ns1_data = self._ns1_data
        # Set NS2 Equipment APIs
        ns2_cell = self._ns2_cell
        ns2_data = self._ns2_data

        nb_tunnel_success = 0
        is_tunnel_limit_power_reached = False
        is_dut_registered = False

        # Set cells off
        ns2_cell.set_cell_off()
        ns1_cell.set_cell_off()

        # Set Cell Band  and ARFCN using NS1_CELL_BAND
        # and NS1_ARFCN parameters
        # Set cell service using NS1_CELL_SERVICE parameter
        # Set Cell Power using NS1_CELL_POWER parameter
        self._ns1_cell.configure_basic_cell_parameters(
            self._ns1_cell_service, self._ns1_cell_band,
            self._ns1_arfcn, self._ns1_cell_power)

        # Set Cell Band  and ARFCN using NS2_CELL_BAND
        # and NS2_ARFCN parameters
        # Set cell service using NS2_CELL_SERVICE parameter
        # Set Cell Power using NS2_CELL_POWER parameter
        self._ns2_cell.configure_basic_cell_parameters(
            self._ns2_cell_service, self._ns2_cell_band,
            self._ns2_arfcn, self._ns2_cell_power)

        # Set NS1_LAC and NS2_LAC parameters
        ns1_cell.set_lac(self._ns1_lac)
        ns2_cell.set_lac(self._ns2_lac)

        # Set NS1_RAC and NS2_RAC parameters
        ns1_cell.set_rac(self._ns1_rac)
        ns2_cell.set_rac(self._ns2_rac)

        # Set cell on
        ns1_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns1_data.get_network_type(),
                                                          self._registration_timeout)

        # Set cell on
        ns2_cell.set_cell_on()

        self._jump_num = 2

        # Perform try catch in order to catch errors during the for iteration.
        # If this try catch isn't done here and a crash appears during the for
        # iteration, the Tear Down will be called without recording Use Case
        # verdict and maybe 1 or more succeeded handovers.
        # FOR iteration 0 to jump_num:
        while nb_tunnel_success != self._jump_num:

            # Log the current iteration
            self._logger.info(
                "Performing tunneling number %d of %d.",
                nb_tunnel_success + 1, self._jump_num)

            # Set the cell_power to NS1 cell power
            # for the first decrementation iteration
            ns1_cell_power = self._ns1_cell_power

            # Set the cell_power to NS2 cell power
            # for the first incrementation iteration
            ns2_cell_power = self._ns2_cell_power

            # DUT is registered on NS1 cell
            is_dut_registered = True

            # Set the limit reached flag to false
            is_tunnel_limit_power_reached = False

            msg = "Begin to decrease NS1 cell power from %.2f dBm "
            msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
            msg += "tunneling isn't performed."
            self._logger.info(
                msg,
                ns1_cell_power,
                self._ns1_limit_power,
                self._decrementation_step_timer,
                self._decrementation_step_power)

            # WHILE DUT is registered to the active
            # cell AND cell_power <= self._ns1_limit_power
            while is_dut_registered \
                and (is_tunnel_limit_power_reached == False):

                # Wait during DECREMENTATION_STEP_TIMER
                time.sleep(self._decrementation_step_timer)

                # Decrement active cell power
                ns1_cell_power -= self._decrementation_step_power
                ns1_cell.set_cell_power(ns1_cell_power)

                # DUT is registered
                is_dut_registered = True

                # IF cell_power < self._tunnel_limit_power
                if (ns1_cell_power - self._decrementation_step_power) < \
                        self._ns1_limit_power:
                    # Set the limit reached flag to true
                    is_tunnel_limit_power_reached = True

            # Let 60 seconds to be sure that we are out of coverage (to reach "No service")
            self._logger.info("Wait 60 seconds for ensure that we are out of coverage")
            time.sleep(60)

            # Check registration state is unregistered before 60 seconds
            self._logger.info("Let 60 seconds for DUT to enter in No-Service State (out of coverage)")
            self._modem_api.check_cdk_no_registration_bfor_timeout(60)

            # DUT is no more registered on active cell
            is_dut_registered = False

            # Set the limit reached flag to false
            is_tunnel_limit_power_reached = False

            msg = "Begin to increase NS2 cell power from %.2f dBm "
            msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
            msg += "tunneling isn't performed."
            self._logger.info(
                msg,
                ns2_cell_power,
                self._ns2_limit_power,
                self._incrementation_step_timer,
                self._incrementation_step_power)

            # while INCREMENTATION NS2 CELL POWER....
            while(is_dut_registered == False) \
                    and (is_tunnel_limit_power_reached == False):

                # Wait during INCREMENTATION_STEP_TIMER
                time.sleep(self._incrementation_step_timer)

                # Decrement NS2 cell power
                ns2_cell_power += self._incrementation_step_power
                ns2_cell.set_cell_power(ns2_cell_power)

                # DUT is not registered
                is_dut_registered = False

                # IF cell_power < self._tunnel_limit_power
                if (ns2_cell_power + self._incrementation_step_power) > \
                        self._ns2_limit_power:
                    # Set the limit reached flag to true
                    is_tunnel_limit_power_reached = True

            # Check Data Connection State => ATTACHED before timeout
            ns2_data.check_data_connection_state("ATTACHED",
                                                 self._registration_timeout,
                                                 blocking=False)

            # DUT is registered on NS2 cell
            is_dut_registered = True

            # Check that DUT is registered on the good RAT
            self._modem_api.check_network_type_before_timeout(ns2_data.get_network_type(),
                                                              self._registration_timeout)

            # Increment number of tunneling success number
            nb_tunnel_success += 1

            # Log the tunneling success
            self._logger.info("Tunneling succeeded")

            # Switch NS1 and NS2 network simulators
            tmp_cell = ns1_cell
            tmp_data = ns1_data
            ns1_cell = ns2_cell
            ns1_data = ns2_data
            ns2_cell = tmp_cell
            ns2_data = tmp_data

            # Log the tunneling success number
            self._logger.info(
                "Tunneling number %d success.",
                nb_tunnel_success)

            # Reset flags to their default values
            is_tunnel_limit_power_reached = False
            is_dut_registered = False

        # END FOR
        # Log a warning in case the expected number
            # of tunneling is reached
            if nb_tunnel_success != self._jump_num:
                msg = "(%d succeeded tunneling on %d)." \
                    % (nb_tunnel_success, self._jump_num)

        # Compute final verdict
        msg = "%d tunneling(s) done." % nb_tunnel_success
        return Global.SUCCESS, msg
