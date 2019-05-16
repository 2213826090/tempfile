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
:summary: This file is the mobility external handover during voice call
and then, after call release, a cell reselection from 2G to 3G.
:since: 20/09/2012
:author: Lvacheyx
"""
import time
import random
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase


class LabMobilityExtHoVcAndCresel(LabMobility3gsmBase):

    """
    Mobility handover during voice call Usecase
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobility3gsmBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)

        # Initialize variables
        self._ho_ns1_limit_power = None
        self._ho_ns2_limit_power = None
        self._wait_before_voice_call_release = None
        self._wanted_reg_state = None

        # Read PHONE_NUMBER from testcase xml parameters
        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Read NS1_CELL_POWER from testcase xml parameters
        self._ns1_cell_power = \
            float(self._tc_parameters.get_param_value("NS1_CELL_POWER"))

        # Read NS1_CELL_SERVICE from testcase xml parameters
        self._ns1_cell_service = \
            str(self._tc_parameters.get_param_value("NS1_CELL_SERVICE"))

        # Read NS2_CELL_POWER from testcase xml parameters
        self._ns2_cell_power = \
            float(self._tc_parameters.get_param_value("NS2_CELL_POWER"))

        # Read CRESEL_LIMIT_POWER from testcase xml parameters
        self._cresel_limit_power = \
            float(self._tc_parameters.get_param_value("CRESEL_LIMIT_POWER"))

        # Read DECREMENTATION_STEP_POWER from testcase xml parameters
        self._decrementation_step_power = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_POWER"))

        # Set INCREMENTATION_STEP_POWER
        self._incrementation_step_power = \
            float(self._tc_parameters.get_param_value("INCREMENTATION_STEP_POWER"))

        # Read DECREMENTATION_STEP_TIMER from testcase xml parameters
        self._decrementation_step_timer = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_TIMER"))

        # Sets INCREMENTATION_STEP_TIMER
        self._incrementation_step_timer = \
            float(self._tc_parameters.get_param_value("INCREMENTATION_STEP_TIMER"))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobility3gsmBase Set_up function
        LabMobility3gsmBase.set_up(self)

        # Get the NS1 cell LAC in order
        # to manage possible INTRA_LA test case
        ns1_lac = self._ns1_cell.get_lac()

        # Get the NS2 cell LAC in order
        # to manage possible INTRA_LA test case
        ns2_lac = self._ns2_cell.get_lac()

        # IF NS1 and NS2 cells LAC are equals,
        # don't call the random lac to stay in INTRA_LA mode
        if ns1_lac != ns2_lac:
            # Sets a random LAC excluding NS2 and NS1 LAC
            self._ns1_cell.set_random_lac([ns1_lac, ns2_lac])

        # Set cell on the NS1 cell
        self._ns1_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

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
                                                               flightmode_cycle=True,
                                                               blocking=False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._wanted_reg_state = self._modem_api.get_network_registration_status()

        self._modem_api.check_cdk_state_bfor_timeout(self._wanted_reg_state,
                                                     self._registration_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        # Set default variables to witch equipments during several handovers
        ns1_cell = self._ns1_cell
        ns1_vc = self._ns1_vc
        ns1_data = self._ns1_data
        ns2_cell = self._ns2_cell
        ns2_vc = self._ns2_vc
        ns2_data = self._ns2_data

        # Get RAT from Equipment
        network_type = ns1_data.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Release any previous call (Robustness)
        self._voicecall_api.release()

        # Perform MO voice call on NS1
        self._voicecall_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        ns1_vc.check_call_connected(self._call_setup_time,
                                    blocking=False)

        # Wait for Voice Call to stay active before HandOver
        time.sleep(5)

        # Simulate that the DUT is moving across cells for Handover
        # Increase NS2 cell power and decrease NS1 cell power

        # Set the ns1_cell_power to NS1 cell power
        # for the first decrementation iteration
        ns1_cell_power = float(-70)
        #  Sets HO_NS1_LIMIT_POWER
        self._ho_ns1_limit_power = float(-90)

        # Set the ns2_cell_power to ns2 cell power
        # for the first incrementation iteration
        ns2_cell_power = float(-110)
        #  Sets HO_NS2_LIMIT_POWER
        self._ho_ns2_limit_power = float(-70)

        # Set cell the NS2 Cell
        self._ns2_cell.set_cell_on()

        # Log the current Cell Reselection iteration
        self._logger.info("Preparing Handover")

        self.decrease_and_increase_cell_power_while_vc(ns1_cell,
                                                       ns1_vc,
                                                       ns1_cell_power,
                                                       ns2_cell,
                                                       ns2_data,
                                                       ns2_cell_power,
                                                       self._decrementation_step_power,
                                                       self._decrementation_step_timer,
                                                       self._ho_ns1_limit_power,
                                                       self._incrementation_step_power,
                                                       self._incrementation_step_timer,
                                                       self._ho_ns2_limit_power)

        # Log the current iteration
        self._logger.info("Performing Handover")

        # Perform handover using 10 seconds timeout
        ns1_cell.execute_external_handover()

        # Wait for HandOver to be done with Voice Call still active
        time.sleep(10)

        # Check call state "CONNECTED" before 5 seconds to validate handover
        ns2_vc.check_call_connected(5, blocking=False)

        # Set the ns1_cell_power (3G cell) to -115 dBm
        # to be sure that DUT stays registered on NS1 2G cell
        ns1_cell_power = float(-115)
        ns1_cell.set_cell_power(ns1_cell_power)

        # Wait 2 or 30 seconds before voice call release
        self._wait_before_voice_call_release = random.randrange(2, 40, 28)
        self._logger.info("Wait %s seconds before voice call release",
                          self._wait_before_voice_call_release)
        time.sleep(self._wait_before_voice_call_release)

        # Release the voice call
        ns2_vc.voice_call_network_release()

        # Get RAT from Equipment
        network_type = ns2_data.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # END OF HANDOVER

        # Wait 15 seconds after HandOver before performing Reselection
        self._logger.info("Wait 15 seconds before performing cell-reselection")
        time.sleep(15)

        # Log the current iteration
        self._logger.info("Performing cell reselection")

        # Set NS1 cell power to NS1_CELL_POWER
        ns1_cell.set_cell_power(self._ns1_cell_power)

        # Set NS2 cell power to NS2_CELL_POWER
        ns2_cell.set_cell_power(self._ns2_cell_power)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._wanted_reg_state = self._modem_api.get_network_registration_status()

        self._modem_api.check_cdk_state_bfor_timeout(self._wanted_reg_state,
                                                     self._registration_timeout)

        msg = "Begin to decrease Camped cell power from %.2f dBm "
        msg += "to %.2f dBm each %d seconds by step of %.2f dBm while "
        msg += "cell reselection isn't performed."
        self._logger.info(
            msg,
            self._ns2_cell_power,
            self._cresel_limit_power,
            self._decrementation_step_timer,
            self._decrementation_step_power)

        # Decrease cell power on NS2 and wait for DUT to be camped on NS1
        self.decrease_cell_power_while_idle(self._ns2_cell,
                                            self._ns1_cell_power,
                                            self._ns1_cell,
                                            self._ns1_data,
                                            self._ns1_cell_service,
                                            self._decrementation_step_power,
                                            self._decrementation_step_timer,
                                            self._cresel_limit_power,
                                            self._ns2_cell_power,
                                            self._ns1_model)

        # Return message and quit the method
        return Global.SUCCESS, "No errors"
