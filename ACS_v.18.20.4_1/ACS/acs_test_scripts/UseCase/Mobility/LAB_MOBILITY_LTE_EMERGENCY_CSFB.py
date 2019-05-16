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
:summary: This file implements the test of a FTP transfer on LTE followed by an
emergency CSFB and resuming the transfer on LTE.
:since: 10/12/2013
"""

from LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from acs_test_scripts.UseCase.Mobility.LAB_MOBILITY_LTE_CSFB import LabMobilityLteCsfb
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd import UECmdTypes

import time


class LabMobilityLteEmergencyCsfb(LabMobilityLteCsfb):
    """
    Class implementing the test in which you perform an CSFB on a 2G cell while
    making a FTP transfer.
    .. warning:: Only supports CSFB with 2G cells for now.
    .. warning:: Not a functional test for now.
    """

    EMERGENCY_NUMBER_PROPERTY_NAME = "ril.ecclist"
    """
    The name of the property corresponding to the
    list of emergency numbers.
    """
    BOUYGUES_MNC = 20
    BOUYGUES_MCC = 208

    def __init__(self, tc_name, global_config):
        """
        Retrieves all the necessary parameters of the test case.
        """
        LabMobilityLteCsfb.__init__(self, tc_name, global_config)
        self._initial_ue_command_em_number = UECmdTypes.EMERGENCY_NUMBERS_LIST
        self._initial_emergency_numbers = None
        self.release_csfb_vc_type = "NR"
        self._vc_type = "MO"

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """
        if self._target_ping_packet_loss_rate in ("", None):
            self._target_ping_packet_loss_rate = float(0)
        else:
            self._target_ping_packet_loss_rate = float(
                self._target_ping_packet_loss_rate)

        emergency_numbers_string = self._device.get_property_value(
                LabMobilityLteEmergencyCsfb.EMERGENCY_NUMBER_PROPERTY_NAME)
        # Split the str to a list
        emergency_numbers = emergency_numbers_string.split(",")
        # Make sure the provided Phone Number parameter is not present
        # in the list of emergency numbers
        if self._phone_number in emergency_numbers:
            # Otherwise create an error message
            message = "Provided parameter PHONE_NUMBER [%s] as a value " \
                "that corresponds to a real emergency number on the LIVE " \
                "network [list: (%s)]. The test will not be executed." % (
                    self._phone_number,
                    emergency_numbers_string)
            # Raise an exception
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     message)
        # Store the list of emergency numbers so that we can
        # restore it later
        self._initial_emergency_numbers = emergency_numbers_string
        UECmdTypes.EMERGENCY_NUMBERS_LIST = \
            self._initial_ue_command_em_number + (self._phone_number,)

        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        #  Set External EPC connection
        self._ns_3gsm_cell.set_external_epc_connection(
                self._ns_lte_ip_lan1,
                self._ns_lte_dl_earfcn)

        self._ns_3gsm_cell.set_mnc(LabMobilityLteEmergencyCsfb.BOUYGUES_MNC)
        self._ns_3gsm_cell.set_mcc(LabMobilityLteEmergencyCsfb.BOUYGUES_MCC)
        # Setting the IMSI attach reject cause to: PLMN Not Allowed.
        self._ns_3gsm_cell.set_lau_reject_gmm_cause(11)
        self._ns_3gsm_cell.set_lau_reject_state("ON")
        self._ns_3gsm_cell.set_network_mode_of_operation(2)

        # Wait 30 sec
        self._logger.info("Wait 30 seconds before powering on LTE cell to"
                          " ensure DUT is unregistered")

        self._modem_api.check_cdk_no_registration_bfor_timeout(30)

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Executes the test:
        1-
        """
        if self._is_emergency_number:
            # Update the property holding the Emergency Numbers list.
            self._device.set_property_value(
                LabMobilityLteEmergencyCsfb.EMERGENCY_NUMBER_PROPERTY_NAME,
                self._phone_number)
        self._ns_3gsm_cell.set_cell_on()
        self._networking_api.set_flight_mode("off")
        # Wait for it to be rejected by the network.
        self._ns_lte_cell.set_cell_on()
        LabMobilityLteCsfb.run_test(self)

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        """
        # Restore the list of emergency number in UE Command categories
        # if we changed it in the beginning of the Use Case
        if self._is_emergency_number:
            UECmdTypes.EMERGENCY_NUMBERS_LIST = \
                self._initial_ue_command_em_number

            # Restore the list of emergency numbers
            self._device.set_property_value(
                LabMobilityLteEmergencyCsfb.EMERGENCY_NUMBER_PROPERTY_NAME,
                self._initial_emergency_numbers)
        return LabMobilityLteCsfb.tear_down(self)
