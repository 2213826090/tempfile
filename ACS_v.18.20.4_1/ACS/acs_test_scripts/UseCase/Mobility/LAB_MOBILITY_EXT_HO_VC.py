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
:summary: This file implements usecase that do a mobility handover
during voice call
:since: 12/08/2011
:author: ccontreras
"""
import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil
from ErrorHandling.TestEquipmentException import TestEquipmentException

from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from ErrorHandling.DeviceException import DeviceException


class LabMobilityExtHoVc(LabMobility3gsmBase):

    """
    Mobility handover during voice call Usecase
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobility3gsmBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)

        # Read PHONE_NUMBER from testcase xml parameters
        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobility3gsmBase Set_up function
        LabMobility3gsmBase.set_up(self)

        # Set cell on
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
                                                               flightmode_cycle=False,
                                                               blocking=False)

        # Get RAT from Equipment
        network_type = self._ns1_data.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type, self._registration_timeout)

        # Set NS2 cell on
        self._ns2_cell.set_cell_on()

        # Perform MO voice call on active network simulator
        self._voicecall_api.dial(self._phone_number, True)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns1_vc.check_call_connected(self._call_setup_time,
                                          blocking=False)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        try:
            # Wait for Voice Call to stay active before HandOver
            time.sleep(5)

            # Perform Handover while Voice call is active between the 2 cells
            # Log the current Hard Handover iteration
            self._logger.info("Performing Hard Handover from Cell1 to Cell2")

            # Perform handover
            self._ns1_cell.execute_external_handover()

            # Wait for HandOver to be done with Voice Call still active
            time.sleep(15)

            # Check call state "CONNECTED" before 10 seconds to validate handover
            self._ns2_vc.check_call_connected(10, blocking=False)

            # Log the current Hard Handover iteration
            self._logger.info("Performing Hard Handover from Cell2 to Cell1")

            # Perform handover
            self._ns2_cell.execute_external_handover()

            # Wait for HandOver to be done with Voice Call still active
            time.sleep(15)

            # Check call state "CONNECTED" before 10 seconds to validate handover
            self._ns1_vc.check_call_connected(10, blocking=False)

        # Catch eventually the exception
        except TestEquipmentException as ex:
            raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED,
                                    "Exception during handover process %s"
                                    % (ex.get_error_message()))

        except DeviceException as ex:
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR,
                                    "Exception during handover process %s"
                                    % (ex.get_error_message()))

        return Global.SUCCESS, "Handovers were performed successfully."

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Mobile Release call
        try:
            self._voicecall_api.release()
        except DeviceException as ex:
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR,
                                    "Exception during call release process %s"
                                    % (ex.get_error_message()))
        return Global.SUCCESS, "No Error"
