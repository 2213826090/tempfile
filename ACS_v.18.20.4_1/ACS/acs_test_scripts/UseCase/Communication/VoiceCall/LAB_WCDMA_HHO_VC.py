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
:summary: This file is the Use Case for wcdma hard handovers during voice call
:since: 05/09/2013
:author: sjamaoui
"""
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Networking.LAB_WCDMA_BASE import LabWcdmaBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException


class LabWcdmaHhoVc(LabWcdmaBase):

    """
    Usecase for mobility Wcdma hard handover during a voice call
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_MOBILITY_3G_HHO_BASE Init function
        LabWcdmaBase.__init__(self, tc_name, global_config)

        # Read PHONE_NUMBER from testcase xml parameters as integer
        self._phone_number = \
            self._tc_parameters.get_param_value("PHONE_NUMBER")
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_MOBILITY_3G_HHO_BASE run_test function
        LabWcdmaBase.run_test(self)

        # Perform a mobile originated call
        self._voicecall_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_vc.check_call_connected(self._call_setup_time, blocking=False)

        # Reset number of succeded jump to 0
        nb_jump_success = 0

        # Perform try catch in order to catch errors during the for iteration.
        # If this try catch isn't done here and a crash appears during the for
        # iteration, the Tear Down will be called without recording Use Case
        # verdict and maybe 1 or more succeeded handovers.
        try:

            # UNTIL CELL_JUMP_NUMBER hasn't been done, DO:
            while nb_jump_success < self._jump_num:
                # Log the current iteration
                self._logger.info(
                    "Performing hard handover number %d of %d.",
                    nb_jump_success + 1,
                    self._jump_num)

                # Perform a hard handover using 10 seconds timeout
                self._ns_cell.execute_hard_handover()
                time.sleep(10)

                # Retrieve the new current ARFCN
                new_current_arfcn = self._ns_cell.get_downlink_arfcn()

                # Retrieve the new pcr ARFCN
                new_pcr_arfcn = self._ns_cell.get_pcr_downlink_arfcn()

                # Check that current and pcr arfcn have been inverted
                if new_current_arfcn != self._pcr_arfcn:
                    msg = "Hard handover has failed: current and pcr arfcn " \
                        + "haven't been inverted"
                    self._logger.error(msg)
                    raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR, msg)

                # Update current and pcr ARFCN attributes
                self._current_arfcn = new_current_arfcn
                self._pcr_arfcn = new_pcr_arfcn

                # Check call state "CONNECTED" before 10 seconds to validate
                # hard handover
                self._ns_vc.check_call_connected(10, blocking=False)

                # Get RAT from Equipment
                network_type = self._ns_data_3g.get_network_type()

                # Check that DUT is registered on the good RAT
                self._modem_api.check_network_type_before_timeout(network_type,
                                                                  self._registration_timeout)

                # Increase number of jump success
                nb_jump_success += 1

                # Log the hard handover success
                self._logger.info(
                    "Hard handover number %d of %d succeeded",
                    nb_jump_success,
                    self._jump_num)

            # END UNTIL loop

            # Release the voice call
            self._ns_vc.voice_call_network_release()

        # Catch eventually the exception
        except TestEquipmentException as ex:
            # Log a warning in case the expected number of hard handover(s)
            # is reached
            if nb_jump_success == self._jump_num:
                self._logger.warning(
                    "Exception during hard handover process %s",
                    str(ex.get_error_message()))

            # In other case raise the error
            else:
                msg = "The hard handover number %d failed" % \
                    (nb_jump_success + 1)
                msg += "(%d succeeded hard handover(s) on %d)." \
                    % (nb_jump_success, self._jump_num)

                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR,
                                       "Exception during hard handover process %s (%s)"
                                       % (ex.get_error_message(), msg))

        # Compute final verdict
        msg = "%d hard handover(s) done on %d." \
            % (nb_jump_success, self._jump_num)

        # Return verdict and verdict message
        return Global.SUCCESS, msg
