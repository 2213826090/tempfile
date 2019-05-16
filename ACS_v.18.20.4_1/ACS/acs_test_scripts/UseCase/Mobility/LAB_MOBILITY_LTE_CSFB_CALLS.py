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
:summary: Test specific cases of calls during a circuit switched fallback from
LTE to UTRAN
:since: 19/11/2013
:author: dgonza4x
"""

from LAB_MOBILITY_LTE_CSFB import LabMobilityLteCsfb
from acs_test_scripts.UseCase.Mobility.LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from acs_test_scripts.Utilities import RegistrationUtilities as RegUtil
from UtilitiesFWK.Utilities import Global

import time


class LabMobilityLteCsfbCalls(LabMobilityLteCsfb):
    """
    Test specific cases of calls during a circuit switched fallback from LTE
    to UTRAN:
    - Network rejecting the call
    - Cancel voice call before establishment
    """

    def __init__(self, tc_name, global_config):
        # Call LabMobilityLteCsfb init method
        LabMobilityLteCsfb.__init__(self, tc_name, global_config)
        self.__rejection_cause = self._tc_parameters.get_param_value("REJECTION_CAUSE", "not defined", str)
        # There are two kinds of call rejection test:
        #  - REJECTION: the call is rejected by the equipment
        #  - REJECTION_CYCLE: first call is rejected by the equipment, then a second call is performed but is not rejected
        self._rejection_states = ("REJECTION", "REJECTION_CYCLE")

    def set_up(self):
        """
        Set up the test configuration
        """
        status, msg = LabMobilityLteCsfb.set_up(self)

        # check REJECTION_CAUSE parameter
        if self._loss_coverage_type in self._rejection_states and \
           self.__rejection_cause in ("not defined", "", None):
            return Global.FAILURE, "Rejection cause parameter is not correct (%s). Please update your test-case"\
                                   % self.__rejection_cause

        # cancel call specific setup
        if self._loss_coverage_type == "CANCELLATION":
            if self._ns_3gsm_cell_tech == "2G":
                # set call setup timeout to 100 seconds
                self._ns_3gsm_vc.set_mt_originate_call_timeout(100)

        elif self._loss_coverage_type in self._rejection_states:
            if self._ns_3gsm_cell_tech == "2G":
                # The network simulator must not ignore calls
                self._ns_3gsm_vc.set_ignore_call_mode("OFF")
                # allow the network simulator to reject calls
                self._ns_3gsm_vc.set_reject_call_mode("ON", self.__rejection_cause)

        return status, msg

    def run_test(self):
        """
        Execute the test
        """
        LabMobilityLte3gsmBase.run_test(self)
        csfb_return_lte_timeout = 30
        time_to_wait_before_3g_idle = 10
        status = Global.SUCCESS
        msg = "No error occurred"
        try:
            # perform a call
            # Release any previous call (Robustness)
            self._voicecall_api.release()
            time.sleep(self._wait_btwn_cmd)

            if self._vc_type == "MT":
                # Since we are testing CSFB MT, we need to HO from LTE cell
                # to 3GSM using "DL Info CS Service Notify" and message number
                # equals to 1 in this case
                self._ns_lte.send_ho_message("1")

                # pylint: disable=E1101
                # Check call status is incoming before 2*callSetupTimeout as 8960 is a bit stressed at that moment
                self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                                   2 * self._call_setup_time)
            else:
                # Initiate a LTE RRC Connection Release when performing MO Call to go back to IDLE
                self._ns_lte_data.ue_detach()

                # Dial using the phone number given in parameters
                self._logger.info("Calling %s ..." % self._phone_number)
                # Dial and wait that call is active
                self._voicecall_api.dial(self._phone_number)
                if self._loss_coverage_type in self._rejection_states:
                    # Then callbox will reject the call, as it is impossible to check on 8960 that call has been rejected
                    # check that phone is no more in call to validate that call has been rejected
                    self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL, self._call_setup_time)
                elif self._ns_3gsm_cell_tech == "2G":
                    # Check voice call is incoming on reselected network side
                    self._ns_3gsm_vc.check_call_state(
                        "ALER", 2 * self._call_setup_time, blocking=True)

            self._modem_api.check_network_type_before_timeout(self._ns_3gsm_data.get_network_type(),
                                                              self._registration_timeout)

            # Specific cases for call cancellation
            # (rejection is done automatically)
            if self._loss_coverage_type == "CANCELLATION":
                self.release_csfb_vc(time_to_wait_before_3g_idle)
            elif self._loss_coverage_type in self._rejection_states:
                # Check Data Connection State on LTE cell => CON before timeout
                # Check that DUT is registered on the good RAT
                RegUtil.check_rat_and_data_connection_state(self._ns_lte_data,
                                                            "CON",
                                                            self._modem_api,
                                                            csfb_return_lte_timeout)
            # make a ping
            self.ping_with_retry()

            # In case of REJECTION_CYCLE test, this call shall be accepted by callbox
            if self._loss_coverage_type == "REJECTION_CYCLE":
                if self._ns_3gsm_cell_tech == "2G":
                    # The network simulator must not ignore calls
                    self._ns_3gsm_vc.set_ignore_call_mode("OFF")
                    # allow the network simulator must not reject calls
                    self._ns_3gsm_vc.set_reject_call_mode("OFF", self.__rejection_cause)

                self.originate_csfb_vc()

                self.release_csfb_vc()

                self.ping_with_retry()

        except Exception:
            self._logger.info("Error occurred")
            status = Global.FAILURE
            msg = "Error occurred"
        finally:
            if self._loss_coverage_type in self._rejection_states:
                if self._ns_3gsm_cell_tech == "2G":
                    # The network simulator must not ignore calls
                    self._ns_3gsm_vc.set_ignore_call_mode("OFF")
                    # allow the network simulator to reject calls
                    self._ns_3gsm_vc.set_reject_call_mode("ON", self.__rejection_cause)

            # Reset 3G cell
            self._ns_3gsm_cell.set_cell_power(self._ns_3gsm_cell_power)
            return status, msg
