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
:summary: Use case to validate KRI targets during IMS voice call
:since: 17/03/2014
:author: fbelvezx
"""

import time
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_IMS_VC_BASE import LabAudioImsVcBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class LabAudioImsVcRi(LabAudioImsVcBase):
    """
    Lab Audio IMS Voice Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call IMS Audio base Init function
        LabAudioImsVcBase.__init__(self, tc_name, global_config)

        self._polqa_result_dl = None
        self._polqa_result_ul = None

        # Read POLQA Targets from Audio_Quality_Targets.xml
        self._polqa_target = ConfigsParser("Audio_Quality_Targets").parse_audio_quality_target("Polqa",
                                                                                               self._codec_type,
                                                                                               "VoLTE")

        # Read the number of pings to do
        self._nb_pings = self._tc_parameters.get_param_value("PACKET_COUNT")

        # Read the data size of a packet
        self._packet_size = self._tc_parameters.get_param_value("PACKET_SIZE")

        # Get target % of received packet for ping
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._result_verdict = Global.SUCCESS

    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_LTE_BASE set_up function
        LabAudioImsVcBase.set_up(self)

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        # Phone has to see the cell off!
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Enable Data Usage
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context()

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check registration state is connected")
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Check IMS connection state
        self._ns_data_4g.check_ims_connection_state("REG", self._registration_timeout)

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call IMS VoiceCall base run_test function
        LabAudioImsVcBase.run_test(self)

        # Release any previous call (Robustness)
        self._voice_call_api.release()

        # Perform MT voice call if requested by the test
        if self._vc_type == "MT":
            # Trigger MT call from the call box
            self._ns_voice_call_4g.mt_originate_call()
            # Wait for state "incoming" before callSetupTimeout seconds
            self._voice_call_api.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                self._call_setup_time)

            # Answer to that call on the DUT
            self._voice_call_api.answer()
        # Otherwise perform a MO call
        else:
            # Dial using PHONE_NUMBER parameter
            self._voice_call_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_4g.check_call_connected(self._call_setup_time)

        # Wait for state "active" before callSetupTimeout seconds
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # Configure Audio output to headset (jack)
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.switch_audio_output("headset")

        start_time = time.localtime()

        # Set Voice Call Volume at 100%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 100)

        polqa_result_dl_total = 0
        polqa_result_ul_total = 0
        nb_it_dl = 0
        nb_it_ul = 0

        if self._elapsed_time > 0:
            self._elapsed_time = 0

        while self._elapsed_time < self._call_duration:

            # Start POLQA measurement in DL
            [self._polqa_result_dl, meas_in_range] = self.__start_audio_quality_mos("POLQA",
                                                                                    "DL",
                                                                                    self._audio_analyzer_meas_retry)

            # If the current measurement result is not in the expected MOS range, do not take it into account
            if not meas_in_range:
                nb_it_dl -= 1

            # If User does not want to keep recorded audio file, it will not be stored
            # only if test is PASS
            if self._keep_record is True or self._polqa_result_dl < float(self._polqa_target):
                self._host_deg_file_path += time.strftime('%Y%m%d_%H%M%S', time.localtime()) + "_DL" ".wav"
                self._audio_analyzer.copy_from_upv(self._aa_deg_file_path, self._host_deg_file_path)

                self._host_deg_file_path = self._host_deg_file_path.split(str(time.localtime().tm_year))[0]

            # Start POLQA measurement in UL
            [self._polqa_result_ul, meas_in_range] = self.__start_audio_quality_mos("POLQA",
                                                                                    "UL",
                                                                                    self._audio_analyzer_meas_retry)

            # If the current measurement result is not in the expected MOS range, do not take it into account
            if not meas_in_range:
                nb_it_ul -= 1

            # If User does not want to keep recorded audio file, it will not be stored
            # only if test is PASS
            if self._keep_record is True or self._polqa_result_ul < float(self._polqa_target):
                self._host_deg_file_path += time.strftime('%Y%m%d_%H%M%S', time.localtime()) + "_UL" + ".wav"
                self._audio_analyzer.copy_from_upv(self._aa_deg_file_path, self._host_deg_file_path)

                self._host_deg_file_path = self._host_deg_file_path.split(str(time.localtime().tm_year))[0]

            self._error.Msg = "Current POLQA result (DL/UL) : %f / %f, POLQA target : %s" % (self._polqa_result_dl,
                                                                                             self._polqa_result_ul,
                                                                                             self._polqa_target)

            polqa_result_dl_total += self._polqa_result_dl
            polqa_result_ul_total += self._polqa_result_ul
            nb_it_dl += 1
            nb_it_ul += 1

            # Wait for the next measurement to be made while checking the IMS call is still active
            self._ns_voice_call_4g.is_voice_call_connected(self._wait_between_measure)

            # Get elapsed time since call establishment in s
            self._elapsed_time = self.__get_elapsed_time(start_time, time.localtime())
            self._logger.info("Time elapsed since the beginning of the call: %d s" % self._elapsed_time)

        # Perform Network release of the voice call if requested by the test
        if self._vc_type == "NR":
            self._ns_voice_call_4g.voice_call_network_release()
        # Otherwise do a Mobile Release
        else:
            # Mobile Release call
            self._voice_call_api.release()

        # Check voice call state is "released"
        time.sleep(self._wait_btwn_cmd)
        self._ns_voice_call_4g.is_voice_call_idle()

        # Check voice call state is "no_call" on DUT
        time.sleep(self._wait_btwn_cmd)
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)

        self._polqa_result_dl = polqa_result_dl_total / float(nb_it_dl)
        self._polqa_result_ul = polqa_result_ul_total / float(nb_it_ul)

        self._error.Msg = "Average POLQA result (DL/UL) : %f / %f, POLQA target : %s / %s" % (self._polqa_result_dl,
                                                                                              self._polqa_result_ul,
                                                                                              self._polqa_target,
                                                                                              self._polqa_target)

        # Compare the result of POLQA process with POLQA targets
        # Compute test verdict (if POLQA result > POLQA target the test pass,
        # else the test fails)
        if float(self._polqa_result_dl) > float(self._polqa_target) \
            and float(self._polqa_result_ul) > float(self._polqa_target):
            self._logger.info(self._error.Msg)

            # Perform a ping session in case of long lasting voice call
            if self._call_duration > 60:
                # Perform a ping session
                # Compute packet loss value
                packet_loss = self._networking_api.\
                    ping(self._server_ip_address,
                         self._packet_size,
                         self._nb_pings)

                # Compute verdict depending on % of packet loss
                if packet_loss.value > self._target_ping_packet_loss_rate:
                    self._result_verdict = Global.FAILURE
                    self._error.Msg += "Measured Packet Loss: %.0f%s (Target: %.0f%s)"\
                        % (packet_loss.value,
                           packet_loss.units,
                           self._target_ping_packet_loss_rate,
                           packet_loss.units)
                else:
                    self._result_verdict = Global.SUCCESS
            else:
                self._result_verdict = Global.SUCCESS
        else:
            self._logger.info(self._error.Msg)
            self._result_verdict = Global.FAILURE

        return self._result_verdict, self._error.Msg

    def tear_down(self):

        # Call LabAudioImsVcBase Teardown function
        LabAudioImsVcBase.tear_down(self)

        return Global.SUCCESS, "No errors"

    def __get_elapsed_time(self, time_stamp_start, time_stamp_current):
        """
        Get elapsed time between 2 time stamps

        :type time_stamp_start: time.struct_time
        :param time_stamp_start: reference time stamp
        :type time_stamp_current: time.struct_time
        :param time_stamp_current: current time stamp
        """
        elapsed_time = [3600 * 24 * 12 * 365, 3600 * 24 * 12, 3600 * 24, 3600, 60, 1]

        for i in range(len(time_stamp_start) - 3):
            elapsed_time[i] *= time_stamp_current[i] - time_stamp_start[i]

        return sum(elapsed_time)

    def __start_audio_quality_mos(self, test_type, direction, meas_retry):
        """
        Macro function that loads a setup on the UPV audio analyzer, then start a measurement, and returns its value,
        if it's within a certain range
        :type test_type: str
        :param test_type: Type of Audio Quality test (POLQA/PESQ/... )
        :type direction: str
        :param direction: Type of stream on which to perform audio measurement (DL/UL)
        :type meas_retry: int
        :param meas_retry: number of retry
        """

        mos_result_in_range = False
        mos_result = 0
        retry_nb = 0

        while not mos_result_in_range and retry_nb < meas_retry:
            # Load POLQA DL setup on RS UPV
            self._audio_analyzer.load_configuration(
                self._name,
                str(self._ns_node.get_param_value("Model")),
                self._aa_ref_file_path,
                self._codec_type,
                direction)

            # Start POLQA measurement
            mos_result = self._audio_analyzer.audio_quality_mos(self._aa_deg_file_path, test_type, direction)
            self._logger.info("%s POLQA result : " % direction + str(mos_result))

            # Check if POLQA result is within the expected range
            if 0.5 < mos_result < 5:
                mos_result_in_range = True
            else:
                warning_msg = "MOS value out of range, retrying the current measurement"
                self._logger.warning(warning_msg)

            retry_nb += 1

        if not mos_result_in_range:
            mos_result = 0
            error_msg = "MOS value out of range, ignoring the current measurement"
            self._logger.error(error_msg)

        return [mos_result, mos_result_in_range]
