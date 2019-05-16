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

:summary: This file implements
Call on simulated network.
:since: 04/09/2013
:author: fbelvezx
"""

import os
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.AudioFrameworkUtilities import AudioFramework
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_WCDMA_VC_BASE import LabAudioWcdmaVcBase
import acs_test_scripts.Utilities.AudioUtilities as AudioUtil
from ErrorHandling.AcsBaseException import AcsBaseException


class LabAudioWcdmaCsv(LabAudioWcdmaVcBase):
    """
    Lab Audio Routing Wcdma Voice Call Mobile Originated/Terminated class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabAudioWcdmaVcBase.__init__(self, tc_name, global_config)

        # Read PHONE_NUMBER from testcase xml parameters
        if (self._tc_parameters.get_param_value("PHONE_NUMBER") not in (None, '')) \
            and str(self._tc_parameters.get_param_value("PHONE_NUMBER")).isdigit():
            self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")
        else:
            self._phone_number = str(self._device.get_phone_number())

        if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":

            # Construct trace file path on audio analyzer
            self._aa_trace_file_path_dl = os.path.join(self._aa_dest_path,
                                                    "RMS_sweep_dl.trc")

            # Construct trace file path on audio analyzer
            self._aa_trace_file_path_ul = os.path.join(self._aa_dest_path,
                                                    "RMS_sweep_ul.trc")

            # Construct trace file path on bench computer
            self._host_trace_file_path_dl = os.path.join(self._aa_conf_path,
                                                         "UPV_traces",
                                                         self._name.split('\\')[-1].split('-')[0] + "-" +
                                                         str(self._tc_parameters.get_ucase_name()) + "_" +
                                                         self._cur_date_time + "_DL_1" ".trc")

            # Construct trace file path on bench computer
            self._host_trace_file_path_ul = os.path.join(self._aa_conf_path,
                                                         "UPV_traces",
                                                         self._name.split('\\')[-1].split('-')[0] + "-" +
                                                         str(self._tc_parameters.get_ucase_name()) + "_" +
                                                         self._cur_date_time + "_UL_1" ".trc")

            # Threshold for audio routing detection
            self._dynamic_threshold = int(self._tc_parameters.get_param_value("THRESHOLD"))

            # Number of iteration for the current test
            self._tc_iteration_count = self._tc_parameters.get_b2b_iteration()

            # Duration of the audio test file
            self._test_file_duration = int(self._tc_parameters.get_param_value("TEST_SIGNAL_DURATION"))

        # Call Stream Volume in percent
        self._call_stream_volume_dut = \
            int(self._tc_parameters.get_param_value("CALL_VOLUME"))

        # Call duration in second
        self._call_duration = int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read time between 2 successive measurement
        self._wait_between_measure = int(self._tc_parameters.get_param_value("WAIT_BETWEEN_MEASURE"))

        self._elapsed_time = 0

    #------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # Call LabAudioVcBase Run Test function
        LabAudioWcdmaVcBase.run_test(self)

        # Initiate a MO voice call
        if self._vc_type == "MO":

            # Dial using phone number
            self._logger.info("Calling %s ..." % self._phone_number)
            self._voicecall_api.dial(self._phone_number)

            # Check call status before callSetupTimeout (NS)
            self._vc_3g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (DUT)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE, # pylint: disable=E1101
                                               self._call_setup_time)

        else:
            # Initiate a MT voice call
            self._vc_3g.mt_originate_call()
            # pylint: disable=E1101
            # Check call status is incoming before callSetupTimeout
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                               self._call_setup_time)

            # Answer incoming call
            self._voicecall_api.answer()

            # Check call status before callSetupTimeout (NS)
            self._vc_3g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (DUT)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                               self._call_setup_time)

        start_time = time.localtime()

        # Configure Audio output to headset (jack)
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.switch_audio_output("headset")

        # Set Voice Call Volume
        self._system_api.adjust_specified_stream_volume("VoiceCall", self._call_stream_volume_dut)

        if self._elapsed_time > 0:
            self._elapsed_time = 0

        self._logger.info("Start verifying audio routing - Call duration = %d s" % self._call_duration)

        while self._elapsed_time < self._call_duration:

            # Scenario 1: The audio analyzer is an equipment (RS UPV here)
            if self._audio_analyzer_node.get_param_value("Model") in "RS_UPV":

                # Load UPV setup for DL path
                self._audio_analyzer.load_configuration(
                    self._name,
                    str(self._ns_node.get_param_value("Model")),
                    self._aa_ref_file_path,
                    self._codec_type,
                    "DL")

                # Perform audio routing verification for DL path first
                self._logger.info("Verify audio routing in DL")
                self._audio_analyzer.start_single_measurement()

                # Wait for the measurement to end
                self._audio_analyzer.wait_for_sweep_state("sweep_waiting")

                # Transfer the result of the measurement to ACS host
                self._audio_analyzer.store_data_list(self._aa_trace_file_path_dl,
                                                     self._host_trace_file_path_dl,
                                                     "SWEep")

                # Get the dynamic range indicator for DL audio path
                res_dl = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path_dl),
                                                      self._logger)

                # Load UPV setup for UL path
                self._audio_analyzer.load_configuration(
                    self._name,
                    str(self._ns_node.get_param_value("Model")),
                    self._aa_ref_file_path,
                    self._codec_type,
                    "UL")

                # Perform audio routing verification for UL path
                self._logger.info("Verify audio routing in UL")
                self._audio_analyzer.start_single_measurement()

                # Wait for the measurement to end
                self._audio_analyzer.wait_for_sweep_state("sweep_waiting")

                # Transfer the result of the measurement to ACS host
                self._audio_analyzer.store_data_list(self._aa_trace_file_path_ul,
                                                     self._host_trace_file_path_ul,
                                                     "SWEep")

                # Get the dynamic range indicator for UL audio path
                res_ul = AudioUtil.get_rms_from_sweep(AudioUtil.get_sweep_trace(self._host_trace_file_path_ul),
                                                      self._logger)

                if res_dl > self._dynamic_threshold and res_ul > self._dynamic_threshold:
                    self._error.Msg = "Audio correctly routed in both DL and UL - " + \
                                      "Dynamic indicator : DL = %f / UL = %f" % (res_dl, res_ul)
                    self._logger.info(self._error.Msg)
                    self._result_verdict = Global.SUCCESS
                elif res_dl < self._dynamic_threshold and res_ul > self._dynamic_threshold:
                    self._error.Msg = "Audio routing error in DL - " + \
                                      "Dynamic indicator : DL = %f / UL = %f" % (res_dl, res_ul)
                    self._logger.error(self._error.Msg)
                    self._result_verdict = Global.FAILURE
                elif res_ul < self._dynamic_threshold and res_dl > self._dynamic_threshold:
                    self._error.Msg = "Audio routing error in UL - " + \
                                      "Dynamic indicator : DL = %f / UL = %f" % (res_dl, res_ul)
                    self._logger.error(self._error.Msg)
                    self._result_verdict = Global.FAILURE
                else:
                    self._error.Msg = "Audio routing error in both DL and UL - " + \
                                      "Dynamic indicator : DL = %f / UL = %f" % (res_dl, res_ul)
                    self._logger.error(self._error.Msg)
                    self._result_verdict = Global.FAILURE

                if self._keep_record:
                    # Update the name of the trace file, in case of back-to-back and KEEP_RECORD = true
                    self._host_trace_file_path_dl = self._host_trace_file_path_dl.strip(self._host_trace_file_path_dl.split('_')[-1]) + \
                                                 str(self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 1) \
                                                 + ".trc"
                    self._host_trace_file_path_ul = self._host_trace_file_path_ul.strip(self._host_trace_file_path_ul.split('_')[-1]) + \
                                                 str(self._tc_parameters.get_b2b_iteration() - self._tc_iteration_count + 1) \
                                                 + ".trc"
                else:
                    os.remove(self._host_trace_file_path_dl)
                    os.remove(self._host_trace_file_path_ul)

                self._tc_iteration_count -= 1

            # Scenario 2: The audio analyzer is an external python framework
            elif self._audio_analyzer_node.get_param_value("Model") in "AUDIO_FRAMEWORK":
                [self._result_verdict, self._error.Msg] = AudioFramework.start_audio_routing(self._audio_framework,
                                                                                             self._ref_file,
                                                                                             self._audio_framework_timeout,
                                                                                             wait_for_timeout=1)

            if self._result_verdict is Global.FAILURE:
                break

            # Wait for the next measurement to be made while checking the CSV call is still active
            self._vc_3g.is_voice_call_connected(self._wait_between_measure)

            # Get elapsed time since call establishment in s
            self._elapsed_time = self.__get_elapsed_time(start_time, time.localtime())
            self._logger.info("Time elapsed since the beginning of the call: %d s" % self._elapsed_time)

        # Release the call
        self._vc_3g.voice_call_network_release()

        try:
            # Check call is released (NS)
            self._vc_3g.check_call_idle(self._registration_timeout,
                                        blocking=False)

            # Check call is released (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL, # pylint: disable=E1101
                                               self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        return self._result_verdict, self._error.Msg

    #------------------------------------------------------------------------------
    def tear_down(self):

        # Call LabAudioVcBase Teardown function
        LabAudioWcdmaVcBase.tear_down(self)

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
