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
:summary: This file implements the switch Codec during Voice call
over GSM network and check the audio quality.
:since: 07/10/2010
:author: ccontreras
"""

import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_GSM_VC_BASE import LabAudioGsmVcBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class LabAudioGsmVcCodSwi(LabAudioGsmVcBase):

    """
    Lab Gsm Voice Call base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabAudioVcBase Init function
        LabAudioGsmVcBase.__init__(self, tc_name, global_config)

        # Read PESQ Targets from Audio_Quality_Targets.xml
        self._pesq_target = ConfigsParser("Audio_Quality_Targets").parse_audio_quality_target("Pesq",
                                                                                              self._codec_type,
                                                                                              "CSV")

        # Read SECONDARY_CODEC from test case xml file
        self.__secondary_codec = str(self._tc_parameters.get_param_value("SECONDARY_CODEC"))

        # Read CODEC_SWITCH_TIME from test case xml file
        self.__codec_switch_time = int(self._tc_parameters.get_param_value("CODEC_SWITCH_TIME"))

        # Read DURATION from test case xml file
        self.__duration = int(self._tc_parameters.get_param_value("DURATION"))

        # Read PESQ_CODEC_SWITCH_FACTOR from test case xml file
        self.__pesq_cswitch_factor = str(self._tc_parameters.get_param_value("PESQ_CODEC_SWITCH_FACTOR"))

        # Calculation of the time sleep duration
        self.__time_sleep = self.__duration - self.__codec_switch_time

        # Calculation of the PESQ target score for codec switch
        self.__pesq_target_cswitch = float(self._pesq_target) - float(self.__pesq_cswitch_factor)

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call UseCaseBase Setup function
        LabAudioGsmVcBase.set_up(self)

        # Time sleep must be superior to 1, if not, raise an exception
        if self.__time_sleep < 0:
            msg = str(self.__time_sleep) + " seconds : waiting time can not have a negative value"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._vc_type == "MO":

            # Dial using a dummy hard-coded phone number
            self._logger.info("Calling 1234 ...")
            self._voicecall_api.dial("1234")

            # Check call status before callSetupTimeout (NS)
            self._vc_2g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
                                               self._call_setup_time)

        elif self._vc_type == "MT":
            # Initiate VoiceCall to CDK
            self._vc_2g.mt_originate_call()
            # pylint: disable=E1101
            # Check call status is incoming before callSetupTimeout
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                                               self._call_setup_time)

            # Answer incoming call
            self._voicecall_api.answer()

            # Check call status before callSetupTimeout (NS)
            self._vc_2g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                               self._call_setup_time)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call UseCaseBase run_test function
        LabAudioGsmVcBase.run_test(self)

        # Configure Audio output to headset (jack)
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.switch_audio_output("headset")

        # Set Voice Call Volume at 100%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 100)

        # Start PESQ
        self._logger.info("Start PESQ measurement")
        self._audio_analyzer.start_single_measurement()

        # Wait during audio file first part playback process using
        # CODEC_SWITCH_TIME parameter
        self._logger.info("Wait during %d seconds for codec switch."
                          % self.__codec_switch_time)

        time.sleep(self.__codec_switch_time)

        # Change voice codec using SECONDARY_CODEC
        self._logger.info("Switch audio codec during file playing.")
        self._vc_2g.set_audio_codec(self.__secondary_codec)

        # Wait during audio file second part playback process
        # (TIME_SLEEP = DURATION - CODEC_SWITCH_TIME)
        self._logger.info("Wait during %d seconds for end file playing."
                          % self.__time_sleep)
        time.sleep(self.__time_sleep)

        # Make a query for PESQ result
        self._pesq_result = float(self._audio_analyzer.query_measurement_result(1, 1))

        self._logger.info("Reference file: %s, Degraded file: %s, Pesq result: %f"
                          % (self._host_ref_file_path,
                             self._host_deg_file_path,
                             self._pesq_result))

        # Get the result of PESQ process and compare with PESQ targets
        # Compute test verdict (if PESQ result > PESQ target the test pass,
        # else the test fails)
        if float(self._pesq_result) > self.__pesq_target_cswitch:
            self._result_verdict = Global.SUCCESS
        else:
            self._result_verdict = Global.FAILURE

        # If User does not want to keep recorded audio file, it will not be stored
        # only if test is PASS
        if self._keep_record is True or self._result_verdict == Global.FAILURE:
            self._audio_analyzer.store_wavefile_UPV(self._aa_deg_file_path)
            self._audio_analyzer.copy_from_upv(self._aa_deg_file_path, self._host_deg_file_path)

        return (self._result_verdict, "PESQ result : %.3f, PESQ target : %s"
                % (self._pesq_result, str(self.__pesq_target_cswitch)))

    def tear_down(self):

        # Release the call
        self._vc_2g.voice_call_network_release()

        try:
            # Check call is released (NS)
            self._vc_2g.check_call_idle(self._registration_timeout,
                                        blocking=False)

            # Check call is released (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL,  # pylint: disable=E1101
                                               self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        # Call LabAudioVcBase Teardown function
        LabAudioGsmVcBase.tear_down(self)

        return Global.SUCCESS, "No errors"
