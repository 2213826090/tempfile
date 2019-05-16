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
:summary: This file implements the check Audio Quality (PESQ) during WCDMA
Voice Call on simulated network.
:since: 07/10/2010
:author: ccontreras
"""

import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Audio.LAB_AUDIO_WCDMA_VC_BASE import LabAudioWcdmaVcBase
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class LabAudioWcdmaVcPesq(LabAudioWcdmaVcBase):

    """
    Lab Audio Wcdma Voice Call Mobile Terminated Pesq base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabAudioWcdmaVcBase.__init__(self, tc_name, global_config)

        # Read PESQ Targets from Audio_Quality_Targets.xml
        self._pesq_target = ConfigsParser("Audio_Quality_Targets").parse_audio_quality_target("Pesq",
                                                                                              self._codec_type,
                                                                                              "CSV")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Setup
        """
        LabAudioWcdmaVcBase.set_up(self)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabAudioVcBase Run Test function
        LabAudioWcdmaVcBase.run_test(self)

        # Make a MO/MT voice call
        if self._vc_type == "MO":

            # Dial using a dummy hard-coded phone number
            self._logger.info("Calling 1234 ...")
            self._voicecall_api.dial("1234")

            # Check call status before callSetupTimeout (NS)
            self._vc_3g.check_call_connected(self._call_setup_time,
                                             blocking=False)

            # Check call status before callSetupTimeout (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,  # pylint: disable=E1101
                                               self._call_setup_time)

        elif self._vc_type == "MT":
            # Initiate VoiceCall to CDK
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

            # Check call status before callSetupTimeout (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
                                               self._call_setup_time)

        # Configure Audio output to headset (jack)
        time.sleep(self._wait_btwn_cmd)
        self._phonesystem_api.switch_audio_output("headset")

        # Set Voice Call Volume at 100%
        self._system_api.adjust_specified_stream_volume("VoiceCall", 100)

        # PESQ measurement
        self._logger.info("Start PESQ measurement")
        self._pesq_result = self._audio_analyzer.audio_quality_mos(self._aa_deg_file_path, "PESQ", "DL")
        self._logger.info("PESQ result : " + str(self._pesq_result))

        # Compare the result of PESQ process with PESQ targets
        # Compute test verdict (if PESQ result > PESQ target the test pass,
        # else the test fails)
        if float(self._pesq_result) > float(self._pesq_target):
            self._result_verdict = Global.SUCCESS
        else:
            self._result_verdict = Global.FAILURE

        # If User does not want to keep recorded audio file, it will not be stored
        # only if test is PASS
        if self._keep_record is True or self._result_verdict == Global.FAILURE:
            self._audio_analyzer.copy_from_upv(self._aa_deg_file_path, self._host_deg_file_path)

        # Release the call
        self._vc_3g.voice_call_network_release()

        try:
            # Check call is released (NS)
            self._vc_3g.check_call_idle(self._registration_timeout,
                                        blocking=False)

            # Check call is released (CDK)
            self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.NOCALL,  # pylint: disable=E1101
                                               self._call_setup_time)

        except AcsBaseException as acs_exception:
            self._logger.warning("Call release fail:" + str(acs_exception))

        return (self._result_verdict, "PESQ result : %.3f, PESQ target : %s"
                % (self._pesq_result, self._pesq_target))

#------------------------------------------------------------------------------
    def tear_down(self):

        # Call LabAudioVcBase Teardown function
        LabAudioWcdmaVcBase.tear_down(self)

        return Global.SUCCESS, "No errors"
