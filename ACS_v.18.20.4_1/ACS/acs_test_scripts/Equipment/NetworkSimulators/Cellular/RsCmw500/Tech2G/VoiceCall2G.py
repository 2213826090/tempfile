"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: voice call 2G implementation for RS CMUW500 cellular network simulator
:since: 09/09/2014
:author: fbelvezx
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IVoiceCall2G import IVoiceCall2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.Audio import Audio as AudioCommon
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.VoiceCall import VoiceCall


class VoiceCall2G(IVoiceCall2G, VoiceCall, AudioCommon):
    """
    Voice Call 2G implementation for RS CMW500
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        VoiceCall.__init__(self, visa)

    def mt_originate_call(self):
        """
        Originates a mobile terminated call
        """
        self.get_logger().info("Network originated call")
        self._visa.send_command("CALL:GSM:SIGN:CSWitched:ACTion CONNect")

    def voice_call_network_release(self):
        """
        Releases a voice call.
        """
        self.get_logger().info("Network released call")
        self._visa.send_command("CALL:GSM:SIGN:CSWitched:ACTion DISConnect")

    def check_call_state(self, state, timeout=0, blocking=True):
        """
        Checks that equipment call state is set to the expected state
        before the given timeout. If timeout = 0, only one test is performed.
        :type state: str
        :param state: the expected state.
        :type timeout: integer
        :param timeout: allowed time in seconds to reach the expected state
        :rtype: bool
        :return: boolean to indicate if expected state was reached (true) or not (false)
        """
        self.get_logger().info(
            "Check call state is %s before timeout %d seconds",
            state,
            timeout)

        timer = timeout
        current_state = self._visa.query_command("FETCh:GSM:SIGN:CSWitched:STATe?")

        while (timer > 0) and (current_state != state):
            current_state = self._visa.query_command("FETCh:GSM:SIGN:CSWitched:STATe?")
            time.sleep(1)
            timer -= 1

        if current_state == state:  # Expected state has been reached
            self.get_logger().info("Call state is %s !", state)
            return True
        else:
            # Timeout to reach desired state (Test failed no TestEquipmentException raised)
            self.get_logger().info("Timeout on check call state: %s does not match %s", current_state, state)
            return False

    def set_audio_codec(self, codec):
        """
        Sets the audio codec.
        :type codec: str
        :param codec: the audio codec to set. Possible values:
            - FR | EFR | HR
            - FR_AMR_NB_GMSK | HR_AMR_NB_GMSK | HR_AMR_NB_8PSK
        """
        self.get_logger().info("Set audio codec to %s" % codec)

        # Transform ACS codecs name into the codec name understood by RS CMW500
        if codec == "FR":
            codec_id = "FV1"
        elif codec == "EFR":
            codec_id = "FV2"
        elif codec == "HR":
            codec_id = "HV1"
        elif codec == "FR_AMR_NB_GMSK":
            codec_id = "ANFG"
        elif codec == "HR_AMR_NB_GMSK":
            codec_id = "ANHG"
        elif codec == "HR_AMR_NB_8PSK":
            codec_id = "ANH8"
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "%s is an Unknown CODEC or is not supported by the equipment" % codec)

        self._visa.send_command("CONFigure:GSM:SIGN:CONNection:CSWitched:TMODe %s" % codec_id)

    def set_speech_configuration(self, audio_source):
        """
        Sets the speech configuration

        :type audio_source: str
        :param audio_source: Selects the data which the CMW500 transmits on its DL traffic channel.
        Possible values:
            - AUDIOBOARD / SPEECH_OUTPUT (Uses the internal audio board of the CMW500)
            - ECHO
        """
        if audio_source in {"AUDIOBOARD", "SPEECH_OUTPUT"}:
            audio_source = "SP1"

        self.set_audio_scenario("GSM Sig1")

        self.get_logger().info("Set speech source to %s" % audio_source)
        self._visa.send_command("CONFigure:GSM:SIGN:CONNection:CSWitched:DSOurce %s" % audio_source)

    def wait_for_state(self, state, call_setup_time):
        """
        This function is an abstract layer to align with the wait_for_state VoiceCall UeCmd used in APx585-based Audio Usecase

        :type state: str
        :param state: expected CS state to be reached before call_setup_timeout
        :type call_setup_time: int
        :param call_setup_time: allowed time in seconds to reach the expected state

        :rtype: bool
        """
        if str(state) is "ACTIVE":
            state = "CEST"
        elif str(state) is "OUTGOING_CALL_RING_BACK":
            state = "ALER"
        elif str(state) is "READY_TO_CALL" or str(state) is "NOCALL":
            state = "SYNC"
        elif str(state) is "INCOMING":
            return True
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "State %s is not supported as of yet, or incorrect" % state)

        return self.check_call_state(state, call_setup_time)