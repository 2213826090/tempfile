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
:summary: voice call TDSCDMA implementation for RS CMUW500 cellular network simulator
:since: may 07th 2015
:author: Martin Brisbarre
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IVoiceCall3G import IVoiceCall3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.Audio import Audio as AudioCommon
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.VoiceCall import VoiceCall


class VoiceCallTDSCDMA(IVoiceCall3G, VoiceCall, AudioCommon):
    """
    Voice Call 3G implementation for RS CMW500
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
        self._visa.send_command("CALL:TDSCdma:SIGN:CSWitched:ACTion CONNect")

    def voice_call_network_release(self):
        """
        Releases a voice call.
        """
        self.get_logger().info("Network released call")
        self._visa.send_command("CALL:TDSCdma:SIGN:CSWitched:ACTion DISConnect")

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
        current_state = self._visa.query_command("FETCh:TDSCdma:SIGN:CSWitched:STATe?")

        while (timer > 0) and (current_state != state):
            current_state = self._visa.query_command("FETCh:TDSCdma:SIGN:CSWitched:STATe?")
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
        Sets the audio codec to use
        :type codec: str
        :param codec: the audio codec to set. Possible values:
            AMR NB codecs :
            - AMR_NB_1220 | AMR_NB_1020 | AMR_NB_795  | AMR_NB_740
            - AMR_NB_670  | AMR_NB_590  | AMR_NB_515  | AMR_NB_475
            AMR WB codecs :
            - AMR_WB_2385 | AMR_WB_2305 | AMR_WB_1985 | AMR_WB_1825
            - AMR_WB_1585 | AMR_WB_1425 | AMR_WB_1265 | AMR_WB_885
            - AMR_WB_660
        """
        self.get_logger().info("Set audio codec to %s" % codec)

        # Transform ACS codecs name into the codec name understood by RS CMW500
        # NB
        if codec == "FR_AMR_NB_1220":
            codec_id = "A"
        elif codec == "FR_AMR_NB_1020":
            codec_id = "B"
        elif codec == "FR_AMR_NB_795":
            codec_id = "C"
        elif codec == "FR_AMR_NB_740":
            codec_id = "D"
        elif codec == "FR_AMR_NB_670":
            codec_id = "E"
        elif codec == "FR_AMR_NB_590":
            codec_id = "F"
        elif codec == "FR_AMR_NB_515":
            codec_id = "G"
        elif codec == "FR_AMR_NB_475":
            codec_id = "H"
        # WB
        elif codec == "AMR_WB_2385":
            codec_id = "A"
        elif codec == "AMR_WB_2305":
            codec_id = "B"
        elif codec == "AMR_WB_1985":
            codec_id = "C"
        elif codec == "AMR_WB_1825":
            codec_id = "D"
        elif codec == "AMR_WB_1585":
            codec_id = "E"
        elif codec == "AMR_WB_1425":
            codec_id = "F"
        elif codec == "AMR_WB_1265":
            codec_id = "G"
        elif codec == "AMR_WB_885":
            codec_id = "H"
        elif codec == "AMR_WB_660":
            codec_id = "I"

        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "%s is an Unknown CODEC or is not supported by the equipment" % codec)

        if "NB" in codec:
            self._visa.send_command("CONFigure:TDSCdma:SIGN:CONNection:VOICe:CODec NB")
            self._visa.send_command("CONFigure:TDSCdma:SIGN:CONNection:VOICe:AMR:NARRow %s" % codec_id)
        elif "WB" in codec:
            self._visa.send_command("CONFigure:TDSCdma:SIGN:CONNection:VOICe:CODec WB")
            self._visa.send_command("CONFigure:TDSCdma:SIGN:CONNection:VOICe:AMR:WIDE %s" % codec_id)

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
            audio_source = "SPEech"

        elif audio_source in "ECHO":
            audio_source = "LOOPback"

        self._visa.send_command("CONFigure:TDSCdma:SIGN:CONNection:UETerminate VOICe")
        self.set_audio_scenario("TDSCDMA Sig1")

        self.get_logger().info("Set speech source to %s" % audio_source)
        self._visa.send_command("CONFigure:TDSCdma:SIGN:CONNection:VOICe:SOURce %s" % audio_source)

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
            state = "REG"
        elif str(state) is "INCOMING":
            return True
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "State %s is not supported as of yet, or incorrect" % state)

        return self.check_call_state(state, call_setup_time)
