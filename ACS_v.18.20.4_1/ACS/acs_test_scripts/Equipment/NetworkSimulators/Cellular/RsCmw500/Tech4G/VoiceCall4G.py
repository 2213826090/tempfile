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

:organization: INTEL CCG OPM CRD PC&WTE
:summary: voice call 4G implementation for RS CMUW500 cellular network simulator.
:since: 07/01/2014
:author: gescoffr
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IVoiceCall4G import IVoiceCall4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.Audio import Audio as AudioCommon
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.VoiceCall import VoiceCall


class VoiceCall4G(IVoiceCall4G, VoiceCall, AudioCommon):
    """
    Voice Call 4G implementation for RS CMW500
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
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:VIRTualsub1:MTCall:CALL")

    def voice_call_network_release(self):
        """
        Releases a voice call.
        """
        self.get_logger().info("Network released call")
        call_id = self._visa.query_command("SENSe:DATA:CONTrol:IMS2:RELease:LIST?")
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:RELease:CALL:ID %s" % call_id)

    def check_call_state(self, state, timeout=0, blocking=True):
        """
        Checks that equipment call state is set to the expected state
        before the given timeout. If timeout = 0, only one test is performed.
        :type state: str
        :param state: the expected state.
        :type timeout: integer
        :param timeout: allowed time in seconds to reach the expected state
        @:rtype: bool
        :return: boolean to indicate if expected state was reached (true) or not (false)
        """
        self.get_logger().info(
            "Check call state is %s before timeout %d seconds",
            state,
            timeout)

        timer = timeout

        current_state = self._visa.query_command("SENSe:DATA:CONTrol:IMS2:EVENts?").split(',')[-1]

        while (timer > 0) and (current_state != state):
            current_state = self._visa.query_command("SENSe:DATA:CONTrol:IMS2:EVENts?").split(',')[-1]
            time.sleep(1)
            timer -= 1

        if current_state == state:  # Expected state has been reached
            self.get_logger().info("Call state is %s !", state)
            return True
        else:
            # Timeout to reach desired state (Test failed no TestEquipmentException raised)
            self.get_logger().info("Timeout on check call state: %s does not match %s", current_state, state)
            return False

    def check_call_connected(self, timeout=0, blocking=True):
        """
        Checks that call is established between equipment test and DUT before
        timeout seconds. If timeout = 0, only one test is performed.
        :type timeout: integer
        :param timeout: allowed time in seconds to establish the call
        :rtype: bool
        :return: boolean to indicate if call was established (true) or not (false)
        """
        return self.check_call_state("EST", timeout)

    def set_audio_codec(self, codec):
        """
        Sets the audio codec to use
        :type codec: str
        :param codec: the audio codec to set. Possible values:
            AMR NB codecs :
            - "AMR_NB_1220" | "AMR_NB_1020" | "AMR_NB_795"  | "AMR_NB_740"
            - "AMR_NB_670"  | "AMR_NB_590"  | "AMR_NB_515"  | "AMR_NB_475"
            AMR WB codecs :
            - "AMR_WB_2385" | "AMR_WB_2305" | "AMR_WB_1985" | "AMR_WB_1825"
            - "AMR_WB_1585" | "AMR_WB_1425" | "AMR_WB_1265" | "AMR_WB_885"
            - "AMR_WB_660"
        """
        self.get_logger().info("Set audio codec to %s" % codec)

        if self.check_call_connected(0, False):
            # If the DUT is in call, a specific command has to be sent to set the audio codec
            cmd_set_audio_codec = "CONFigure:DATA:CONTrol:IMS2:UPDate:AMR:TYPE"
        else:
            cmd_set_audio_codec = "CONFigure:DATA:CONTrol:IMS2:VIRTualsub:AMR:TYPE"

        if "NB" in codec:
            self._visa.send_command("%s NARRowband" % cmd_set_audio_codec)
            # Remove all previous codec settings
            for i in range(1, 9):
                self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:VIRTualsub:AMR:CODec%s:ENABle OFF" % i)
        elif "WB" in codec:
            self._visa.send_command("%s WIDeband" % cmd_set_audio_codec)
            # Remove all previous codec settings
            for i in range(1, 10):
                self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:VIRTualsub:AMR:CODec%s:ENABle OFF" % i)

        # Transform ACS codecs name into the codec name understood by RS CMW500
        # NB
        if codec == "FR_AMR_NB_1220":
            codec_id = "8"
        elif codec == "FR_AMR_NB_1020":
            codec_id = "7"
        elif codec == "FR_AMR_NB_795":
            codec_id = "6"
        elif codec == "FR_AMR_NB_740":
            codec_id = "5"
        elif codec == "FR_AMR_NB_670":
            codec_id = "4"
        elif codec == "FR_AMR_NB_590":
            codec_id = "3"
        elif codec == "FR_AMR_NB_515":
            codec_id = "2"
        elif codec == "FR_AMR_NB_475":
            codec_id = "1"
        # WB
        elif codec == "AMR_WB_2385":
            codec_id = "9"
        elif codec == "AMR_WB_2305":
            codec_id = "8"
        elif codec == "AMR_WB_1985":
            codec_id = "7"
        elif codec == "AMR_WB_1825":
            codec_id = "6"
        elif codec == "AMR_WB_1585":
            codec_id = "5"
        elif codec == "AMR_WB_1425":
            codec_id = "4"
        elif codec == "AMR_WB_1265":
            codec_id = "3"
        elif codec == "AMR_WB_885":
            codec_id = "2"
        elif codec == "AMR_WB_660":
            codec_id = "1"

        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "%s is an Unknown CODEC or is not supported by the equipment" % codec)

        self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:VIRTualsub:AMR:CODec%s:ENABle ON" % codec_id)

    def set_speech_configuration(self, audio_source):
        """
        Sets the speech configuration

        :type audio_source: str
        :param audio_source: Selects the data which the CMW500 transmits on its DL traffic channel.
        Possible values:
            - AUDioboard (Uses the internal audio board of the CMW500)
            - LOOPback (Echo mode)
            - FORWard (Route to an external media endpoint)
        """
        # Associate the audio measurement interface with the corresponding signalling unit
        self.set_audio_scenario("DAU IMS Server")

        # On the signalling unit, set the speech source to audio_source
        self.get_logger().info("Set speech source to %s" % audio_source)
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:VIRTualsub:MEDiaendpoin %s" % audio_source)

    def set_quality_of_service(self, precondition):
        """
        Specifies whether a voice call session is established with or without quality of service
        preconditions.
        :type precondition: str
        :param precondition: Quality of service precondition
        """
        self.get_logger().info("Set quality of service precondition to %s" % precondition)

        self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:VIRTualsub:MTCall:SIGType %s" % precondition)

    def calibrate_analog_audio_level(self, input_calib_level, output_calib_level):
        """
        Calibrates the input/output peak level of AF connectors
        :type input_calib_level: float
        :param input_calib_level: Input analog full scale peak level
        :type output_calib_level: float
        :param output_calib_level: Output analog full scale peak level
        """
        AudioCommon.calibrate_analog_audio_level(self, input_calib_level, output_calib_level)

    def set_ip_security(self, ip_sec):
        """
        Enables or disables support of the IP security mechanisms by the IMS server

        :type ip_sec: str
        :param ip_sec: Determines if IP security is ON or OFF
        """
        self.get_logger().info("Set IP Sec to %s" % ip_sec)

        self._visa.send_command("CONF:DATA:CONTrol:IMS2:SUBScriber1:IPSec:ENABle %s" % ip_sec)

    def wait_for_state(self, state, call_setup_time):
        """
        This function is an abstract layer to align with the wait_for_state VoiceCall UeCmd used in APx585-based Audio Usecase
        """
        if state is "ACTIVE":
            state = "EST"
        elif state is "OUTGOING_CALL":
            state = "PROG"
        elif state is "OUTGOING_CALL_RING_BACK":
            state = "RING"
        elif state is "OUTGOING_CALL_CANCELING":
            state = "NOK"
        elif state is "READY_TO_CALL":
            state = "REL"
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "State %s is not supported as of yet, or incorrect" % state)

        return self.check_call_state(state, call_setup_time)
