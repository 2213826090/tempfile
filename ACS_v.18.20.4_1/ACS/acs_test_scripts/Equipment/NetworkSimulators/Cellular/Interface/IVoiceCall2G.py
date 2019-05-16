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
:summary: virtual interface of 2G voice call functionalities for cellular
network simulators
:since: 10/02/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IVoiceCall2G(object):

    """
    IVoiceCall2G class: virtual interface of 2G voice call functionalities
    for cellular network simulators.
    """

    def mt_originate_call(self):
        """
        Originates a mobile terminated call.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_mt_originate_call_timeout(self, timeout):
        """
        Set Originates a mobile terminated call timeout.
        :type delay: integer
        :param config: the maximum timeout before aborting call setup to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_audio_codec(self, codec):
        """
        Sets the audio codec.
        :type codec: str
        :param codec: the audio codec to set. Possible values:
            - "FR" | "EHR" | "HR"
            - "FR_AMR_NB_1220" | "FR_AMR_NB_1020" | "FR_AMR_NB_795" | "FR_AMR_NB_740"
            - "FR_AMR_NB_670"  | "FR_AMR_NB_590"  | "FR_AMR_NB_515" | "FR_AMR_NB_475"
            - "HR_AMR_NB_795"  | "HR_AMR_NB_740"  | "HR_AMR_NB_670" | "HR_AMR_NB_590"
            - "HR_AMR_NB_515"  | "HR_AMR_NB_475"
            - "AMR_WB_1265" | "AMR_WB_885" | "AMR_WB_660"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_speech_configuration(self, config):
        """
        Sets the speech configuration.
        :type config: str
        :param config: the speech configuration to set. Possible values:
                - "ECHO"
                - "SPEECH_OUTPUT"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_echo_loopback_delay(self, delay):
        """
        Sets echo loopback delay.
        :type delay: double
        :param config: the speech echo loopback delay to set. A double
        from 0 to 4 with a resolution of 0.2 (in seconds).
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_call_state(self, state, timeout=0, blocking=True):
        """
        Checks that equipment call state is set to the expected sate
        before the given timeout. If timeout is <= 0, only one test is performed.
        :type state: str
        :param state: the expected state. Possible values:
            .. todo:: to complete with possible values
        :type timeout: integer
        :param timeout: allowed time in seconds to reach the expected state
        :raise: raises TestEquipmentException (error code, error message) in case of failure.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def include_calling_party_number(self, state):
        """
        Sets whether to include the calling party number information record.
        :type state: str
        :param state: If "on" the calling party number parameters is sent to
            the mobile station. If "no" the calling party number parameters is not
            sent to the mobile station.
        :raise exception: AcsBaseException if the parameter is neither on or
            off.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_calling_party_pi(self, state):
        """
        Sets the presentation indicator (PI) used for the calling party number.
        The value is included in the SETUP message to the called mobile station
        when INCLude is selected by CALL:CPNumber:INCLusion
        :type state: str
        :param state: the wanted state for the PI used for the calling party
            number. Should be either ALLOWED, RESTRICTED or NNAVAILABLE.
        :raise AcsBaseException: If the input parameter is not in the expected
            range.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_calling_party_number(self, number):
        """
        Sets the ASCII representation of the calling party number.
        The value is included in the SETUP message to the called mobile
        station when INCLude is selected by CALL:CPNumber:INCLusion.
        It is displayed on the called mobile station's screen when "ALLowed"
        is selected by CALL:CPNumber:PRESentation[:INDicator].
        :type number: str
        :param number: Range: 0 to 20 characters, each character from the set
        of 0123456789abc*# .
        :raise AcsBaseException: if the number passed as parameter is not in
        the expected range.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ignore_call_mode(self, state):
        # TODO: dgonza4x STORY 195 [QCTV][+7] Automate CSFB...
        """
        Enable or disable the ignore call functionality.

        :param state: ignore call mode. Can be "OFF" or "ON"
        :type state: str
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_reject_call_mode(self, state, reason):
        # TODO: dgonza4x STORY 195 [QCTV][+7] Automate CSFB...
        """
        Enable or disable the reject call functionality
        from the network.

        :param state: reject call mode. Can be "OFF" or "ON"
        :type state: str

        :param reason: Integer representing the call rejection reason.
        :type reason: int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def voice_call_network_release(self):
        """
        Releases a voice call.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)