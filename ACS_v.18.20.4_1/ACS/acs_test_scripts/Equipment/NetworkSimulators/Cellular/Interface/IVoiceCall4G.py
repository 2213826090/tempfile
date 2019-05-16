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
:summary: virtual interface of 4G voice call functionalities for cellular network simulators
:since: 07/01/2014
:author: gescoffr
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IVoiceCall4G(object):
    """
    IVoiceCall4G class: virtual interface of 4G voice call functionalities
    for cellular network simulators.
    """

    def mt_originate_call(self):
        """
        Originates a mobile terminated call
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def voice_call_network_release(self):
        """
        Releases a voice call.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_call_state(self, state, timeout=0):
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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_audio_codec(self, codec):
        """
        Sets the audio codec to use
        @type codec: basestring
        @param codec: the audio codec to set. Possible values:
            AMR NB codecs :
            - "AMR_NB_1220" | "AMR_NB_1020" | "AMR_NB_795"  | "AMR_NB_740"
            - "AMR_NB_670"  | "AMR_NB_590"  | "AMR_NB_515"  | "AMR_NB_475"
            AMR WB codecs :
            - "AMR_WB_2385" | "AMR_WB_2305" | "AMR_WB_1985" | "AMR_WB_1825"
            - "AMR_WB_1585" | "AMR_WB_1425" | "AMR_WB_1265" | "AMR_WB_885"
            - "AMR_WB_660"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_speech_configuration(self, audio_source):
        """
        Sets the speech configuration
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_quality_of_service(self, precondition):
        """
        Specifies whether a voice call session is established with or without quality of service
        preconditions.
        @type precondition: basestring
        @param precondition: Quality of service precondition
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def calibrate_analog_audio_level(self, input_calib_level, output_calib_level):
        """
        Calibrates the input/output peak level of AF connectors
        @type input_calib_level
        @param input_calib_level
        @type output_calib_level
        @param output_calib_level
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ip_security(self, ip_sec):
        """
        Enables or disables support of the IP security mechanisms by the IMS server

        :type ip_sec: str
        :param ip_sec: Determines if IP security is ON or OFF
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)