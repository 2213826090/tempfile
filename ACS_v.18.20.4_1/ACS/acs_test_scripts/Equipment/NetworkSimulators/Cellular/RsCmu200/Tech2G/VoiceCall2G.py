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
:summary: voice call 2G implementation for RS CMU200 cellular network simulator
:since: 05/04/2011
:author: ymorel
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IVoiceCall2G import IVoiceCall2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech2G import WVoiceCall2G as W


class VoiceCall2G(IVoiceCall2G):

    """
    Voice Call 2G implementation for RS CMU200
    """

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (RsCmu200)
        """
        IVoiceCall2G.__init__(self)
        self.__root = root

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def get_root(self):
        """
        Get the root object of the equipment
        :rtype: RsCmu200
        :return: the root object of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def mt_originate_call(self):
        """
        Originates a mobile terminated call.
        """
        self.get_logger().info("Network originated call")
        (err, msg) = W.MtOriginateCall(self.get_root())
        self.__error_check(err, msg)

    def set_mt_originate_call_timeout(self, timeout):
        """
        Set Originates a mobile terminated call timeout.
        :type delay: integer
        :param config: the maximum timeout before aborting call setup to set (0..60).
        """
        (err, msg) = W.SetMtOriginateCallTimeout(self.get_root(), int(timeout))
        self.__error_check(err, msg)

    def set_audio_codec(self, codec):
        """
        Sets the audio codec.
        :type codec: str
        :param codec: the audio codec to set. Possible values:
            - "FR" | "EFR" | "HR"
            - "FR_AMR_NB_1220" | "FR_AMR_NB_1020" | "FR_AMR_NB_795" | "FR_AMR_NB_740"
            - "FR_AMR_NB_670"  | "FR_AMR_NB_590"  | "FR_AMR_NB_515" | "FR_AMR_NB_475"
            - "HR_AMR_NB_795"  | "HR_AMR_NB_740"  | "HR_AMR_NB_670" | "HR_AMR_NB_590"
            - "HR_AMR_NB_515"  | "HR_AMR_NB_475"
            - Following WFSP codecs are not supported by CMU200 :
                - "AMR_WB_1265" | "AMR_WB_885" | "AMR_WB_660"
        """
        cmu_codec = None
        # Transform ACS codecs name into the codec name understood by Agilent 8960
        if codec == "FR":
            cmu_codec = "FR"
        elif codec == "EFR":
            cmu_codec = "EFR"
        elif codec == "HR":
            cmu_codec = "HR"
        elif codec == "FR_AMR_NB_1220":
            cmu_codec = "FR_AMR_1220"
        elif codec == "FR_AMR_NB_1020":
            cmu_codec = "FR_AMR_1020"
        elif codec == "FR_AMR_NB_795":
            cmu_codec = "FR_AMR_795"
        elif codec == "FR_AMR_NB_740":
            cmu_codec = "FR_AMR_740"
        elif codec == "FR_AMR_NB_670":
            cmu_codec = "FR_AMR_670"
        elif codec == "FR_AMR_NB_590":
            cmu_codec = "FR_AMR_590"
        elif codec == "FR_AMR_NB_515":
            cmu_codec = "FR_AMR_515"
        elif codec == "FR_AMR_NB_475":
            cmu_codec = "FR_AMR_475"
        elif codec == "HR_AMR_NB_795":
            cmu_codec = "HR_AMR_795"
        elif codec == "HR_AMR_NB_740":
            cmu_codec = "HR_AMR_740"
        elif codec == "HR_AMR_NB_670":
            cmu_codec = "HR_AMR_670"
        elif codec == "HR_AMR_NB_590":
            cmu_codec = "HR_AMR_590"
        elif codec == "HR_AMR_NB_515":
            cmu_codec = "HR_AMR_515"
        elif codec == "HR_AMR_NB_475":
            cmu_codec = "HR_AMR_475"
        else:
            raise TestEquipmentException(
                TestEquipmentException.INVALID_PARAMETER,
                "%s is an Unknown CODEC or is not supported by the equipment" % codec)

        (err, msg) = W.SetAudioCodec(self.get_root(), cmu_codec)
        self.__error_check(err, msg)

    def set_speech_configuration(self, config):
        """
        Sets the speech configuration.
        :type config: str
        :param config: the speech configuration to set. Possible values:
            - "ECHO"
            - "NONE"
            - "PRBS15"
            - "PRBS9"
            - "SIN300"
            - "SIN1000"
            - "SIN3000"
            - "MULTITONE"
            - "CUST": custom
        """
        (err, msg) = W.SetSpeechConfiguration(self.get_root(), config)
        self.__error_check(err, msg)

    def is_voice_call_idle(self):
        """
        Tests if voice call is idle (disconnected).
        :rtype: boolean
        :return:
            - true if the voice Call status is idle
            - false otherwise
        :raise: raises TestEquipmentException (error code, error message) in case of failure.
        """
        (err, status, msg) = W.GetCallControlStatus(self.get_root())
        self.__error_check(err, msg)
        # Idle status can be IDLE (ON) or OFF or SYNC RS CMU200 cell state
        return (status == "IDLE") or (status == "OFF") or (status == "SYNC")

    def is_voice_call_connected(self, connection_time=0, blocking=True):
        """
        Tests if the voice call is established between equipment and DUT for
        connection_time seconds. If connection_time is <= 0, only one test
        is performed.
        :type connection_time: integer
        :param connection_time: expected minimum connection time
        :type blocking: boolean
        :param blocking: raise or not an exception
        :rtype: boolean
        :return:
            - true if the voice call is established for connection_time seconds or if
        voice call is connected
            - false otherwise
        :raise: TestEquipmentException (error code, error message) in case of failure.
        """
        self.get_logger().info(
            "Check if voice call connected for %d seconds",
            connection_time)

        timer = 0
        (err, status, msg) = W.GetCallControlStatus(self.get_root())
        self.__error_check(err, msg)

        while (timer < connection_time) and (status == "CONN"):
            (err, status, msg) = W.GetCallControlStatus(self.get_root())
            self.__error_check(err, msg)
            time.sleep(1)
            timer += 1

        if status != "CONN":  # Fail
            if blocking:
                msg = "Voice call interrupted after %d seconds!" % (timer)
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                msg = "Voice call interrupted after %d seconds!" % (timer)
                self.get_logger().info(msg)
                return False

        self.get_logger().info(
            "Voice call still connected after %d seconds!",
            connection_time)
        return True

    def check_call_state(self, state, timeout=0, blocking=True):
        """
        Checks that equipment call state is set to the expected sate
        before the given timeout. If timeout is <= 0, only one test is performed.
        :type state: str
        :param state: the expected state.
        :type timeout: integer
        :param timeout: allowed time in seconds to reach the expected state
        :raise: raises TestEquipmentException (error code, error message) in case of failure.
        """
        self.get_logger().info(
            "Check call state is %s before timeout %d seconds",
            state,
            timeout)

        timer = timeout
        (err, current_state, msg) = W.GetCallControlStatus(self.get_root())
        self.__error_check(err, msg)

        while (timer > 0) and (current_state != state):
            time.sleep(1)
            (err, current_state, msg) = W.GetCallControlStatus(self.get_root())
            self.__error_check(err, msg)
            timer -= 1

        if current_state == state:  # Expected state has been reached
            self.get_logger().info("Call state is %s !", state)
        else:
            if blocking:
                # Timeout to reach desired state
                msg = "Check Call state to %s timeout !" % state
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                # Timeout to reach desired state (Test failed no TestEquipmentException raised)
                self.get_logger().info("Check Call state to %s timeout !", state)

    def check_call_connected(self, timeout=0, blocking=True):
        """
        Checks that call is established between equipment test and DUT before
        timeout seconds. If timeout is <= 0, only one test is performed.
        :type timeout: integer
        :param timeout: allowed time in seconds to establish the call
        :raise: raises TestEquipmentException (error code, error message) in case of failure.
        """
        self.check_call_state("CONN", timeout, blocking=True)

    def check_call_idle(self, timeout=0, blocking=True):
        """
        Checks that call is finished (idle) between equipment test and DUT before
        timeout seconds. If timeout is <= 0, only one test is performed.
        :type timeout: integer
        :param timeout: allowed time in seconds to establish the call
        :raise: raises TestEquipmentException (error code, error message) in case of failure.
        """
        self.get_logger().info(
            "Check call is idle before timeout %d seconds",
            timeout)

        timer = timeout
        idle = self.is_voice_call_idle()

        while (timer > 0) and not idle:
            time.sleep(1)
            idle = self.is_voice_call_idle()
            timer -= 1

        if idle:  # Expected state has been reached
            self.get_logger().info("Call state is idle !")
        else:
            # Fail
            if blocking:
                msg = "Check Call state to idle timeout !"
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                # TIMEOUT to reach desired state (Test failed no TestEquipmentException raised)
                self.get_logger().info("Check Call state to idle timeout !")

    def voice_call_network_release(self):
        """
        Wraps to VoiceCallNetworkRelease driver function
        :raise TestEquipmentException: failed to call VoiceCallNetworkRelease
        driver function
        :rtype: integer
        :return: the error code of the driver function
        """
        (err, msg) = W.VoiceCallNetworkRelease(self.get_root())
        self.__error_check(err, msg)
