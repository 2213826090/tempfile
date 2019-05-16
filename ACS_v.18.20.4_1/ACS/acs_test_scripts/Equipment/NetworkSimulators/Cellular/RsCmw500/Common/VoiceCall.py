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
:summary: voice call implementation for RS CMUW500 cellular network simulator
:since: may 07th 2015
:author: Martin Brisbarre
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.IVoiceCall2G import IVoiceCall2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Common.Audio import Audio as AudioCommon


class VoiceCall(AudioCommon):
    """
    Voice Call 2G implementation for RS CMW500
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        AudioCommon.__init__(self, visa)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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

    def wait_for_state(self, state, call_setup_time):
        """
        This function is an abstract layer to align with the wait_for_state VoiceCall UeCmd used in APx585-based Audio Usecase

        :type state: str
        :param state: expected CS state to be reached before call_setup_timeout
        :type call_setup_time: int
        :param call_setup_time: allowed time in seconds to reach the expected state

        :rtype: bool
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_call_connected(self, timeout=0, blocking=True):
        """
        Checks that call is established between equipment test and DUT before
        timeout seconds. If timeout = 0, only one test is performed.
        :type timeout: integer
        :param timeout: allowed time in seconds to establish the call
        :rtype: bool
        :return: boolean to indicate if call was established (true) or not (false)
        """
        return self.check_call_state("CEST", timeout)

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
        idle = self.check_call_state("REL")

        while (timer > 0) and not idle:
            time.sleep(1)
            idle = self.check_call_state("REL")
            timer -= 1

        if idle:  # Expected state has been reached
            self.get_logger().info("Call is idle !")
        else:  # Fail
            if blocking:
                msg = "Check Call state to idle timeout !"
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                # TIMEOUT to reach desired state (Test failed no TestEquipmentException raised)
                self.get_logger().info("Check Call state to idle timeout !")

    def is_voice_call_idle(self):
        """
        Tests if voice call is idle (disconnected).
        :rtype: None
        :raise: raises DeviceException (error code, error message) in case of failure.
        """
        self.get_logger().info("Check if voice call is idle")

        status = self.check_call_state("REL")

        if status is False:
            msg = "Call was not released correctly, state is %s !" % status
            self.get_logger().error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
        else:
            self.get_logger().info("Voice call was correctly released!")

    def is_voice_call_connected(self, connection_time=0, blocking=True):
        """
        Tests if the voice call is established between equipment and DUT for
        connection_time seconds. If connection_time is = 0, only one test is performed.
        :type connection_time: integer
        :param connection_time: expected minimum connection time in sec
        :rtype: None
        :raise: raises DeviceException (error code, error message) in case of failure.
        """
        self.get_logger().info(
            "Check if voice call connected for %d seconds",
            connection_time)

        timer = 0
        status = True

        while (timer < connection_time) and (status is True):
            status = self.check_call_connected()
            time.sleep(1)
            timer += 1

        if status is False:
            msg = "Voice call interrupted after %d seconds, state is %s !" % (timer, status)
            self.get_logger().error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, msg)
        else:
            self.get_logger().info(
                "Voice call still connected after %d seconds!",
                connection_time)

    def dial(self, number_to_call):
        """
        This function is an abstract layer to align with the dial VoiceCall UeCmd used in APx585-based Audio Usecase

        :type number_to_call: str
        :param number_to_call: number to call (MSISDN)
        """
        self.mt_originate_call()

    def answer(self):
        """
        Stub to align with APx585 audio bench
        """
        pass

    def check_state(self, state):
        """
        This function is an abstract layer to align with the check_state VoiceCall UeCmd used in APx585-based Audio Usecase

        :type state: str
        :param state: expected CS state

        :rtype: bool
        """
        return self.wait_for_state(state, 0)

    def release(self):
        """
        This function is an abstract layer to align with the release VoiceCall UeCmd used in APx585-based Audio Usecase
        """
        self.voice_call_network_release()
