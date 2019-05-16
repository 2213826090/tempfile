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
:summary: This file implements the Vibrator UEcmd for Android device
:since: 28/01/2014
:author: mmorchex
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.System.IVibrator import IVibrator
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

class Vibrator(BaseV2, IVibrator):

    """
    :summary: Vibrator UEcommands operations for Android platforms.
    """

    @need('vibrator')
    def __init__(self, device):
        """
        Constructor.
        """

        BaseV2.__init__(self, device)
        IVibrator.__init__(self, device)

        self._logger = device.get_logger()

        self._target_class = "acscmd.vibrator.VibratorModule"

    def start_vibrator_for_duration(self, duration):
        """
        Start vibrator for a given duration
        :type duration: int
        :param duration: duration of the vibration in milliseconds
        :rtype: None
        :return: None
        """
        self._logger.info("Start vibrator for %s ms", duration)
        target_method = "startVibratorForDuration"
        cmd_args = "--ei duration %s " % str(duration)
        self._internal_exec_v2(self._target_class, target_method, cmd_args)

    def start_vibrator(self):
        """
        Start vibrator without a timeout
        :rtype: None
        :return: None
        """
        self._logger.info("Start vibrator")
        cmd = "adb shell echo 1 > /sys/devices/pci0000:00/0000:00:17.0/vibrator"
        self._exec(cmd)

    def stop_vibrator(self):
        """
        Stop the vibrator
        :rtype: None
        :return: None
        """
        self._logger.info("Stopping vibrator")
        cmd = "adb shell echo 0 > /sys/devices/pci0000:00/0000:00:17.0/vibrator"
        self._exec(cmd)
