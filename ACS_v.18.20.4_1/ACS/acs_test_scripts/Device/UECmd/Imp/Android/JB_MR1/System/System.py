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
:summary: This file implements the System UEcmd for Android JB_MR1 device
:since: 2014/10/27
:author: msouyrix
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.System \
    import System as SystemCommon
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global, internal_shell_exec


class System(SystemCommon):

    """
    :summary: System UEcommands operations for JB_MR1 Android platform that handle duration modes retrieving
    """

    def get_mode_duration(self, mode):
        """
        Get the duration passed in the given mode (up time, idle time, ...) since device startup

        :type mode: str
        :param mode: mode we want to retrieve the duration. Accepted value:
        - up (for time)
        - idle (for idle time)
        - sleep (for deep sleep

        :rtype: int
        :return: time (in seconds) passed in the given mode.
        """
        allowed_mode = ["up", "idle", "sleep"]
        self._logger.debug("get_mode_duration: mode to treat {0}.".format(mode))
        if mode not in allowed_mode:
            msg = "get_mode_duration : invalid mode {0}. Allowed values: {1}".format(mode, allowed_mode)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Extract state duration from uptime command result and converts it into an operation evaluable
        # by python (uptime result in human readable format: hh:mm:ss)
        cmd_extract = "adb shell uptime | sed -e 's/.*{0} time:[^:]*\(..\):\(..\):\(..\).*$/\\1*3600+\\2*60+\\3/' |"\
            .format(mode)
        cmd_extract += "sed 's/+0/+/g'"
        self._logger.debug("get_mode_duration: extraction command: {0}".format(cmd_extract))

        verdict, str_time = internal_shell_exec(cmd_extract, timeout=60, silent_mode=True)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "get_mode_duration: unable to run extraction command")

        # Evaluate generated operation to get the duration in seconds instead of human readable format
        self._logger.debug("get_mode_duration: string to eval: {0}".format(str_time))
        duration_result = eval(str_time)
        self._logger.info("get_mode_duration: mode: {0}, duration: {1}".format(mode, duration_result))

        return duration_result
