"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL CCG OPM PC&WTE
:summary: This file implements the Miscellaneous UEcmd for Android phone
:since: 2015/08/03
:author: ldesvigx
"""

from acs_test_scripts.Device.UECmd.Imp.Android.LLP.Misc.PhoneSystem import PhoneSystem as PhoneSystemLLP
from ErrorHandling.DeviceException import DeviceException


class PhoneSystem(PhoneSystemLLP):

    def set_display_brightness(self, brightness):
        """
        Sets the display brightness
        :type: brightness: int
        :param brightness: brightness to set, in percentage from 0 to 100

        :rtype: list
        :return: operation status & output log
        """
        self._logger.info("Setting screen brightness to : %s " % brightness)

        actual_brightness = None
        dpst_file = self._system_module.system_properties.dpst_file
        if dpst_file:
            cmd = "echo 'DSP DISABLE END' > %s && cat %s" % (dpst_file, dpst_file)

            output = self._exec("adb shell %s" % cmd)
            if output != "DPST Enabled:No":
                self._logger.error("Disabling DPST failed : %s" % output)

        output = self._internal_exec_v2(self._display_module,
                                        "setScreenBrightness",
                                        "--ei brightness %s " %
                                        brightness, is_system=True)

        if "actual_brightness" in output:
            actual_brightness = output["actual_brightness"]

        if dpst_file:
            cmd = "echo 'DSP ENABLE END' > %s && cat %s" % (dpst_file, dpst_file)
            output = self._exec("adb shell %s" % cmd)
            if output != "DPST Enabled:Yes":
                self._logger.error("Enabling DPST failed : %s" % output)

            self._exec("adb shell setprop coreu.dpst.aggressiveness 3")

        if not actual_brightness:
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR,
                                  "Failed to set brightness : '%s'" % output)
        return actual_brightness

    def set_screen_timeout(self, timeout):
        """
        Sets the screen timeout.

        :type timeout: int
        :param timeout: the screen timeout in seconds. Use 0 to set maximum delay

        :return: None
        """

        if not isinstance(timeout, int) or timeout < 0:
            self._logger.error("set_screen_timeout : Bad timeout value :%s", timeout)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Parameter timeout must be integer superior or equal to 0")

        if timeout == 0:
            self._logger.debug("set_screen_timeout maximum delay required. Set to 1800s")
            timeout = 1800

        self._logger.info("Set screen timeout to %s seconds" % timeout)

        function = "setScreenOffTimeout"
        cmd_args = "--ei timeout %s" % timeout
        self._internal_exec_v2(self._display_module, function, cmd_args, is_system=True)

    def set_brightness_mode(self, mode):
        """
        Sets the mode of the brightness

        :type mode: str
        :param mode: mode to set, 'manual' or 'automatic'
        """
        self._logger.info("Setting brightness mode to : %s " % mode)
        self._internal_exec_v2(self._display_module,
                               "setScreenBrightnessMode",
                               "--es mode %s" % mode, is_system=True)

    def set_silent_vibrate(self, silent_enable, vibrate_mode):
        """
        Set silent mode enable/disable and vibrate mode for phone

        :type silent_enable: str
        :param silent_enable: sound silent mode enable: True|False
        :type vibrate_mode: String
        :param vibrate_mode: vibrate mode: always|never|silent|notsilent

        """
        self._logger.info("Setting silent mode: %s, vibrate mode: %s" %
                          (silent_enable, vibrate_mode))

        function = "setSilentAndVibrateMode"
        cmd_args = "--es silent_mode_enable %s --es vibrate_mode %s" % (silent_enable, vibrate_mode)
        self._internal_exec_v2(self._ringtone_module, function, cmd_args, is_system=True)
