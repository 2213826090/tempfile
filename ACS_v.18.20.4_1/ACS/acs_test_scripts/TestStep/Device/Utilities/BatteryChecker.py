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

:organization: INTEL PnP
:summary: This file implements a Test Step base class PnP
:since:09/06/2015
:author: fmartin
"""


import re
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from Device.DeviceManager import DeviceManager
from UtilitiesFWK.Utilities import str_to_bool


class BatteryChecker(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._device_manager = DeviceManager()
        self._low_threshold = int(self._pars.get_attr("low_threshold"))
        self._required_level = int(self._pars.get_attr("required_level"))
        self._battery_infos = dict()

    def update_battery_infos(self):
        cmd_user = "adb shell dumpsys battery"
        status, logcat_output = self._device.run_cmd(cmd_user, self._device.get_uecmd_timeout(), force_execution=True,
                                                     wait_for_response=True, silent_mode=True)

        for match in re.finditer(r'^\s+(.*?):\s*(.+)$', logcat_output, re.MULTILINE):
            self._battery_infos[match.group(1)] = match.group(2)

    def get_current_battery_level(self):
        self.update_battery_infos()

        if 'level' in self._battery_infos and 'scale' in self._battery_infos:
            return 100 * int(self._battery_infos['level']) / int(self._battery_infos['scale'])
        else:
            raise DeviceException(DeviceException.CRITICAL_FAILURE, "Unable to read battery service information.")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info("Getting the current battery capacity...")

        device_plug = self._device_manager.get_device_config(self._pars.device).get("OnBattery", "False")
        self._logger.info("device plugged on battery: %s" % device_plug)
        if device_plug is not None and not str_to_bool(device_plug):
            self._logger.warning("Device plugged on Power Supply and in Low battery mode, can't wait charging")
            return

        current_level = self.get_current_battery_level()

        if current_level < self._low_threshold:
            self._logger.info("Low battery level detected... ({}%)".format(current_level))

            if 'status' in self._battery_infos and int(self._battery_infos['status']) == 2:
                self._logger.info("Battery is charging...")

                while current_level < self._required_level:
                    if 'status' in self._battery_infos and int(self._battery_infos['status']) == 2:
                        self._logger.info("Waiting for required ({0}%) battery level... ({1}% reached)"
                                          .format(self._required_level, current_level))
                        time.sleep(60)
                        current_level = self.get_current_battery_level()
                    else:
                        raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                              "Battery isn't charging or battery service unreachable.")

                cmd_user = "adb shell cat /proc/cmdline"
                status, output = self._device.run_cmd(cmd_user, self._device.get_uecmd_timeout(),
                                                      force_execution=True, wait_for_response=True,
                                                      silent_mode=True)

                # if the device started with battlow, we have reboot to flush this state:
                if "battlow" in output:
                    self._device.reboot()

            else:
                raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                      "Battery isn't charging or battery service unreachable.")

        self._logger.debug(self.ts_verdict_msg)