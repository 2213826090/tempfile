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

:organization: INTEL OTC QA Automation
:summary: This file implements the UC base class
:since: 9/10/2015
:author: rcstamat
"""

import time

from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from UtilitiesFWK.Utilities import Global


class HDKAndroid(IntelDeviceBase):
    """
        Definition for the HDK generic model instance that works without Android system running.
        Only levels active are Android HAL and below.
    """

    def __init__(self, config, device_config):
        """
        Constructor

        :type  phone_name: str
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """
        IntelDeviceBase.__init__(self, config, device_config)
        self.switch_on()

    def device_alive(self, tries):
        status = self.get_state()
        while (status != "alive"):
            self.get_logger().error("Device status: " + status)
            if tries > 0:
                tries -= 1
                time.sleep(1)
            else:
                return Global.FAILURE
            status = self.get_state()
        return Global.SUCCESS

    def stop_android(self):
        status, msg = self.run_cmd("adb root", 10, force_execution=True)
        if (self.device_alive(3)) == Global.FAILURE:
            self.get_logger().error("Adb device did not return a device after adb root")
        status, msg = self.run_cmd("adb remount", 10, force_execution=True)

        if (self.device_alive(3)) == Global.FAILURE:
            self.get_logger().error("Adb device did not return a device after adb remount")
        status, msg = self.run_cmd("adb shell stop", 10, force_execution=True)

    def switch_on(self, boot_timeout=10, settledown_duration=None, simple_switch_mode=False):
        message = ""
        self.state = self.get_state()
        if self.state != "alive":
            self.get_logger().error("Adb status is not alive: " + self.state)
            output, message = self._run_adb_cmd("adb kill-server", 5, True)

        state = ""
        boot_time_elapsed = 0
        self.state = self.get_state()
        while self.state != "alive":
            if boot_time_elapsed > boot_timeout:
                self._is_device_connected = False
                status = Global.FAILURE
                message = "Can't establish start up connection with the device"
                self.get_logger().error(message)
                return status, message

            # retry connection every 2 seconds
            time.sleep(2)
            boot_time_elapsed = boot_time_elapsed + 2
            self.state = self.get_state()

        self.stop_android()
        self._is_device_connected = True
        status = Global.SUCCESS
        return status, message

    def get_state(self):
        """
        Adapted from AndroidDeviceBase
        """
        state = "unknown"
        try:
            retry = 2
            try_index = 0
            # Implement a retry mechanism as on some benches as on first
            # run of adb (if shutdown previously), it always return unknown
            while try_index < retry and state != "alive":
                try_index += 1
                output = self.run_cmd("adb get-state", 10, True)
            if output[0] != Global.FAILURE:
                state = (output[1].strip()).lower()
                if state.endswith("device"):
                    state = "alive"
                    if self.run_cmd("adb shell echo", 10, True)[0] \
                            == Global.FAILURE:
                        state = "unknown"
        except Exception as error:  # pylint: disable=W0703
            self.get_logger().debug("get_state: error happened: %s" % str(error))
            state = "unknown"
        return state

    def get_apk_version(self, package_name):
        """
        Only HAL active, no apks running, no need for this
        """
        return None

    def switch_off(self):
        """
        Device always on, no need for switch off
        """
        return Global.SUCCESS
