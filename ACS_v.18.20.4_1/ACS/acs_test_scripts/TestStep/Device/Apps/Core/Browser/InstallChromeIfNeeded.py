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
:summary: This file implements a Test Step to uninstall Chrome application from the device
:since: 09/03/2015
:author: tchourrx
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from acs_test_scripts.TestStep.Device.System.AppMgmt.InstallApp import InstallApp
from Device.DeviceManager import DeviceManager
from UtilitiesFWK.Utilities import str_to_bool



class InstallChromeIfNeeded(InstallApp):

    """
    Uninstall Chrome application from the device
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        chromeInstall = DeviceManager().get_device_config("PHONE1").get("ChromeInstall", "False")
        if not str_to_bool(chromeInstall):
            self._logger.info("No Chrome installation asked")
            return

        timeout_60 = 60
        timeout_5 = 5

        self._device.run_cmd("adb root", timeout_5)
        self._device.run_cmd("adb disable-verity", timeout_60)
        self._device.reboot()
        self._device.run_cmd("adb root", timeout_5)
        self._device.run_cmd("adb remount", timeout_60)
        _, output = self._device.run_cmd("adb shell pm list package | grep -i chrome", timeout_60)
        if output:
            package_name = output.splitlines()[0]
            self._logger.debug("chrome package name: %s" % package_name.split(':')[-1])
            _, output = self._device.run_cmd("adb shell pm dump %s |grep Path" % package_name.split(':')[-1], timeout_60)
            package_path = output.splitlines()[0]
            self._logger.debug("chrome package path: %s" % package_path.split('=')[-1])
            self._device.run_cmd('adb shell rm -rf %s' % package_path.split('=')[-1], timeout_5)
            self._device.reboot()

            _, output = self._device.run_cmd("adb shell pm list package | grep -i chrome", timeout_60)
            if output and "chrome" in output:
                self._logger.warning("Failed to uninstall Chrome application")
            else:
                self._logger.info("Chrome application successfully uninstalled ")
        else:
            self._logger.info("Chrome not yet installed")

        InstallApp.run(self, context)

        # Grant runtime permissions to chrome
        _, output = self._device.run_cmd("adb shell pm list package | grep -i chrome", timeout_60)
        if output:
            chrome_package_name = output.splitlines()[0].split(':')[-1]
            # grant runtime permissions to chrome
            self._device.grant_runtime_permissions(chrome_package_name)
