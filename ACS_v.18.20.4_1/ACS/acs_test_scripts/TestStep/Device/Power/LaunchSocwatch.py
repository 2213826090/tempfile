"""
PREREQUISITES:
Android:
    * socwatch_*.tgz must be installed (Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/socwatch/socwatch_1_5_4.tgz).
@since 13 November 2015
@author: tchourrx
@organization: INTEL SSG-SI-PNP

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from Device.DeviceManager import DeviceManager
from UtilitiesFWK.Utilities import str_to_bool


class LaunchSocwatch(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")

        # Delete all previous csv files saved into socwatch folder
        self._device.run_cmd('adb shell rm %s/*.csv' % self._pars.install_path, timeout=5)

        socwatch_enabling = DeviceManager().get_device_config("PHONE1").get("enableSocwatch", "False")
        if not str_to_bool(socwatch_enabling):
            self._logger.info("No Socwatch informations needed")
            return

        self._device.run_cmd('adb shell chmod 0755 %s/socwatch_run.sh' % self._pars.install_path, timeout=5)

        socperf_driver = self._device.get_socperf_driver()

        #Extract socwatch driver
        _, socwatch_driver = self._device.run_cmd('adb shell find /lib/modules/ -name socwatch*.ko | grep -o -e "socwatch[0-9]_[0-9].ko"', 5)
        socwatch_driver = socwatch_driver.splitlines()[0]
        self._logger.debug("socwatch driver catched: %s" % socwatch_driver)

        args = "{0} {1} {2} {3} {4} {5}".format(socperf_driver, socwatch_driver, self._pars.feature, self._pars.duration, self._pars.delay, self._pars.result_filename)

        self._device.run_cmd('adb shell cd %s; nohup sh socwatch_run.sh %s' % (self._pars.install_path, args), timeout=1, force_execution=True, wait_for_response=False)

        self._logger.info(self._pars.id + ": Done")
