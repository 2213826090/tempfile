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


class GetSocwatchPackage(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")

        socwatch_enabling = DeviceManager().get_device_config("PHONE1").get("enableSocwatch", "False")
        if not str_to_bool(socwatch_enabling):
            self._logger.info("No Socwatch informations needed")
            context.set_info(self._pars.need_install, 0)
            return

        _, version = self._device.run_cmd('adb shell find /system/lib/modules /lib/modules/ -iname socwatch*.ko 2&> /dev/null | grep -o -e "[0-9]_[0-9]"', 5)

        if not version:
            self._logger.info("No Socwatch version found on the device")
            context.set_info(self._pars.need_install, 0)
            return

        version = version.splitlines()[0]
        self._logger.debug("socwatch version catched: %s" % version)

        context.set_info(self._pars.package, "socwatch_" + version + ".tgz")
        context.set_info(self._pars.need_install, 1)

        self._logger.info(self._pars.id + ": Done")
