"""
@summary: This file implements a Test Step to run a shell script that invokes
Bonnie++ and keeps it running for the specified duration.  It uses a
script that is run on the DUT in order to minimize the ADB traffic.
Prerequisites: the following scripts and binaries must be located in the path specified by SCRIPTS_PATH:
    bonnie++
    killer.sh
    process
    loopBonnie++.sh
These files can be found in this tarball in Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/bonnie_plus_plus/bonnie_plus_plus.tgz.This activity is controlled by shell
@since 26 June 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global

class RunBonniePlusPlus(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunBonniePlusPlus: Starting")

        cmd = "adb shell cd %s; sh ./loopBonnie++.sh %d %d"%(self._pars.scripts_path,self._pars.duration,self._pars.size)
        if isinstance(self._pars.target_devices, str):
            cmd = cmd + " " + self._pars.target_devices
        if isinstance(self._pars.target_devices, list):
            for dev in self._pars.target_devices:
                cmd = cmd + " " + dev

        """Adding generous amount to timeout since Bonnie++ can take much longer than expected to run a given iteration.
        Our shell script tries to account for this, but there's no guarantee we'll end when we're supposed to."""
        verdict, output = self._device.run_cmd(cmd, (self._pars.duration+30)*60)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "RunBonniePlusPlus: run_cmd failed")
        self._logger.info("RunBonniePlusPlus: Done")
