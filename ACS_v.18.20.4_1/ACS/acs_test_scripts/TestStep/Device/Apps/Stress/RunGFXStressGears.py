"""
@summary: This file implements a Test Step to run a shell script that invokes
a lightweight graphics stress app called Gears.
This renders some spinning gears on the screen.
Prerequisites: the following scripts and binaries must be located in the path specified by SCRIPTS_PATH.
    GearsES2_20110906.apk
    GearsES2_v1.sh
    killer.sh
    process
These files can be found in this tarball in Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/gears/GearsES2_v1.tgz.
Android only -- I don't know of a similar executable for Windows.
@since 27 June 2014
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

class RunGFXStressGears(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunGfxStressGears: Run")

        self.system_api = self._device.get_uecmd("System")

        script = self._pars.scripts_path + "/GearsES2_v1.sh"
        args = str(self._pars.duration)

        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 120)

        self._logger.info("RunGfxStressGears: Done")
