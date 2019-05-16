"""
@summary: This file implements a Test Step to run a shell script that invokes
Basemark and keeps it running for the specified duration.
Prerequisites: the contents of this tarball must be unpacked into the directory specified in SCRIPTS_PATH.
    basemarkes2v1_files.tgz
This can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/basemark.
@since 1 July 2014
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

class RunBasemark(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunBasemark: Starting")

        cmd = "adb shell cd %s; sh run_basemark_es20.sh %d"%(self._pars.scripts_path, self._pars.duration)

        #Added generous amount to timeout since the app may go over the desired time.
        verdict, output = self._device.run_cmd(cmd, (self._pars.duration+8)*60)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "RunBasemark: run_cmd failed with message '%s'"%output)
        self._logger.info("RunBasemark: Done")
