"""
@summary: This file implements a Test Step to run a shell script that invokes Linpack.
The Linpack benchmark is a measure of a computer's floating-point rate of execution.
This benchmark computes the solution of a system of N linear equations with N unknowns.
There are several versions the Linpack benchmark, differing in size, numerical precision, and rules.
This version is an Intel-customized version. The original Linpack was intended for systems with
a distributed memory.
Prerequisites: scripts and binaries from linpack_files.tgz must be located in the path specified by SCRIPTS_PATH.
This tarball can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/linpack/linpack_files.tgz.
@since 3 July 2014
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

class RunLinpack(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunLinpack: Starting")

        cmd = "adb shell cd %s; sh linpack.sh %d lininput_xeon32_512MB"%(self._pars.scripts_path, self._pars.duration)

        verdict, output = self._device.run_cmd(cmd, (self._pars.duration+1)*60)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "RunLinpack: run_cmd failed with message '%s'"%output)
        self._logger.info("RunLinpack: Done")
