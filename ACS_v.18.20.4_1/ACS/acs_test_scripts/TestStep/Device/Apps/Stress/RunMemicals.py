"""
@summary: This file implements a Test Step to run a shell script that invokes an
memory stress test called "memicals".
Prerequisites: the following scripts and binaries must be located in the path
specified by SCRIPTS_PATH.
    memicals
    runMemicals.sh
These files can be found in this tarball in Artifactory:
    acs_test_artifacts/CONCURRENCY/TESTS/memicals/memicals.tgz.
@since 25 September 2014
@author: Jongyoon Choi
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
import math
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class RunMemicals(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunMemicals: Run")

        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename="run_test"
        #To make this OS-independent, look for different possible filenames.
        # If anyone enables this in Windows, I assume they will create a DOS batch file to do what runMemicals.sh does.
        #Add to this list if required for any other OS.
        possible_extensions=[".sh", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg="RunMemicals: Could not find a "+basename+".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        number_of_tests = self._pars.fastrandom + self._pars.lfsr + self._pars.randomaddr + self._pars.switchaddr + self._pars.walkingbits
        duration_per_test = math.trunc(self._pars.duration / number_of_tests)

        args = "{0} {1} {2} {3} {4} {5} {6}".format(duration_per_test, number_of_tests, self._pars.fastrandom, self._pars.lfsr, self._pars.randomaddr, self._pars.switchaddr, self._pars.walkingbits)

        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60+60)

        self._logger.info("RunMemicals: Done")
