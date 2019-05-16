"""
@summary: This file implements a Test Step to run the "CacheMem" stress.
  This consists of launching a memory stress test called cachebench, and
  simultaneously doing some file operations like unpacking a tarball to
  interfere with memory paging.  This activity is controlled by shell
  scripts that are run on the DUT in order to have better control and minimize
  the ADB traffic, so it is easily encapsulated by this single test step.
  PREREQUISITES:
    Android: the following scripts and binaries must be located in the path specified by SCRIPTS_PATH.
            cachebench
            cacheMem.sh
            memorytest.sh
            run_em.sh
            killer.sh
            killcacheMem.sh
            process
            memorytest.tar.gz
        These files can be found in this tarball in Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/CacheMem/cacheMem_v2.3.tar.gz
    Windows: the binary and device-side scripts are not yet created.
@since 23 June 2014
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

class RunCachemem(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunCachemem: Run")

        runtime = self._pars.duration
        if runtime < 5:
            runtime = 5
            self._logger.info("RunCachemem: INCREASING RUN TIME to 5 minutes, since this is the minimum amount of time it takes to do a single iteration")

        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename = "cacheMem"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a DOS batch file to do what cacheMem.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg = "RunCachemem: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args = "{0} {1}".format(runtime, self._pars.instances_file_ops)

        '''Adding 5.5 minutes to run_cmd timeout as a buffer in case the script happens to go a little longer than expected.
        It can go as long as 5 minutes past the desired run time, if it happens to start a new iteration just before the
        time is up.'''
        '''run_shell_executable doesn't return anything, but will raise an exception if it fails'''
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=runtime*60 + 330)

        self._logger.info("RunCachemem: Done")
