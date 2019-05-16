"""
@summary: This file implements a Test Step to run a shell script that invokes
a file system benchmark tool called IOZone.  This generates and measures a
variety of file operations.  A side effect of this is that it also adds
significant stress on the CPU and memory.
PREREQUISITES:
Android: the following scripts and binaries must be located in the path specified by SCRIPTS_PATH.
        iozone
        process
        killer.sh
        loopIOZone.sh
    These files can be found in Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/IOZone/iozone.tgz
Windows: the device-side scripts have not yet been created.
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
from ErrorHandling.DeviceException import DeviceException

class RunIOZone(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunIozone: Run")
        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename = "loopIOZone"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a DOS batch file to do what loopIOZone.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg = "RunIOZone: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args = "{0} {1}".format(self._pars.duration, self._pars.file_size)
        if isinstance(self._pars.target_devices, str):
            args = args + " " + self._pars.target_devices
        if isinstance(self._pars.target_devices, list):
            for dev in self._pars.target_devices:
                args = args + " " + dev

        """Adding significant timeout buffer because time to execute each loop can be quite large."""
        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 600)
        self._logger.info("RunIozone: Done")
