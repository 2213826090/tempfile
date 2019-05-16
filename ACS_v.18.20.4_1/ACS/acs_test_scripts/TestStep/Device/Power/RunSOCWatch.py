"""
@summary: Run SocWatch for the specified amount of time to measure PM residency and IO band width.
This uses a device-side script to maintain the SocWatch execution for an extended amount of time without intervention from the host.
This is done to avoid adding traffic to the host-device connection (e.g. ADB for Android) throughout the test.
PREREQUISITES:
Android:
    * socwatch_*.tgz must be installed (Artifactory: acs_test_artifacts/CONCURRENCY/TESTS/socwatch/socwatch_1_5_4.tgz).
    * run_socwatch.sh must be located in the directory specified by SCRIPTS_PATH (from acs_test_scripts/Lib/ShellScripts/Android/System/socwatch, can use INSTALL_SCRIPTS_FROM_LIB).
Windows: Device-side script not yet created.
@since 20 April 2015
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

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import re

class RunSOCWatch(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")

        self.system_api = self._device.get_uecmd("System")

        basename = "run_socwatch"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a script to do what loopAudioPlayback.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".ps1"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self._device.get_uecmd("File").exist(script)
            if found:
                break
        if not found:
            msg = "run_socwatch: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        socperf_driver=self._device.get_socperf_driver()
        socwatch_driver=self._device.get_socwatch_driver()

        args = "{0} {1} {2} {3} {4}".format(self._pars.feature, self._pars.duration*60, self._pars.output, socperf_driver, socwatch_driver)

        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        """Adding a extra small amount of time to the timeout, in case something goes a little long"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 60)

        context.set_info(self._pars.saved_socwatch_result, self._pars.scripts_path + "/" + self._pars.output + ".csv")

        self._logger.info(self._pars.id + ": Done")
