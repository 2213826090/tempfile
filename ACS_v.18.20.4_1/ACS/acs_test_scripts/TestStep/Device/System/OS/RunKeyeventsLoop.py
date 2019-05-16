"""
@summary: (Android only) Runs a shell script that periodically injects keyevents for the specified amount of time.
The script uses keyevents that are unlikely to interfere with the operation of other apps that
might be running in the test.  Examples are the volume and backlight controls.  A shell script is
used, instead of driving each event through ADB, in order to reduce traffic on USB.
I do not know if there is something comparable for Windows, so for now this is written to support
only Android.  If it's practical on Windows, a device-side script could be written to do the same there
and this could be easily enhanced.
@since 30 July 2014
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

class RunKeyeventsLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunKeyeventsLoop: Run")

        runtime = self._pars.duration
        if runtime < 2:
            runtime = 2
            self._logger.info("RunKeyeventsLoop: INCREASING RUN TIME to 2 minutes, since this is the minimum amount of time it takes to do a single iteration")

        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename = "keyevents"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a DOS batch file to do what keyevents does.
        #Add to this list if required for any other OS.
        possible_extensions = ["", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg = "RunKeyeventsLoop: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args = "{0} {1}".format(runtime, self._pars.event_interval)

        '''Adding 1 minutes to run_cmd timeout as a buffer in case the script happens to go a little longer than expected'''
        '''run_shell_executable doesn't return anything, but will raise an exception if it fails'''
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=runtime*60 + 60)

        self._logger.info("RunKeyeventsLoop: Done")