"""
@summary: This file implements a Test Step to repeatedly changes the screen
    orientation for the amount of time specified, and restores the screen to
    auto-rotate mode at the end.  The sequence can be random or fixed.
    If RANDOM_SEQUENCE=False then it goes in this order:
        portrait, landscape, reverse portrait, reverse landscape.
    The amount of time between orientation changes will be a fixed value
    if CHANGE_INTERVAL_MIN=CHANGE_INTERVAL_MAX, or a randomly selected amount
    between CHANGE_INTERVAL_MIN and CHANGE_INTERVAL_MAX if they are not equal.
    PREREQUISITES:
        Installed application: orientation_change_loop.sh from
        Lib-ShellScripts-Android-Display-orientation_change
@since 2 September 2014
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

class RunOrientationChangeLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("RunOrientationChangeLoop: Run")

        runtime=self._pars.duration
        if self._pars.random_sequence==True:
            random_sequence=1
        else:
            random_sequence=0

        self.system_api=self._device.get_uecmd("System")
        self.file_api=self._device.get_uecmd("File")

        basename="orientation_change_loop"
        # To make this OS-independent, look for different possible filenames.
        # If anyone enables this in Windows, I assume they will create a PowerShell file to do what orientation_change_loop.sh does.
        # Add to this list if required for any other OS.
        possible_extensions=[".sh", ".ps1"]
        for ext in possible_extensions:
            # uecmd or underlying code should take care of changing path delimiters as needed
            script=self._pars.scripts_path+"/"+basename+ext
            found=self.file_api.exist(script)
            if found:
                break
        if not found:
            msg="RunOrientationChangeLoop: Could not find a "+basename+".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args="{0} {1} {2} {3}".format(runtime, random_sequence, self._pars.change_interval_min, self._pars.change_interval_max)

        '''Adding 5.5 minutes to run_cmd timeout as a buffer in case the script happens to go a little longer than expected.
        It can go as long as 5 minutes past the desired run time, if it happens to start a new iteration just before the
        time is up.'''
        '''run_shell_executable doesn't return anything, but will raise an exception if it fails'''
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=runtime*60+60)

        self._logger.info("RunOrientationChangeLoop: Done")
