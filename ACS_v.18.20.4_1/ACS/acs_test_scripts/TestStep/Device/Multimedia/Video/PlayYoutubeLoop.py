"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL PEG SVE DSV
:summary: Repeatedly play a Youtube video clip for the specified amount of time.
:since: 07/14/14
:author: jongyoon
"""

from UtilitiesFWK.Utilities import Global
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class PlayYoutubeLoop(DeviceTestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("PlayYoutubeLoop: Run")

        self.system_api = self._device.get_uecmd("System")

        basename = "loopYoutubePlayback"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a script to do what loopYoutubePlayback.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".ps1"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + "/" + basename + ext
            found = self._device.get_uecmd("File").exist(script)
            if found:
                break
        if not found:
            msg = "PlayYoutubeLoop: Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args = str(self._pars.duration)

        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        """Adding a extra small amount of time to the timeout, in case something goes a little long"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 180)

        self._logger.info(self._pars.id + ": Done")
