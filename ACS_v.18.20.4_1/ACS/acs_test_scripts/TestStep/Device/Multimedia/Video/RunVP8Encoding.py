"""
@summary:   Repeatedly do video conversion from YUV to VP8.  This uses a device-side
            script to maintain the encoding for an extended amount of time without intervention from the host.  This
            is done to avoid adding traffic to the host-device connection (e.g. ADB for Android) throughout the test.
            PREREQUISITES:
            Android: The following files must be in the directory specified in SCRIPTS_PATH:
                    new_va_encode
                    vp8_loop.sh
                These can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/vp8_encoding.
                Optionally, you may also provide a YUV video file specified in INPUT_YUV_FILE.  If this is "None" then
                new_va_encode will create its own YUV file to use.
            Windows: executable and device-side script not yet created.
@since 8 August 2014
@author: Stephen A Smith
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

class RunVP8Encoding(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")
        self.system_api = self._device.get_uecmd("System")
        self.file_api = self._device.get_uecmd("File")

        basename = "vp8_loop"
        #To make this OS-independent, look for different possible filenames.
        #If anyone enables this in Windows, I assume they will create a DOS batch file to do what vp8_loop.sh does.
        #Add to this list if required for any other OS.
        possible_extensions = [".sh", ".bat"]
        for ext in possible_extensions:
            #uecmd or underlying code should take care of changing path delimiters as needed
            script = self._pars.scripts_path + self._device.get_device_os_path().sep + basename + ext
            found = self.file_api.exist(script)
            if found:
                break
        if not found:
            msg = self._pars.id + ": Could not find a " + basename + ".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #Do we have optional input yuv file?
        if self._pars.input_yuv_path is None:
            args = '%d'%(self._pars.duration)
        else:
            args = '%d %s'%(self._pars.duration, self._pars.input_yuv_path)
        """Adding a extra small amount of time to the timeout, in case something goes a little long"""
        """run_shell_executable doesn't return anything, but will raise an exception if it fails"""
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=self._pars.duration*60 + 120)
        self._logger.info(self._pars.id + ": Done")
