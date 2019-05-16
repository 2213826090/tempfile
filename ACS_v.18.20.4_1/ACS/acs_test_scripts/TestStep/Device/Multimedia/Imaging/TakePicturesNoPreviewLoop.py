"""
@summary: Repeatedly take pictures without preview for the specified amount of time. This will take several images(NUM_IMAGES) at one time,
            and wait for DELAY_BETWEEN_SHOTS until next shot. Note that captured image size depends on WIDTH and HEIGHT.
            Because of this reason, EXPECTED_IMAGE_SIZE is required for verification of operation. The image size can be get from TAKE_RAW_IMAGE_NO_PREVIEW TestStep
            PREREQUISITES:
                Android: This requires v4l2n and take_pictures_no_preview.sh to be installed.
                         The app can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/nopreview_imaging.
                         The script can be found in ACS repo at acs_test_scripts/Lib/ShellScripts/Android/Imaging/take_pictures_no_preview.
                Windows: Device-side script not yet created. We don't yet know of a Windows application that will allow us to take pictures without preview.
@since 5 September 2014
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

class TakePicturesNoPreviewLoop(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("TakePicturesNoPreviewLoop: Run")

        runtime=self._pars.duration

        self.system_api=self._device.get_uecmd("System")
        self.file_api=self._device.get_uecmd("File")

        basename="take_pictures_no_preview"
        # To make this OS-independent, look for different possible filenames.
        # If anyone enables this in Windows, I assume they will create a DOS batch file to do what take_pictures_no_preview.sh does.
        # Add to this list if required for any other OS.
        possible_extensions=[".sh", ".bat"]
        for ext in possible_extensions:
            # uecmd or underlying code should take care of changing path delimiters as needed
            script=self._pars.scripts_path+"/"+basename+ext
            found=self.file_api.exist(script)
            if found:
                break
        if not found:
            msg="TakePicturesNoPreviewLoop: Could not find a "+basename+".* script."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        args="{0} {1} {2} {3} {4} {5}".format(runtime, self._pars.delay_between_shots, self._pars.num_images, self._pars.width, self._pars.height, self._pars.expected_image_size)

        '''Adding 30 seconds to run_cmd timeout as a buffer in case the script happens to go a little longer than expected.
        It can go as long as 5 minutes past the desired run time, if it happens to start a new iteration just before the
        time is up.'''
        '''run_shell_executable doesn't return anything, but will raise an exception if it fails'''
        self.system_api.run_shell_executable(script, args, io_redirection=2, timeout=runtime*60+30)

        self._logger.info("TakePicturesNoPreviewLoop: Done")


