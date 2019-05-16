"""
@summary: Take a single picture without preview and download it to host. This returns a file path on the host pc (RAW_IMAGE_PATH) and file size(RAW_IMAGE_SIZE)
            PREREQUISITES:
                This requires v4l2n to be installed.
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
import os
import time
import tempfile
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class TakeRawImageNoPreview(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("TakeRawImageNoPreview: Run")

        self.camera_api=self._device.get_uecmd("Camera")

        # Create the target download directory path under temporary directory
        temp_path = tempfile.gettempdir()
        image_download_dir = os.path.join(temp_path, "TakeRawImageNoPreview_{0}".format(time.strftime("%Y-%m-%d_%Hh%M.%S", time.localtime())))
        self._logger.debug("make {0} folder".format(image_download_dir))
        try:
            os.makedirs(image_download_dir)
        except Exception, e:
            msg = "Failed to create local download directory, \"{0}\": {1}".format(image_download_dir, e)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        raw_filename = self.camera_api.take_raw_image_no_preview(self._pars.target_app_path, image_download_dir, self._pars.width, self._pars.height)

        raw_filepath = os.path.join(image_download_dir, raw_filename)
        if not os.path.exists(raw_filepath):
            msg="TakeRawImageNoPreview: Failed to capture an image. Could not find a {0} under {1}".format(raw_filename, image_download_dir)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        raw_filesize = os.path.getsize(raw_filepath)

        self._logger.info("Saved raw image from camera to {0} : width={1}, height={2} size={3}".format(raw_filepath, self._pars.width, self._pars.height, raw_filesize))

        # Write raw_image_path and  raw_image_size to context so that it can be used by other TestStep. i.e. TakePicturesNoPreviewLoop
        context.set_info(self._pars.raw_image_path, raw_filepath)
        context.set_info(self._pars.raw_image_size, raw_filesize)

        self._logger.info("TakeRawImageNoPreview: Done")


