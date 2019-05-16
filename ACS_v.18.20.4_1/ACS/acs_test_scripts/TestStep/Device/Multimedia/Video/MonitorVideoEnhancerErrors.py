"""
@summary: Set up log trigger messages to watch for errors resulting from video processing by VSP.
          Invoke this before running the application, and invoke CHECK_VSP_ERRORS afterwards.
related to VSP encoding.
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

class MonitorVideoEnhancerErrors(DeviceTestStepBase):

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info(self._pars.id + ":  Run")
        device_logger = self._device.get_device_logger()

        system_api = self._device.get_uecmd("System")

        # Add triglogs to look for error conditions
        video_enhancer_messages = self._device.get_video_enhancer_error_messages()
        system_api.start_log_monitoring(device_logger=device_logger, target_messages=video_enhancer_messages)

        self._logger.info(self._pars.id + ":  Done")
