"""
@summary: This file implements a Test Step to capture a screen shot and save it.
@since 24 March 2015
@author: Thierry CHOURRY
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
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class CaptureScreenShot(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        if self._pars.filepath is not None:
            filepath = self._pars.filepath.rsplit("/", 1)[0]
            report_dir = self._device.get_report_tree()
            report_dir.create_subfolder(filepath)


    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        screenshot_path = os.path.join(self._device.get_report_tree().get_report_path(), self._pars.filepath)
        path = self._device.screenshot(filename=screenshot_path)
        if path is not None:
            self._logger.info("Screenshot saved in: %s" % path)
        else:
            raise Exception("Fail to save screenshot")
