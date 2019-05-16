"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: This file implement the research of the last device plugged on
:since: 03/10/2014
:author: dpierrex
"""
import re
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException


class FindLastPlugSd(DeviceTestStepBase):
    """
    Push file on device
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._file_api = self._device.get_uecmd("File")


    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        # get the dmesg and extract the sd insertion
        sd_name = self._file_api.get_last_sd_insert()
        if sd_name == "":
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "Unable to find the device")

        context.set_info(self._pars.save_as, sd_name)
