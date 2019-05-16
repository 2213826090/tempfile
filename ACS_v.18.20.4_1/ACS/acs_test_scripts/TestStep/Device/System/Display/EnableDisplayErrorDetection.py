"""
@summary: This file does the enabling Display Register Checking.
@author: Santhosh Reddy Dubbaka
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
import time
from ErrorHandling.DeviceException import DeviceException

class EnableDisplayErrorDetection(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):

        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        self._logger.debug(self._pars.id +" : Run")
        DeviceTestStepBase.run(self, context)
        self._acs_displaychecks_module = self._device.get_device_module("DeviceChecksModule")

        pretest_check_results = self._acs_displaychecks_module.enable_tearing_effect_detection()
        if pretest_check_results == "FAIL":
            error_msg = "tearing effect detection is not enabled. "
            self._logger.debug(self._pars.id+" :"+error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED,self._pars.id+" :"+ error_msg)





