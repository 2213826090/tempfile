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

:organization: INTEL PEG-SVE-DSV
:summary: This file implements a Test Step to set screen properties.
:since: 28 July 2014
:author: Jongyoon Choi
"""
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class GetHDMIConnectionStatus(DeviceTestStepBase):
    SINGLE_RUN_STOP_ON_FAIL = 1
    SINGLE_RUN_CONTINUE_ON_FAIL = 2
    MULTI_RUN_STOP_ON_FAIL = 3

    """
    Set screen properties.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._api = self._device.get_uecmd("PhoneSystem")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("GetHDMIConnectionStatus: Run")

        self.display_api = self._device.get_uecmd("Display")

        if self._pars.check_interval != GetHDMIConnectionStatus.MULTI_RUN_STOP_ON_FAIL:
            return_value = self.display_api.is_hdmi_connected()
            self._logger.debug(return_value)
            if return_value == False and self._pars.stop_on_fail == True:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to find HDMI")

            context.set_info(self._pars.is_hdmi_connected, return_value)
        else:
            # Get Parameters
            test_duration = self._pars.duration * 60
            start_time = time.time()

            #Start checking loop
            while time.time()-start_time < test_duration:
                return_value = self.display_api.is_hdmi_connected()

                if return_value == True:
                    self._logger.debug("HDMI connected")
                else:
                    raise DeviceException(DeviceException.OPERATION_FAILED, "HDMI disconnected")

                time.sleep(self._pars.check_interval)

            context.set_info(self._pars.is_hdmi_connected, True)

        self._logger.info("GetHDMIConnectionStatus: Done")
