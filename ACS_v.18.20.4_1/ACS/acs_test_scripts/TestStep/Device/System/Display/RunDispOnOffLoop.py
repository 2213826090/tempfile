"""
@summary: This file implements a Test Step to run the Disp_on_off which repeatedly does
power on and off the display for the test duration.
@author: Megha Cheboli
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


class RunDispOnOffLoop(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.phonesystem_api=self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")


    def run(self, context):

        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        self._logger.debug(self._pars.id +" : Test Run")
        DeviceTestStepBase.run(self, context)
        loop_count =1

        # Start the RunDispOnOffLoop Test control loop.
        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        while (time.time() < end_time):
            self._keyevent_api.scenario(["POWER_BUTTON"])
            time.sleep(5)
            self._keyevent_api.scenario(["POWER_BUTTON"])
            time.sleep(5)
            # Check for system UI
            systemui_ok =self.phonesystem_api.check_process("systemui")
            if systemui_ok :
                self._logger.debug(self._pars.id +" : Systemui is active")

            else:
                self._logger.error("%s : Systemui is not active,Error occured in loop %d."%(self._pars.id,loop_count))
                raise DeviceException(DeviceException.OPERATION_FAILED, "%s : Systemui is not active,Error occured in loop %d."%(self._pars.id,loop_count))
            loop_count+=1

        self._logger.debug(self._pars.id +" : Completed the test. ")














