"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This module implements a step to get sensor data
@since: 26/11/14
@author: kturban
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class GetSensorData(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        sensor_api = self._device.get_uecmd("Sensor")
        duration = self._pars.duration if self._pars.duration > 0 else None
        output, posx, posy, posz = sensor_api.get_sensor_data(sensor=self._pars.sensor,
                                                              information=self._pars.mode,
                                                              duration=duration)
        msg = "Output: {3} - Sensor data : ({0},{1},{2})".format(posx, posy, posz, output)

        self._logger.info(msg)
        self._ts_verdict_msg = msg
        self._context.set_info(self._pars.save_as, dict(x=posx,
                                                        y=posy,
                                                        z=posz))
