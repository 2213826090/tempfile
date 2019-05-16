"""
@summary: This file checks for the status of the particular register values for any errors.
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

class CheckDisplayRegisters(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):

        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        self._logger.debug(self._pars.id +" : Test Run")
        DeviceTestStepBase.run(self, context)
        self._acs_displaychecks_module = self._device.get_device_module("DeviceChecksModule")

        if self._pars.type =="mipi" :
            mipi_intr_stat = self._acs_displaychecks_module.check_mipi_errors()
            if mipi_intr_stat == "FAIL":
                error_msg = "Mipi failures found. "
                self._logger.debug(self._pars.id+" :"+error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+" :"+error_msg)

        elif self._pars.type == "pipe_hung" :
            pipe_hung = self._acs_displaychecks_module.check_pipe_hang()

        #Checks for the HDMI(checking the pipe which is connected to hdmi) underflow
        elif self._pars.type =="hdmi_pipeunderflow" :
            hdmi_pipeunderflow = self._acs_displaychecks_module.check_hdmi_errors()

            if hdmi_pipeunderflow == "FAIL":
                error_msg = "hdmi pipe underflow occured. "
                self._logger.error(self._pars.id+" :"+error_msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, self._pars.id+" :"+error_msg)
        else :
            self._logger.error(self._pars.id+" : Type entered is not correct")
            raise DeviceException(DeviceException.OPERATION_FAILED,self._pars.id+ " : Type entered is not correct")





