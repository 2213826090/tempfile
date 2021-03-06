"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL NDG SW
:summary: This file implements the test Step for getting the direction of a GPIO
:since 12/09/2014
:author: dpierrex
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.TestStep.Device.LowSpeedIO.GPIO.GpioBase import GpioBase


class GetGpioDirection(GpioBase):

    def run(self, context):
        """
        Runs the test step
        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Mandatory parameters
        gpio_number = str(self._pars.gpio_number)

        # Where the information will be stored into the context
        save_as = self._pars.save_as

        self._logger.info("read_gpio {0} direction".format(gpio_number))

        status, output = self._gpio_api.get_gpio_direction(gpio_nbr=gpio_number)

        if not status:
            self._raise_config_exception(AcsConfigException.OPERATION_FAILED, "error occured : %s" % output)

        # We have the value, let's save it into the context
        context.set_info(save_as, str(output))
