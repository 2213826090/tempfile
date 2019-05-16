"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android SSG
:summary: This test step will flash a given .bin program on an STM board
:since:3/08/16
:author: mmaraci
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException

class StmFlash(EquipmentTestStepBase):
    """
    Class responsible for calling the flash method of the STM32F4xx board equipment
    """

    def run(self, context):
        EquipmentTestStepBase.run(self, context)
        self._api = self._equipment_manager.get_stm32f4(self._pars.eqt)
        output, err_output, return_code = self._api.flash_device(self._pars.flasher_path, self._pars.binary_name)
        if return_code != 0:
            err_msg = "Return code was {0} with output {1} and stderr {2}".format(return_code, output, err_output)
            raise TestEquipmentException(TestEquipmentException.COMMAND_LINE_ERROR, err_msg)