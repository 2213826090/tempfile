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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step which decrease cell power step by step
:author Martin Brisbarre
:since 03/09/2015
:organization: INTEL QCTV
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
import time


class NsDecreaseCellPowerByStep(EquipmentTestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        EquipmentTestStepBase.run(self, context)
        net_sim = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt, visa=True)

        current_power = float(self._pars.init_power)
        end_power = float(self._pars.end_power)
        while current_power > end_power:
            current_power -= float(self._pars.decrementation_step_power)
            if current_power >= end_power:
                current_power = end_power
            net_sim.get_cell().set_cell_power(current_power)
            time.sleep(self._pars.decrementation_step_timer)
