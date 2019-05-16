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

:summary: This file implements a Test Step for initializing a network simulator
:author: jduran4x
:since 12/09/2014
:organization: INTEL QCTV
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase


class InitNetworkSimulator(EquipmentTestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        EquipmentTestStepBase.run(self, context)

        net_sim = self._equipment_manager.get_cellular_network_simulator(self._pars.eqt, visa=True)
        cell_tech = self._pars.feature.split(";")

        net_sim.set_cells_technology(cell_tech)
        net_sim.apply_bench_config(self._global_conf.benchConfig)
