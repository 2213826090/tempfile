"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the base step for logic analyzers.
:since: 2014-11-17
:author: emarchan

"""
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase


class LogicAnalyzerBase(EquipmentTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._log_an_params = global_conf.benchConfig.get_parameters(self._pars.eqt)


    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        EquipmentTestStepBase.run(self, context)
        self._log_an = self._equipment_manager.get_logic_analyzer(self._pars.eqt)

