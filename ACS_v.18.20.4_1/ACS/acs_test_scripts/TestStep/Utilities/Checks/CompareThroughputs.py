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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

@summary: This file implements a Test Step for comparing measured and expected throughputs
@author: jduran4x
:since 12/09/2014
@organization: INTEL QCTV
"""

from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException

UNITS = ["bits/sec", "kbits/sec", "mbits/sec", "gbits/sec", "tbits/sec"]


class CompareThroughputs(TestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        TestStepBase.run(self, context)
        # ensure both units are ?bits/sec and not ?bps
        self._pars.value_unit = self._pars.value_unit.lower().replace("bps", "bits/sec")
        self._pars.target_unit = self._pars.target_unit.lower().replace("bps", "bits/sec")

        self._logger.info("checking measured value (%.2f %s) against expected target (%.2f %s)"
                          % (self._pars.value, self._pars.value_unit, self._pars.target, self._pars.target_unit))
        # make sure both values are in the same unit range
        value_unit_index = UNITS.index(self._pars.value_unit)
        target_unit_index = UNITS.index(self._pars.target_unit)
        if value_unit_index < target_unit_index:
            multiplier = 1000 * (target_unit_index - value_unit_index)
            self._pars.target *= multiplier
        elif value_unit_index > target_unit_index:
            multiplier = 1000 * (value_unit_index - target_unit_index)
            self._pars.value *= multiplier
        # else same range => nothing to do

        if self._pars.value < self._pars.target:
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR,
                                         "measured throughput is lower than target")
        self._logger.info("measured throughput is higher than or equal to target")