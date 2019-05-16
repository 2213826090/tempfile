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

@organization: INTEL MCG PSI
@summary: This module implements a test step to compute scores
@since: 16/06/16
@author: tchourrx
"""
import numpy
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException


class Compute(TestStepBase):
    """
    Compute a list of score values
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        inputs = context.get_info(self._pars.input)

        for input in inputs:
            values = numpy.array(input).astype(numpy.float)

        if self._pars.method:
            if self._pars.method.lower() == "median":
                context.set_info(self._pars.result, numpy.median(values))
            elif self._pars.method.lower() == "average":
                context.set_info(self._pars.result, numpy.average(values))
            elif self._pars.method.lower() == "max":
                context.set_info(self._pars.result, numpy.max(values))
            elif self._pars.method.lower() == "min":
                context.set_info(self._pars.result, numpy.min(values))
            else:
                raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR, "Compute method not known")
        else:
            raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR, "Method parameter has to be set")
