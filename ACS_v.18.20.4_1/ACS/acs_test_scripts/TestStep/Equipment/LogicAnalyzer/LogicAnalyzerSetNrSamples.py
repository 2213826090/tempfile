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
:summary: This file implements the step to set the number of samples you want to capture.
:since: 2014-11-17
:author: emarchan

"""
from acs_test_scripts.TestStep.Equipment.LogicAnalyzer.LogicAnalyzerBase import LogicAnalyzerBase


class LogicAnalyzerSetNrSamples(LogicAnalyzerBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        LogicAnalyzerBase.run(self, context)
        assert any(char.isdigit() for char in self._pars.nr_samples), \
            "passed value for nr_samples (%s) is invalid at this stage" % self._pars.nr_samples

        self._log_an.set_number_of_samples(self._pars.nr_samples)
