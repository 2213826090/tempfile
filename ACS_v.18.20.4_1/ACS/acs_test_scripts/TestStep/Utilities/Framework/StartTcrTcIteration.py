"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC SSG PNP
:summary: This file implements a Test Step to send additional test case data to tcr
:since: 13/04/2016
:author: tchourrx
"""

from Core.TestStep.DeviceTestStepBase import TestStepBase
from Core.Report.Live.LiveReporting import LiveReporting

class StartTcrTcIteration(TestStepBase):
    """
    Send additional test case data to TCR
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        :type tc_conf: :py:class:`~src.Core.TCParameters`
        :param tc_conf: test case parameters
        :type  global_conf: object
        :param global_conf: global configuration data
        :type  ts_conf: object
        :param ts_conf: test steps parameters
        :type  factory: object
        :param factory: it's responsible for creating ACS objects needed by the test step
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        if self._tc_parameters.get_b2b_iteration() > 1:
            LiveReporting.instance().send_start_tc_info(tc_name=self._pars.iteration_name, iteration=True)
