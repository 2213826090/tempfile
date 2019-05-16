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

@summary: This file implements Test Step for Audio Framework (AFFuTe) base
@since: 7/30/2015
@author: fbelvezx
@organization: CCG-CRD-OPM PC&WTE
"""

from acs_test_scripts.Utilities.AudioFrameworkUtilities import AudioFramework as AFFuTe
from Core.TestStep.EquipmentTestStepBase import TestStepBase


class AFFuTeBase(TestStepBase):
    """
    Base class for interaction with AFFuTe python framework
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._audio_fwk_api = AFFuTe()

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        self._audio_fwk_api.server_address = (self._pars.server_address,
                                              int(self._pars.server_port))

        # Initialize AFFuTE
        self._audio_fwk_api.connect_fwk()
        self._audio_fwk_api.initialize_fwk()