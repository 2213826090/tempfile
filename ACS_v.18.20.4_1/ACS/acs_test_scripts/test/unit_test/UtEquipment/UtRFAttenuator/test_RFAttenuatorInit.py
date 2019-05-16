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

:organization: INTEL MCG PSI
:summary: Nokia BH214 implementation
:since: 20/02/2014
:author: fbongiax
"""

import mock

from acs_test_scripts.TestStep.Equipment.RFAttenuator.RFAttenuatorInit import RFAttenuatorInit
from acs_test_scripts.test.unit_test.UtEquipment.UtRFAttenuator.test_RFAttenuatorBase import test_RFAttenuatorBase


class RFAttenuationInitTest(test_RFAttenuatorBase):
    """
    Test class for BtMRFAttenuator
    """
    def setUp(self):
        test_RFAttenuatorBase.setUp(self)
        
        
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = RFAttenuatorInit(None,mock.Mock() , test_step_pars, mock.Mock())
        sut._timeout = 0
        return sut

    def test_initiate_ok(self):
        self._sut = self._create_sut()
        self._assert_run_returned(self._sut)

        
    def _assert_run_returned(self, sut):
        sut.run(None)
        sut._rf_attn.configure.assert_called_once_with()
