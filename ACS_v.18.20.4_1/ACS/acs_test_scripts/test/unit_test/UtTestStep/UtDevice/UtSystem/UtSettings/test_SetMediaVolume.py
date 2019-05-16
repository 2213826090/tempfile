"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
@summary: Unit test module
@since 19/03/2013
@author: baptistx
"""

from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Settings.SetMediaVolume import SetMediaVolume
from UtilitiesFWK.Utilities import Global


class SetStreamVolumeTestCase(UTestTestStepBase):

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def test_set_stream_volume_50_succeed(self):
        self._create_sut(50)
        self._sut.run(self._context)
        self._assert_adjust_specified_stream_volume_was_called_with_value(50)

    def _create_sut(self, volume):
        self._sut = SetMediaVolume(None, None, {"VOLUME": volume}, Mock())

    def _assert_adjust_specified_stream_volume_was_called_with_value(self, volume):
        self._sut._system.adjust_specified_stream_volume.assert_called_once_with("Media", volume)

