# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 25/02/14
:author: cbonnard
"""
from mock import Mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Misc.ConnectDevice import ConnectDevice
from ErrorHandling.DeviceException import DeviceException


class ConnectDeviceTestCase(UTestTestStepBase):

    def test_connect_board(self):
        """
            Check that contract interface method is used
            connect_board fwk devicemodel
        """
        teststep = ConnectDevice(None, None, None, Mock())
        teststep.run(self._context)
        teststep._device.connect_board.assert_called_once_with()

    def test_exception_failed_connection(self):
        """
         Check that exception is raised when connection to device fails
        """
        teststep = ConnectDevice(None, None, None, Mock())
        teststep._device.is_available.return_value = False
        with self.assertRaises(DeviceException):
            teststep.run(self._context)







