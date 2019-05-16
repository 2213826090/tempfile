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
:since: 24/02/14
:author: cbonnard
"""

from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Misc.RebootDevice import RebootDevice
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class RebootDeviceTestCase(UTestTestStepBase):

    def test_reboot_unable_to_switchon_when_not_booted(self):
        """
         Check that exception is raised when trying to switchon when is_booted is false
        """
        teststep = RebootDevice(None, None, None, Mock())
        teststep._device.is_booted.return_value = False
        teststep._device.switch_off.return_value = (Global.SUCCESS, "")
        teststep._device.switch_on.return_value = (Global.FAILURE, "unable to switch on")
        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_reboot_unable_to_switchon_when_booted(self):
        """
         Check that exception is raised when trying to switchon when is_booted is true
        """
        teststep = RebootDevice(None, None, None, Mock())
        teststep._device.is_booted.return_value = True
        teststep._device.switch_off.return_value = (Global.SUCCESS, "")
        teststep._device.switch_on.return_value = (Global.FAILURE, "unable to switch on")
        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_reboot_unable_to_switchoff_when_booted(self):
        """
         Check that exception is raised when trying to switchoff
        """
        teststep = RebootDevice(None, None, None, Mock())
        teststep._device.switch_on.return_value = (Global.SUCCESS, "")
        teststep._device.switch_off.return_value = (Global.FAILURE, "unable to switch off")
        with self.assertRaises(DeviceException):
            teststep.run(self._context)







