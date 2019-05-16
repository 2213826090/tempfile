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
:summary: unit test
:since:02/04/2014
:author: eponscol
"""

from mock import patch, Mock, MagicMock
from acs_test_scripts.Device.UECmd.Imp.Android.JB_MR1.Misc.PhoneSystem import PhoneSystem as PhoneSystemJB
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from ErrorHandling.DeviceException import DeviceException
from unit_test_fwk.UTestBase import UTestBase
from unit_test_fwk.UtDevice.UtDeviceModel.DeviceHelpers import DeviceHelpers

# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212

class PhoneSystemTest(UTestBase):
    def setUp(self):
        # Minimal objects to mock for instantiating a Device are:
        self._device_helpers = DeviceHelpers()
        self._device_helpers.start_device_mock()

        # Equipment manager
        self._equipment_manager = patch("acs_test_scripts.Equipment.EquipmentManager.EquipmentManager")
        self._equipment_manager.start()

        # Device controller
        self._device_controller = patch("Device.DeviceController.DeviceController.DeviceController")
        self._device_controller.start()

        # Device manager
        self._device_manager = patch("Device.DeviceManager.DeviceManager")
        device_man = self._device_manager.start()()

        # Mock global config of the device manager to handle some
        # specific value
        self.global_cfg = Mock()
        # We handle a path for campaignConfig because of some
        # os.path in AndroidDeviceBase __init__
        self.global_cfg.campaignConfig = MagicMock()
        self.global_cfg.campaignConfig.get.return_value = False
        # Attach the global config to the device manager
        device_man.get_global_config.return_value = self.global_cfg

    # tearDown are executed after each test
    def tearDown(self):
        # Stop mocking engine
        self._equipment_manager.stop()
        self._device_controller.stop()
        self._device_manager.stop()
        self._device_helpers.stop_device_mock()

    def _get_device(self, config=None):
        device_config = self._device_helpers.get_device_config(config)
        device = AndroidDeviceBase(device_config, MagicMock())
        return device

    @patch.object(AndroidDeviceBase, "get_device_module")
    @patch.object(PhoneSystemJB, "_exec")
    def test_unexpected_error_when_set_verify_application(self, mocked_exec_cmd):
        """
        Checks that exception is raised when an unexpected error occurs

        """
        mocked_exec_cmd.return_value = "Unexpected error"
        device = self._get_device()
        device._uecmd_default_timeout = 1
        phone_system = PhoneSystemJB(device)

        self.assertRaisesRegexp(DeviceException, "Unexpected error - Unable to set verify application setting to 0", phone_system.set_verify_application, False, False)

    @patch.object(AndroidDeviceBase, "get_device_module")
    @patch.object(PhoneSystemJB, "_exec")
    def test_locked_database_when_set_verify_application_until_timeout(self, mocked_exec_cmd):
        """
        Checks that if the database is locked, retries are done until timeout is reached

        """
        mocked_exec_cmd.return_value = "Error: database is locked"

        device = self._get_device()
        device._uecmd_default_timeout = 1
        phone_system = PhoneSystemJB(device)

        self.assertRaisesRegexp(DeviceException, "Unable to set verify application setting to 0 - Timeout 1 has been reached", phone_system.set_verify_application, False, False)


