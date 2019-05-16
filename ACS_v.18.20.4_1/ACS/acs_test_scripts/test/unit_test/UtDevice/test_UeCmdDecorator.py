#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
@summary: This module implements unit tests for UeCmdUtil
@since: 11/08/14
@author: kturban
"""


import mock
from mock import patch
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from Device.DeviceCapability.DeviceCapability import DeviceCapability
from Device.Model.DeviceBase import DeviceBase
from unit_test_fwk.UTestBase import UTestBase
from ErrorHandling.DeviceException import DeviceException

class UeCmdDecoratorTestCase(UTestBase):
    """
    def setUp(self):
        # Minimal objects to mock for instantiating a Device are:
        self._device_helpers = DeviceHelpers()
        self._device_helpers.start_device_mock()

        # MagicMock global config of the device manager to handle some
        # specific value
        glb_cfg = MagicMock()

        # We handle a path for campaignConfig because of some
        # os.path in Device __init__
        glb_cfg.campaignConfig = MagicMock()
        glb_cfg.campaignConfig.get.return_value = False

        # Attach the global config to the device manager
        self._device_helpers.device_manager.get_global_config.return_value = glb_cfg

    # tearDown are executed after each test
    def tearDown(self):
        # Stop mocking engine
        self._device_helpers.stop_device_mock()
    """

    def setUp(self):
        # Mock device manager
        self.__mock_dm = patch("Device.DeviceManager.DeviceManager")
        self.__mock_dm.start()()

    def tearDown(self):
        self.__mock_dm.stop()


    def _get_device(self, capabilities):
        capabilities = [DeviceCapability({'name': capability}) for capability in capabilities]
        glb_cfg = mock.MagicMock()
        logger = mock.MagicMock()
        device = DeviceBase(glb_cfg, logger)
        device._device_capabilities = capabilities
        return device

    def test_need_cap_ok(self):
        """
        Test when device has the required capability
        Method must be executed properly
        """
        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap2", "cap3", "cap1"])
        ue_cmd = fakeUEcmd(fake_device)
        execution_result = ue_cmd.my_test(True)
        self.assertTrue(execution_result)

    def test_need_missing_cap(self):
        """
        Test when device does not have the required capability
        Method must not be executed and an exception is raised
        """
        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap2", "cap3"])
        ue_cmd = fakeUEcmd(fake_device)
        with self.assertRaises(DeviceException):
            ue_cmd.my_test(True)

    def test_need_cap_ok_with_multiple_cap_or(self):
        """
        Test when device has one of the required capabilities
        Method must be executed properly
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1 or cap2")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap2", "cap3"])
        ue_cmd = fakeUEcmd(fake_device)
        execution_result = ue_cmd.my_test(True)
        self.assertTrue(execution_result)

    def test_need_cap_ok_with_multiple_cap_and(self):
        """
        Test when device has all required capabilities
        Method must be executed properly
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1 and cap2")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap1", "cap2", "cap3", "cap4"])
        ue_cmd = fakeUEcmd(fake_device)
        execution_result = ue_cmd.my_test(True)
        self.assertTrue(execution_result)

    def test_need_cap_ko_with_multiple_cap_or(self):
        """
        Test when device does not have the required capability
        Method must not be executed and an exception is raised
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1 or cap2")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap3", "cap4"])
        ue_cmd = fakeUEcmd(fake_device)
        with self.assertRaises(DeviceException):
            ue_cmd.my_test(True)

    def test_need_cap_ok_with_multiple_cap_and(self):
        """
        Test when device has only one of the required capabilities
        Method must not be executed and an exception is raised
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1 and cap2")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap1", "cap3", "cap4"])
        ue_cmd = fakeUEcmd(fake_device)
        with self.assertRaises(DeviceException):
            ue_cmd.my_test(True)


    def test_need_cap_ok_with_multiple_required_cap(self):
        """
        Test when method needs multiple cap
        Method must be executed properly
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1")
            @need("cap2")
            @need("cap3")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap1", "cap2", "cap3", "cap4"])
        ue_cmd = fakeUEcmd(fake_device)
        execution_result = ue_cmd.my_test(True)
        self.assertTrue(execution_result)

    def test_need_cap_ko_with_multiple_required_cap(self):
        """
        Test when device does not have one of the required capability
        Method must not be executed and an exception is raised
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1")
            @need("cap2")
            @need("cap3")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap1", "cap2", "cap4"])
        ue_cmd = fakeUEcmd(fake_device)
        with self.assertRaises(DeviceException):
            ue_cmd.my_test(True)

    def test_need_cap_ko_not_well_formated(self):
        """
        Test when capability request is too complexe (mixed or/and)
        Method must not be executed and an exception is raised
        """

        class fakeUEcmd(object):
            def __init__(self, device):
                self._device = device

            @need("cap1 and cap2 or cap3 or cap4")
            def my_test(self, my_arg):
                return my_arg

        fake_device = self._get_device(["cap1", "cap2", "cap3", "cap4"])
        ue_cmd = fakeUEcmd(fake_device)
        with self.assertRaises(DeviceException):
            ue_cmd.my_test(True)
