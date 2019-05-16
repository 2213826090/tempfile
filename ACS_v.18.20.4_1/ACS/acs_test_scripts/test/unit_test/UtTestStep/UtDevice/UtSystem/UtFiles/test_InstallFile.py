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
:author: kturban
"""
import os
import posixpath
from mock import Mock, MagicMock
from Core.TestStep.TestStepContext import TestStepContext
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Files.InstallFile import InstallFile
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class InstallFileTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._first_check_file = True

    def test_install_file_ko_unexisting_file(self):
        """
            Check that unexisting app path will raise error
        """
        teststep = InstallFile(None, None, None, Mock())
        teststep._pars.file_path = os.path.join(os.path.dirname(__file__), "doesnotexist")
        with self.assertRaises(AcsConfigException):
            teststep.run(self._context)

    def test_push_file_bin_type_ko(self):
        context = TestStepContext()
        teststep = InstallFile(None, None, None, Mock())
        teststep._pars.type = "bin"
        teststep._pars.file_path = os.path.join(__file__)
        teststep._pars.destination = "/sdcard/acs_files"
        teststep._pars.timeout = 10
        teststep._system_api.check_device_file_install.side_effect = MagicMock(side_effect=self._stub_check_device_file_install_KO_OK)

        teststep._system_api.install_device_executable.return_value = Global.FAILURE, "push failed"

        with self.assertRaises(DeviceException):
            teststep.run(self._context)
        teststep._system_api.install_device_executable.assert_called_once_with(os.path.join(__file__),
                                                                           "/sdcard/acs_files",
                                                                           10)
        self.assertEqual(teststep._device.push.call_count, 0)

    def test_push_file_bin_type_ok(self):
        context = TestStepContext()
        teststep = InstallFile(None, None, None, Mock())
        teststep._pars.type = "bin"
        teststep._pars.file_path = os.path.join(__file__)
        teststep._pars.destination = "/sdcard/acs_files"
        teststep._pars.timeout = 10
        teststep._pars.destination_stored_path = "MY_KEY"

        teststep._system_api.install_device_executable.return_value = Global.SUCCESS, "push done"
        teststep._system_api.check_device_file_install.side_effect = MagicMock(side_effect=self._stub_check_device_file_install_KO_OK)

        teststep._device.get_device_os_path.return_value = posixpath
        expected_destination = "/sdcard/acs_files/%s" % (os.path.basename(__file__))

        teststep.run(context)
        teststep._system_api.install_device_executable.assert_called_once_with(os.path.join(__file__),
                                                                           "/sdcard/acs_files",
                                                                           10)
        self.assertEqual(teststep._device.push.call_count, 0)

    def test_push_file_media_type_ko(self):
        teststep = InstallFile(None, None, None, Mock())
        teststep._pars.type = "media"
        teststep._pars.file_path = os.path.join(__file__)
        teststep._pars.destination = "/sdcard/acs_files"
        teststep._pars.timeout = 10

        teststep._system_api.check_device_file_install.side_effect = MagicMock(side_effect=self._stub_check_device_file_install_KO_OK)
        teststep._device.push.return_value = Global.FAILURE, "push failed"
        teststep._device.get_device_os_path.return_value = posixpath
        expected_destination = "/sdcard/acs_files/%s" % (os.path.basename(__file__))

        with self.assertRaises(DeviceException):
            teststep.run(self._context)
        teststep._device.push.assert_called_once_with(os.path.join(__file__), expected_destination, 10)
        self.assertEqual(teststep._device.install_device_executable.call_count, 0)

    def test_push_file_media_type_ok(self):
        context = TestStepContext()
        teststep = InstallFile(None, None, None, Mock())
        teststep._pars.type = "media"
        teststep._pars.file_path = os.path.join(__file__)
        teststep._pars.destination = "/sdcard/acs_files"
        teststep._pars.timeout = 10
        teststep._pars.destination_stored_path = "MY_KEY"

        teststep._device.push.return_value = Global.SUCCESS, "push done"
        teststep._device.get_device_os_path.return_value = posixpath
        expected_destination = "/sdcard/acs_files/%s" % (os.path.basename(__file__))
        teststep._system_api.check_device_file_install.side_effect = MagicMock(side_effect=self._stub_check_device_file_install_KO_OK)

        teststep.run(context)
        teststep._device.push.assert_called_once_with(os.path.join(__file__), expected_destination, 10)
        self.assertEqual(teststep._device.install_device_executable.call_count, 0)
        self.assertEqual(context.get_info("MY_KEY"), expected_destination)

    def _stub_check_device_file_install_KO_OK(self, a, b):
        if (self._first_check_file == True):
            self._first_check_file = False
            return Global.FAILURE, ""
        else:
            self._first_check_file = True
            return Global.SUCCESS, "File already exist at : "
