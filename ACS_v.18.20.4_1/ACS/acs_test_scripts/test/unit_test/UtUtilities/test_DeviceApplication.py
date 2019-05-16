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
:since: 10/09/2014
:author: cbonnard
"""

import unittest
from mock import patch, Mock

from acs_test_scripts.Utilities.DeviceApplication import AndroidApplication
import acs_test_scripts.Utilities.DeviceApplication as DeviceApplication


class DeviceApplicationTestCase(unittest.TestCase):

    PARSED_DATA = "package: name='com.intel.test' versionCode='1' versionName='1.0'"

    def test_extract_application_data_fills_members(self):
        app = AndroidApplication()

        app._extract_application_data("package: name='com.intel.test' versionCode='1' versionName='1.0'")

        self.assertTrue(isinstance(app._pkg_name, str))
        self.assertTrue(isinstance(app._version_code, str))
        self.assertTrue(isinstance(app._version_name, str))

    @patch.object(DeviceApplication, "internal_shell_exec")
    def test_parse_application_data_calls_extract_application_data(self, mock_internal_shell_exec):
        app = AndroidApplication(path="test.apk")
        app._extract_application_data = Mock()
        mock_internal_shell_exec.return_value = 0, self.PARSED_DATA
        app._parse_application_data()

        app._extract_application_data.assert_called_once_with(self.PARSED_DATA)
        self.assertEqual(mock_internal_shell_exec.call_count, 1)

    def test_get_package_name(self):
        app = AndroidApplication()
        app._extract_application_data("package: name='com.intel.test' versionCode='1' versionName='1.0'")

        self.assertTrue(app.get_package_name() == "com.intel.test")

    def test_get_package_version_code(self):
        app = AndroidApplication()
        app._extract_application_data("package: name='com.intel.test' versionCode='1' versionName='1.0'")

        self.assertTrue(app.get_package_version_code() == "1")

    def test_get_package_version_name(self):
        app = AndroidApplication()
        app._extract_application_data("package: name='com.intel.test' versionCode='1' versionName='1.0'")

        self.assertTrue(app.get_package_version_name() == "1.0")

    @patch.object(DeviceApplication, "internal_shell_exec")
    def test_extract_application_data_empty_members(self, mock_internal_shell_exec):
        app = AndroidApplication(path="test.apk")
        app._extract_application_data = Mock()
        mock_internal_shell_exec.return_value = 0, "package: name='' versionCode='' versionName=''"
        app._parse_application_data()

        self.assertTrue(app.get_package_name() is None)
        self.assertTrue(app.get_package_version_name() is None)
        self.assertTrue(app.get_package_version_code() is None)
