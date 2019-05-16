#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -------------------------------------------------------------------------------
# @copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.

# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.

# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.

# @organization: INTEL MCG PSI
# @summary: Unittest for the BinaryManager Class.
# @since: 4/25/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import unittest2 as unittest
from mock import patch

from setups.managers.binary import BinaryManager as Mgr


class TestBinaryManager(unittest.TestCase):

    def setUp(self):
        self.mgr = Mgr()

    def tearDown(self):
        del self.mgr

    def test_file_extensions_with_unix_paths(self):
        """
        Tells if the provided string endswith on of passed extensions.

        """
        file_root_tmpl = "/home/acs/.cache/MobaXterm/MobaXterm{0}"

        well_formed_files, well_formed_ext = [], ('.zip', '.tar', '.tar.gz')
        for ext in well_formed_ext:
            well_formed_files.append(file_root_tmpl.format(ext))

        non_well_formed_files = []
        for ext in ('.exe', '.txt', '.rar'):
            non_well_formed_files.append(file_root_tmpl.format(ext))

        for well in well_formed_files:
            ret = self.mgr.file_extensions(well, well_formed_ext)
            self.assertTrue(ret)

        for non_well in non_well_formed_files:
            ret = self.mgr.file_extensions(non_well, well_formed_ext)
            self.assertFalse(ret)

    def test_file_extensions_with_windows_paths(self):
        """
        Tells if the provided string endswith on of passed extensions.

        """
        file_root_tmpl = "C:/home/acs/.cache/MobaXterm/MobaXterm{0}"

        well_formed_files, well_formed_ext = [], ('.zip', '.tar', '.tar.gz')
        for ext in well_formed_ext:
            well_formed_files.append(file_root_tmpl.format(ext))

        non_well_formed_files = []
        for ext in ('.exe', '.txt', '.rar'):
            non_well_formed_files.append(file_root_tmpl.format(ext))

        for well in well_formed_files:
            ret = self.mgr.file_extensions(well, well_formed_ext)
            self.assertTrue(ret)

        for non_well in non_well_formed_files:
            ret = self.mgr.file_extensions(non_well, well_formed_ext)
            self.assertFalse(ret)

    @patch("setups.managers.binary.path")
    @patch("setups.managers.ManagerBase.run_command")
    def test_which_method(self, mocked_run_command, mocked_os_path):
        """
        This test ensure that the Cache System engine, correctly handle
        local cache artifacts

        """
        mocked_os_path.exists.return_value = True

        if self.mgr.session.os == "win32":
            mocked_winreg_OpenKey = patch("setups.managers.binary.winreg.OpenKey")
            mocked_QueryInfoKey = patch("setups.managers.binary.winreg.QueryInfoKey")

            mocked_QueryInfoKey.return_value = (1, 5, 758)

            method = 'winreg'
            value = r'SOFTWARE\Microsoft\VisualStudio\10.0\VC\VCRedist\x86'

            self.mgr.win_which(method, value)
            self.assertEqual(1, mocked_winreg_OpenKey.call_count)
        else:
            self.mgr.is_executable_present({"name": "java"})
            self.assertEqual(1, mocked_run_command.call_count)

    @patch.object(Mgr, "session")
    @patch.object(Mgr, "win_which")
    def test_is_executable_present(self, mocked_win_which, mocked_context):
        """
        This test ensure that the Cache System engine, correctly handle
        local cache artifacts

        """
        mocked_context.ops = "win32"
        mocked_win_which.return_value = r"C:\Windows\System32\java.exe"

        java = {
            "name": "java",
            "check_exists": ('which', ["java.exe"])
        }

        is_present = self.mgr.is_executable_present(java)

        self.assertEqual(1, mocked_win_which.call_count)
        self.assertTrue(is_present)
