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
import mock
from mock import patch

from setups.managers.pypi import PyPiManager as Mgr


class TestPypiManager(unittest.TestCase):

    def setUp(self):
        self._mocked_logger = patch("setups.managers.pypi.logger")
        self._mocked_setup_proxy = patch("setups.helpers.system.setup_proxy")

        self._mocked_setup_proxy.start()
        self._logger = self._mocked_logger.start()

        self.mgr = Mgr(repo=Mgr.DEFAULT_REPO_BASE, cache=Mgr.DEFAULT_CACHE, data={})

    def tearDown(self):
        self._mocked_setup_proxy.stop()
        self._mocked_logger.stop()

        del self.mgr
        del self._logger

    def test_configure_pip_wheel_update_pip(self):
        """
        Configures all requirements for allowing use of :mod:`pip` module.
        Ensures Pip version is minimum 1.5.4+

        """
        pkg_resources_mock = mock.Mock()
        pip_mock = mock.Mock()

        pkg_resources_mock.parse_version = lambda v: int(''.join(v.split('.')))
        pip_mock.__version__ = current_v = '1.4.1'

        with patch.dict('sys.modules', {
            'pkg_resources': pkg_resources_mock,
            'pip': pip_mock
        }):
            self.mgr.ensure_pip_wheel()
            self.assertTrue(1, self._logger.info.call_count)
            logger_info_message = ""
            try:
                logger_info_message = self._logger.info.call_args[0][0]
            finally:
                self.assertEqual("PIP Current version ({0}) "
                                 "must be upgraded to at least 1.5.4".format(current_v), logger_info_message)
            self.assertTrue(1, pip_mock.main.call_count)

    def test_configure_pip_wheel_version_satisfied(self):
        """
        Configures all requirements for allowing use of :mod:`pip` module.
        Ensures Pip version is minimum 1.5.4+

        """
        pkg_resources_mock = mock.Mock()
        pip_mock = mock.Mock()

        pkg_resources_mock.parse_version = lambda v: int(''.join(v.split('.')))
        pip_mock.__version__ = current_v = '1.5.5'

        with patch.dict('sys.modules', {
            'pkg_resources': pkg_resources_mock,
            'pip': pip_mock
        }):
            self.mgr.ensure_pip_wheel()
            self.assertTrue(1, self._logger.info.call_count)
            logger_info_message = ""
            try:
                logger_info_message = self._logger.info.call_args[0][0]
            finally:
                self.assertEqual("PIP ({0}) Already Installed!".format(current_v), logger_info_message)
            self.assertEqual(0, pip_mock.main.call_count)

    @patch.object(Mgr, "easy_install")
    def test_configure_pip_wheel_with_no_pip(self, mocked_easy_install):
        """
        Configures all requirements for allowing use of :mod:`pip` module.
        Ensures Pip version is minimum 1.5.4+

        """
        pkg_resources_mock = mock.Mock()
        pip_mock = mock.Mock()

        pkg_resources_mock.parse_version = lambda v: int(''.join(v.split('.')))
        pip_mock.side_effect = ImportError

        with patch.dict('sys.modules', {
            'pkg_resources': pkg_resources_mock,
            'pip': None
        }):
            self.mgr.ensure_pip_wheel()
            self.assertTrue(1, self._logger.info.call_count)
            logger_info_message = ""
            try:
                logger_info_message = self._logger.info.call_args[0][0]
            finally:
                self.assertEqual("Installing PIP...", logger_info_message)
            self.assertEqual(1, mocked_easy_install.call_count)

    @patch("setups.managers.pypi.ez_setup")
    def test_configure_setuptools(self, mocked_ez_setup):
        """

        """
        with patch.dict('sys.modules', {
            'setuptools': None
        }):
            self.mgr.configure_setuptools()

            logger_info_message = ""
            try:
                logger_info_message = self._logger.info.call_args[0][0]
            finally:
                self.assertEqual("Installing SETUPTOOLS...", logger_info_message)
            self.assertEqual(1, mocked_ez_setup.main.call_count)
            self.assertEqual(2, self._logger.warning.call_count)
