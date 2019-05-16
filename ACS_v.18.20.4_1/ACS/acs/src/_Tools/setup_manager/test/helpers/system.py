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

from setups.helpers import system
from setups.helpers.system import DEFAULT_PROXY_SETTINGS


class TestSystem(unittest.TestCase):

    def setUp(self):
        self._mocked_os = patch("setups.helpers.system.os")
        self._mocked_os_obj = self._mocked_os.start()
        self._mocked_logger = patch("setups.helpers.system.logger")
        self._mocked_logger_obj = self._mocked_logger.start()
        self._test_os_environ = {
            "HTTP_PROXY": "http://test/unit/proxy",
            "HTTPS_PROXY": "http://test/unit/proxy"
        }
        self.test_proxy_value = self._test_os_environ['HTTP_PROXY']
        self.default_proxy_value = DEFAULT_PROXY_SETTINGS['http']

    def tearDown(self):
        self._mocked_os.stop()
        self._mocked_logger.stop()
        del self._mocked_os_obj
        del self._mocked_logger_obj

    def _mock_os_environ(self, env=None):
        """
        """
        env = env or {}
        self._mocked_os_obj.environ = env

    def test_reset_setup_proxy(self):
        """

        """
        self._mock_os_environ(self._test_os_environ)
        system.setup_proxy(reset=True)
        self.assertTrue(1, self._mocked_logger_obj.info.call_count)
        logger_info_message = ""
        try:
            logger_info_message = self._mocked_logger_obj.info.call_args[0][0]
        finally:
            self.assertIn('Resetting HTTP_PROXY', logger_info_message)
        self.assertIn("NO_PROXY", self._mocked_os_obj.environ)
        self.assertEqual(".intel.com", self._mocked_os_obj.environ["NO_PROXY"])
        self.assertNotIn("HTTP_PROXY", self._mocked_os_obj.environ)
        self.assertNotIn("HTTPS_PROXY", self._mocked_os_obj.environ)

    def test_setup_proxy_with_no_proxy_in_os_environ(self):
        """

        """
        self._mock_os_environ()
        system.setup_proxy()
        self.assertTrue(2, self._mocked_logger_obj.info.call_count)
        for idx in xrange(2):
            logger_info_message = ""
            try:
                logger_info_message = self._mocked_logger_obj.mock_calls[idx][1][0]
            finally:
                self.assertEqual("Setting {0} to {1}".format("HTTPS_PROXY" if idx else "HTTP_PROXY",
                                                             self.default_proxy_value), logger_info_message)
        self.assertNotIn("NO_PROXY", self._mocked_os_obj.environ)
        self.assertIn("HTTP_PROXY", self._mocked_os_obj.environ)
        self.assertEqual(self.default_proxy_value, self._mocked_os_obj.environ["HTTP_PROXY"])
        self.assertIn("HTTPS_PROXY", self._mocked_os_obj.environ)
        self.assertEqual(self.default_proxy_value, self._mocked_os_obj.environ["HTTPS_PROXY"])

    def test_setup_proxy_with_proxy_in_os_environ(self):
        """

        """
        self._mock_os_environ(self._test_os_environ)
        system.setup_proxy()
        self.assertTrue(2, self._mocked_logger_obj.info.call_count)
        for idx in xrange(2):
            logger_info_message = ""
            try:
                logger_info_message = self._mocked_logger_obj.mock_calls[idx][1][0]
            finally:
                self.assertEqual("{0} Proxy Already Set: {1}".format("HTTPS" if idx else "HTTP",
                                                                     self.test_proxy_value), logger_info_message)
        self.assertNotIn("NO_PROXY", self._mocked_os_obj.environ)
        self.assertIn("HTTP_PROXY", self._mocked_os_obj.environ)
        self.assertEqual(self.test_proxy_value, self._mocked_os_obj.environ["HTTP_PROXY"])
        self.assertIn("HTTPS_PROXY", self._mocked_os_obj.environ)
        self.assertEqual(self.test_proxy_value, self._mocked_os_obj.environ["HTTPS_PROXY"])

    def test_setup_exception(self):
        """

        """
        class_name = 'TestExceptionName'
        exception_message = 'A Test Exception ...'
        exc = system.setup_exception(class_name, exception_message)
        self.assertTrue(issubclass(exc.__class__, Exception))
        self.assertTrue(exc.__class__.__name__ == class_name)
        self.assertTrue(exc.message == exception_message)
