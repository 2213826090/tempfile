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
# @summary: Unittest for ACSSetup Class.
# @since: 4/25/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import unittest2 as unittest
from mock import patch, PropertyMock

from setups.context import SETUP_SESSION
from setups.setup_mgr import ACSSetup


class TestACSSetup(unittest.TestCase):

    def setUp(self):

        self._mocked_logger = patch("setups.setup_mgr.logger")
        self._mocked_setup_proxy = patch("setups.helpers.system.setup_proxy")

        self._mocked_setup_proxy.start()
        self._logger = self._mocked_logger.start()

        self._create_setup = ACSSetup
        self._parser = ACSSetup.build_parser()

    def tearDown(self):
        self._mocked_setup_proxy.stop()
        self._mocked_logger.stop()

        del self._logger
        del self._create_setup

    def _main(self, command_args=None):
        setup = self._create_setup(self._parser.parse_args(command_args or []))
        setup.run()

    @patch('setups.setup_mgr.ACSSetup.is_root', new_callable=PropertyMock)
    def test_run_on_linux_non_root(self, mocked_is_root):
        """
        Tells if the provided string endswith on of passed extensions.

        """
        mocked_is_root.return_value = False
        SETUP_SESSION.os = "linux2"
        self.assertRaises(Exception, self._main, ['install', 'user'])
