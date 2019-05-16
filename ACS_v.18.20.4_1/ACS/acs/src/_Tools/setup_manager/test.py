#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------------
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
# @summary: ""
# @since: 4/23/14
# @author: nbrissox
#-------------------------------------------------------------------------------

import os
import sys

import unittest2 as unittest


def load_tests():
    """
    Discover unit test file to execute

    :return: The Test Suite
    :rtype: unittest.TestSuite

    """
    suite = unittest.TestSuite()
    unit_test_dir = os.path.join(os.path.dirname(__file__), "test")
    print(unit_test_dir)
    for test in unittest.defaultTestLoader.discover(unit_test_dir, pattern='*.py'):
        suite.addTests(test)

    return suite


def run_test():
    """
    Tests runner

    :return: the TestResult
    :rtype: unittest.TestResult

    """
    # Update Python path(s)
    acs_setups_dir_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    sys.path.insert(0, acs_setups_dir_path)

    # Execute unit test
    return unittest.TextTestRunner(verbosity=1).run(load_tests())

if __name__ == '__main__':
    sys.exit(0) if run_test().wasSuccessful() else sys.exit(1)
