#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
:summary: Configure python path and execute all test available in the unit_test folder
:since: 1/27/14
:author: sfusilie
"""

import os
import sys

import unittest2 as unittest


MY_FOLDER = os.path.dirname(__file__)
UT_FOLDER = os.path.normpath(os.path.join(MY_FOLDER, "unit_test"))
# Update python paths
PATH_TO_ADD = [os.path.normpath(os.path.join(MY_FOLDER, "..")),
               os.path.normpath(os.path.join(MY_FOLDER, "..", "test")),
               os.path.normpath(os.path.join(MY_FOLDER, "..", "..")),
               os.path.normpath(os.path.join(MY_FOLDER, "..", "..", "acs", "src")),
               os.path.normpath(os.path.join(MY_FOLDER, "..", "..", "acs", "test"))]

for src_path in PATH_TO_ADD:
    if os.path.exists(src_path) and src_path not in sys.path:
        sys.path.append(src_path)


def load_tests():
    # Discover unit test file to execute
    suite = unittest.TestSuite()
    unit_test_dir = os.path.join(os.path.dirname(__file__), UT_FOLDER)
    for test in unittest.defaultTestLoader.discover(unit_test_dir, pattern='test_*.py'):
        suite.addTests(test)
    return suite


def run_test():

    # Execute unit test
    return unittest.TextTestRunner(verbosity=1).run(load_tests())

if __name__ == '__main__':
    if run_test().wasSuccessful():
        sys.exit(0)
    else:
        sys.exit(1)
