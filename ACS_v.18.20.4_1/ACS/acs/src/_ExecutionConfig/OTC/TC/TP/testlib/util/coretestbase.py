import sys
import os
import unittest

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig


class CoreTestBase(unittest.TestCase):
    """
    Android Core Test Case Implementation

    """
    # Configuration parser
    config = TestConfig()

    def setUp(self):
        super(CoreTestBase, self).setUp()

        if not 'TEST_DATA_ROOT' in os.environ:
            raise Exception('env \'TEST_DATA_ROOT\' not defined !')

        # Contexts object is injected by nose reporter plugin.
        if hasattr(self, 'contexts'):
            g_common_obj.set_context(self.contexts)

    def tearDown(self):
        super(CoreTestBase, self).tearDown()
