import traceback
import sys
import os
import unittest

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig


class UIATestBase(unittest.TestCase):
    """
    Android UI Automataed Test Case Implementation

    """
    # Configuration parser
    config = TestConfig()

    def setUp(self):
        super(UIATestBase, self).setUp()

        if not 'TEST_DATA_ROOT' in os.environ:
            raise Exception('env \'TEST_DATA_ROOT\' not defined !')

        # Contexts object is injected by nose reporter plugin.
        if hasattr(self, 'contexts'):
            g_common_obj.set_context(self.contexts)

        user_log_dir = os.environ.get('PYUNIT_USER_LOG_DIR', None)

        if (user_log_dir):
            g_common_obj.globalcontext.user_log_dir = user_log_dir


        # Start the execption handling.
        g_common_obj.start_exp_handle()
        g_common_obj.back_home()

    def tearDown(self):
        super(UIATestBase, self).tearDown()
        # Stop the execption handling.
        g_common_obj.stop_exp_handle()
        g_common_obj.back_home()
        # Restart server
        #g_common_obj.restart_server()

    def assert_exp_happens(self):
        # Assert the execption hanppening
        g_common_obj.assert_exp_happens()

    @property
    def failureException(self):
        try:
            scname = g_common_obj.globalcontext.user_log_dir + "/screenshot.png"
            g_common_obj.take_screenshot(scname)
        except Exception as e:
            print e.message, e.args
            traceback.print_exc()

