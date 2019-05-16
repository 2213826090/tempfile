# -*- coding: utf-8 -*-
'''
Created on 04/07/2015
@author: Ding, JunnanX
'''

import re

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase


class DPSTSanityTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DPSTSanityTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DPSTSanityTest, self).setUp()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DPSTSanityTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DPSTSanityTest, cls).tearDownClass()

    def test_DPST_Sanity(self):
        """
        test_DPST_Sanity

        Steps:
        1. adb shell cat /sys/kernel/debug/dri/0/i915_capabilities.
            has_dpst should be reported as yes.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. adb shell cat /sys/kernel/debug/dri/0/i915_capabilities.
            has_dpst should be reported as yes."""
        cmd = 'cat /sys/kernel/debug/dri/0/i915_capabilities'
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        result = re.findall(".*has_dpst:.*yes", msg, re.IGNORECASE)
        print "[Debug] %s" % (''.join(result))
        assert result,\
            "[FAILURE] Not found has_dpst:yes\n %s" % (msg)
