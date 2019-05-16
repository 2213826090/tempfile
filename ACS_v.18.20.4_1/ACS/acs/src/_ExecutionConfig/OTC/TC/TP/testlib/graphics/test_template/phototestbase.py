# -*- coding: utf-8 -*-

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.common import rotate_screen


class PhotoTestBase(UIATestBase):

    @classmethod
    def setUpClass(cls):
        super(PhotoTestBase, cls).setUpClass()
        cls.base_d = g_common_obj.get_device()

    def setUp(self):
        super(PhotoTestBase, self).setUp()

    def tearDown(self):
        super(PhotoTestBase, self).tearDown()
