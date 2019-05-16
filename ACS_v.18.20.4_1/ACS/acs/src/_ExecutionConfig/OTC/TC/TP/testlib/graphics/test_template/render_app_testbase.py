# -*- coding: utf-8 -*-

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.common import rotate_screen, close_all_tasks


class RenderAppTestBase(UIATestBase):

    @classmethod
    def setUpClass(cls):
        super(RenderAppTestBase, cls).setUpClass()
        cls.base_d = g_common_obj.get_device()

    def setUp(self):
        super(RenderAppTestBase, self).setUp()
        # self.base_natural_orientation = self.base_d.info['naturalOrientation']
        # self.base_orientation = self.base_d.orientation
        # rotate_screen(mode='narrow')
        close_all_tasks()

    def tearDown(self):
        # self.base_d.orientation = self.base_orientation
        # self.base_d.freeze_rotation(self.base_natural_orientation)
        super(RenderAppTestBase, self).tearDown()
