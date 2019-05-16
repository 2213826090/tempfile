# -*- coding: utf-8 -*-
'''
Created on 6/26/2015

@author: zhangroX
'''

from testlib.graphics.MIPI_impl import MIPIImpl
from testlib.util.uiatestbase import UIATestBase


class MIPI(UIATestBase):

    def setUp(self):
        super(MIPI, self).setUp()
        self.MIPI_impl = MIPIImpl()

    def tearDown(self):
        super(MIPI, self).tearDown()
        self.MIPI_impl = None

    def test_DisplayInterface_MIPI(self):
        self.MIPI_impl.test_MIPI()

