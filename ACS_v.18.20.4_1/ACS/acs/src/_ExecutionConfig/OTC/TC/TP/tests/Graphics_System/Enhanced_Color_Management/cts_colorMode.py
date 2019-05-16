#!/usr/bin/python
# -*- coding:utf-8 -*

from .common import ColorModeTestBase


class ColorMode(ColorModeTestBase):
    '''
    Wide ColorMode Test
    '''

    def test_AttributeWideColorModeTest_testDefaultColorMode(self):
        self.clm.instr_run_class('AttributeWideColorModeTest#testDefaultColorMode')

    def test_WideColorModeTest_testDefaultColorMode(self):
        self.clm.instr_run_class('WideColorModeTest#testDefaultColorMode')
