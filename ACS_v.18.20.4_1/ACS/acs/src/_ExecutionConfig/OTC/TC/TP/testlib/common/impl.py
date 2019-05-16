# coding: UTF-8
import os
import sys
import time
from testlib.common.common import g_common_obj2

class ImplCommon(object):
    def __init__(self, device=None):
        self.d = device
        self.commonObj = g_common_obj2
        if self.d is None:
            self.d = self.commonObj.get_device()


    def setChecked(self, uiautomatorObj, checked=True):
        checked = bool(checked)
        if uiautomatorObj.checked != checked:
            uiautomatorObj.click()

    def __getattr__(self, name):
        if hasattr(self, name+"Kwargs"):
            return self.d(**getattr(self, name+"Kwargs"))
        raise AttributeError("%s has no such attr: %s"%(self.__class__.__name__, name))