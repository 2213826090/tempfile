# coding: UTF-8
import os, sys
import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.calculator.calculator import Calculator as Impl

class Calculator(UIATestBase2):
    impl=Impl
    def testCalculator(self):
        self.impl.launch()
        for exp, result in self.getExpList():
            self.impl.clearText()
            assert self.impl.calcExp(exp, result)

    def getExpList(self):
        retList=[]
        for i in range(1,100):
            if hasattr(self, "exp%d"%i):
                each=(getattr(self,"exp%d"%i), getattr(self,"exp%d_result"%i))
                retList.append(each)
            else:
                break
        return retList