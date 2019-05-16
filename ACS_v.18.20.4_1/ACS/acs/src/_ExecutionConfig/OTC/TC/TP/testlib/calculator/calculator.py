# coding: UTF-8
import os
import sys
import time
from testlib.common.impl import ImplCommon

class Calculator(ImplCommon):

    def launch(self):
        self.commonObj.launchAppByName("Calculator")
        time.sleep(2)

    def clearText(self):
        if self.d(resourceId="com.android.calculator2:id/clr").exists:
            self.d(resourceId="com.android.calculator2:id/clr").click()
        else:
            def clearTextByDel():
                for i in range(20):
                    for j in range(10):
                        if not self.d(resourceId="com.android.calculator2:id/formula").info[u"text"]:
                            return
                        self.d(resourceId="com.android.calculator2:id/del").click()
            clearTextByDel()


    def calcExp(self, exp, result):
        print exp, "=", result
        exp=exp.replace("-", "−")
        exp=exp.replace("/", "÷")
        exp=exp.replace("*", "×")
        for each in exp.decode("utf-8"):
            self.d(className="android.widget.Button", text=each).click()
        self.d(text="=").click()
        time.sleep(1)
        result = result.replace(".", ".")
        print result
        return self.d(resourceId="com.android.calculator2:id/formula", text=result).exists


