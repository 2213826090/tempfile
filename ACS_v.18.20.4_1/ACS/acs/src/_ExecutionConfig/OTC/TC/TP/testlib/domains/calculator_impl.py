# coding: UTF-8
import os
import time
from testlib.util.common import g_common_obj

class CalculatorImpl:
    """
    Implements Calculator app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_calculator(self):
        '''
        Launch Calculator app.
        '''
        print "[Info] ---Launch Calculator app."
        g_common_obj.launch_app_am("com.android.calculator2", ".Calculator")
        time.sleep(1)
        assert self.d(text="=").exists

    def input_number(self):
        '''
        Input number.
        '''
        print "[Info] ---Input number."
        for i in range(10):
            self.d(text=i).click()
        assert self.d(text="0123456789").exists

    def rotate_screen(self, text = "0123456789"):
        '''
        Rotate screen.
        '''
        print "[Info] ---Rotate screen."
        self.d.orientation = "l"
        time.sleep(1)
        assert self.d(text=text).exists
        self.d.orientation = "u"
        time.sleep(1)
        assert self.d(text=text).exists
        self.d.orientation = "r"
        time.sleep(1)
        assert self.d(text=text).exists
        self.d.orientation = "n"
        time.sleep(1)
        assert self.d(text=text).exists

