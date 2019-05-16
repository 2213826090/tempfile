# -*- coding: utf-8 -*-
'''
Created on 03/20/2015
@author: Ding, JunnanX
'''
from tests.Graphics_System.GLES.conformance import RunConformance


class BasicTexExtyuv420p(RunConformance):

    def test_basic_texExtyuv420p(self):
        """
        test_basic_texExtyuv420p

        Steps:
        1. run the suite image-external_es from  OGL Conformance tool on es2 switch:"adb shell /data/app/oglconform -v 4 -minFmt -es2 -test image-external_es".
            basic.texExtyuv420p must pass
        """
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case('es2', 'image-external_es')
