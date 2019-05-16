'''
Created on Jan 8, 2015

@author: yusux
'''
import os

from testlib.graphics.rs_imageprocessorImpl import RS_ImageProcessor_Impl
from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase


class RS_ImageProcessor(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        '''
        install apk
        '''
        super(RS_ImageProcessor, self).setUpClass()
        '''
            install from Artifactory
        '''
        self.imageprocessor = RS_ImageProcessor_Impl()
        self.imageprocessor.installImageProcessor()

    def setUp(self):
        super(RS_ImageProcessor, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        self.imageprocessor = None
        super(RS_ImageProcessor, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testImageProcessing_GenGPU(self):
        self.imageprocessor.hw_accelerated("Compute Device: GPU")

    def testrunRsIPBenchmark(self):
        self.imageprocessor.run_benchmark()
