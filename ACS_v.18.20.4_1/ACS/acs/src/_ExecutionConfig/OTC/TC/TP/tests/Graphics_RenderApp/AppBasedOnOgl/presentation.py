# -*- coding: utf-8 -*-
'''
@author: yusux
'''
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase


class RunPresentation(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        '''
        install apk
        '''
        super(RunPresentation, self).setUpClass()
        SampleApiDemoImpl().install_apk()

    def setUp(self):
        super(RunPresentation, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._runPresentation = SampleApiDemoImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunPresentation, self).tearDown()
        self._runPresentation.stop_app_am()
        self._runPresentation = None

    def test_start_presentation(self):
        self._runPresentation.launch_app_am()
        self._runPresentation.start_presentation()

    def test_presentation_through_mediarouter(self, delay=21):
        self._runPresentation.launch_app_am()
        self._runPresentation.countdown_t0_perform_presentation_through_mediarouter(
            delay)
