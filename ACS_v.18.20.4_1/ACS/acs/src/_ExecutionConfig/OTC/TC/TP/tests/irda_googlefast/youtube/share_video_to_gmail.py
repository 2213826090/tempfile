"""
@summary: Test share youtube video
@since: 09/30/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.youtube.youtube_impl import YoutubeImpl

class ShareVideoToGmail(UIATestBase):
    """
    @summary: youtube used to test share video via gmail function
    """

    def setUp(self):
        super(ShareVideoToGmail, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        cfg_file = os.path.join(os.environ.get('TEST_REPO_ROOT', ''), \
            'tests.tablet.google_fast.conf')
        self.youtube = YoutubeImpl(\
            self.config.read(cfg_file, 'youtube'))
        self.youtube.set_orientation_n()

    def tearDown(self):
        super(ShareVideoToGmail, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testShareVideo(self):
        """
        This test used to test share the video to gmail function.
        The test case spec is following:
        1. Launch the "youtube" and choose a video.
        2. share the video to gmail success.
        """
        print "[RunTest]: %s" % self.__str__()
        self.youtube.share_video_to_gmail()