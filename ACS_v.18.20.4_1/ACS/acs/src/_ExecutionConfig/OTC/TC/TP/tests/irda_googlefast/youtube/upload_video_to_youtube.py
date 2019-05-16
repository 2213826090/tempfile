"""
@summary: Test share youtube video
@since: 09/30/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.youtube.youtube_impl import YoutubeImpl
from testlib.camera.camera import Camera
import time

class UploadVideoToYoutube(UIATestBase):
    """
    @summary: youtube used to test share video via gmail function
    """

    def setUp(self):
        super(UploadVideoToYoutube, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        cfg_file = os.path.join(os.environ.get('TEST_REPO_ROOT', ''), \
            'tests.tablet.google_fast.conf')
        self.youtube = YoutubeImpl(\
            self.config.read(cfg_file, 'youtube'))
        self.youtube.set_orientation_n()
        self.youtube.cleanCameraData()
        self.camera = Camera()

    def tearDown(self):
        super(UploadVideoToYoutube, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.youtube = None
        self.camera = None

    def testUploadVideoToYoutube(self):
        """
        This test used to test share the video to gmail function.
        The test case spec is following:
        1. Launch the "camere" and record a video.
        2. Test share the video to youtube success.
        """

        print "[RunTest]: %s" % self.__str__()
        self.camera.launchCamera()
        self.camera.cameraSwitchTo("Video")
        self.camera.videoRecordStart()
        time.sleep(4)
        self.camera.videoRecordEnd()
        self.youtube.upload_video_to_youtube()
