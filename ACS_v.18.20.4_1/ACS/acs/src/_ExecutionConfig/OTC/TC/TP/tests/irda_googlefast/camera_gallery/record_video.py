import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.camera.camera_impl import CameraImpl as Impl

class RecordVideo(UIATestBase2):
    impl = Impl
    def testRecordVideo(self):
        self.impl.launchCamera()
        self.impl.cameraSwitchTo("Video")
        self.impl.videoRecordStart()
        time.sleep(int(self.record_time))
        self.impl.videoRecordEnd()
        time.sleep(1)
        self.impl.deleteVideo()
