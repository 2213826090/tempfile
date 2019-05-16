import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.camera.camera_impl import CameraImpl as Impl


class TakePictures(UIATestBase2):
    impl = Impl
    def testTakePictures(self):
        self.impl.launchCamera()
        self.impl.cameraSwitchTo("Camera")
        num = int(self.take_pics_number)/2
        self.impl.takePics(num)
        self.impl.cameraSwitchBackFront()
        self.impl.takePics(num)
        self.impl.cameraSwitchBackFront()



