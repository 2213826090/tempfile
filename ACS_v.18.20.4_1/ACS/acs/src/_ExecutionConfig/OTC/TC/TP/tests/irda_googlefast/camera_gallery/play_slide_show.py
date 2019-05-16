# coding: UTF-8
import os, sys
import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.photos.photos_impl import PhotosImpl as Impl

class PlaySlideShow(UIATestBase2):
    impl = Impl
    def testPlaySlideShow(self):
        self.impl.openCameraFolder()
        self.impl.openFist()
        assert self.impl.isPic(), "fail"
        for i in range(int(self.searchConf("TakePictures","take_pics_number"))-1):
            self.impl.playNextPic()
            assert self.impl.isPic(), "fail"


