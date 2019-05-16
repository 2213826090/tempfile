# coding: UTF-8
import os, sys
import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.photos.photos_impl import PhotosImpl as Impl

class EditPicture(UIATestBase2):
    impl=Impl
    def testEditPicture(self):
        self.impl.openCameraFolder()
        self.impl.openFist()
        self.impl.editPicture()




