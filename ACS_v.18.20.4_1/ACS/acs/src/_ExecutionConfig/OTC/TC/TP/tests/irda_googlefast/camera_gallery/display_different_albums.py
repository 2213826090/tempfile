# coding: UTF-8
import os, sys
import time
from testlib.common.uiatestbase import UIATestBase2
from testlib.photos.photos_impl import PhotosImpl as Impl

class DisplayDifferentAlbums(UIATestBase2):
    impl = Impl
    def testDisplayDifferentAlbums(self):
        self.impl.launchPhotos()
        self.impl.openPhotosFolders()
        self.impl.checkDifferentAlbums()
