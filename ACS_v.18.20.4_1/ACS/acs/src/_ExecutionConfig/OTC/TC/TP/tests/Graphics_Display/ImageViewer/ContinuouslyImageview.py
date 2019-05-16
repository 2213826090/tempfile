# -*- coding: utf-8 -*-
'''
Created on 2015-02-02
@author: junnanx.ding@intel.com
'''
from testlib.graphics.gits_test_template import GitsTestTemplate

class ContinuouslyImageview(GitsTestTemplate):

    @property
    def _config(self):
        return 'tests.tablet.gits_test_template.conf'

    def test_MoreThan1000Images(self):
        """test_ContinuouslyImageview_MoreThan1000Images"""
        self.commonTest(self._config)

    def test_MoreThan10BMPImages(self):
        """test_ContinuouslyImageView_MoreThan10BMPImages"""
        self.commonTest(self._config)

    def test_MoreThan10BMPImagesOfDifferentResolution(self):
        """test_ContinuouslyImageView_MoreThan10BMPImagesOfDifferentResolution"""
        self.commonTest(self._config)

    def test_MoreThan10GIFImages(self):
        """test_ContinuouslyImageView_MoreThan10GIFImages"""
        self.commonTest(self._config)

    def test_MoreThan10GIFImagesOfDifferentResolution(self):
        """test_ContinuouslyImageview_MoreThan10GIFImagesOfDifferentResolution"""
        self.commonTest(self._config)

    def test_MoreThan10JPEGImages(self):
        """test_ContinuouslyImageView_MoreThan10JPEGImages"""
        self.commonTest(self._config)

    def test_MoreThan10JPEGImagesOfDifferentResolution(self):
        """test_ContinuouslyImageview_MoreThan10JPEGImagesOfDifferentResolution"""
        self.commonTest(self._config)

    def test_MoreThan10PNGImages(self):
        """test_ContinuouslyImageView_MoreThan10PNGImages"""
        self.commonTest(self._config)

    def test_MoreThan10PNGImagesOfDifferentResolution(self):
        """test_ContinuouslyImageView_MoreThan10PNGImagesOfDifferentResolution"""
        self.commonTest(self._config)

    def test_MoreThan10WBMPImagesOfDifferentResolution(self):
        """test_ContinuouslyImageView_MoreThan10WBMPImagesOfDifferentResolution"""
        self.commonTest(self._config)

    def test_MoreThan10WEBPImagesOfDifferentResolution(self):
        """test_ContinuouslyImageView_MoreThan10WEBPImagesOfDifferentResolution"""
        self.commonTest(self._config)