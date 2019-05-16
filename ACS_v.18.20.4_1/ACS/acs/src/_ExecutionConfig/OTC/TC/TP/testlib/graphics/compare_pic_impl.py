# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

"""
@summary: SpecialActionsImpl class
@since: 05/27/2015
@author: Zhao, Xiangyi
"""

from testlib.util.common import g_common_obj
from testlib.util.log import Logger
from PIL import Image, ImageChops
import math
import operator
import cv2
import numpy as np
import os


LOG = Logger.getlogger(__name__)


class ComparePicImpl(object):

    """ComparePicImpl"""

    def __init__(self):
        self._device = g_common_obj.get_device()
        self.actions = []

    def compare_pic(self, filepath1, filepath2):
        """
        compare 2 pictures, when the 2 pictures are totally same, rms return 0,
        else the greater the deffrence image is,the greater the rms values;
        RMS(Root Mean Square) is the standard deviation defined in an histogram;
        To know more about rms, see at http://mathworld.wolfram.com/Root-Mean-Square.html
        """
        img1 = Image.open(filepath1)
        img2 = Image.open(filepath2)

        try:
            diff = ImageChops.difference(img1, img2)
        except ValueError:
            return False

        h = ImageChops.difference(img1, img2).histogram()
        sq = (value * ((idx % 256) ** 2) for idx, value in enumerate(h))
        sum_of_squares = sum(sq)

        rms = math.sqrt(sum_of_squares / float(img1.size[0] * img1.size[1]))
        LOG.info("rms value is: %f" % rms)

        return rms

    def compare_pic_with_redrawLhist(self, filepath1, filepath2):
        """Redraw the gray scale histogram h1 and h2,compare the h1 and h2 by rms;
        compare 2 pictures, when the 2 pictures are totally same, rms return 0,
        else the greater the deffrence image is,the greater the rms values;
        RMS(Root Mean Square) is the standard deviation defined in an histogram;
        To know more about rms, see at http://mathworld.wolfram.com/Root-Mean-Square.html
        rms=(((h1[0]-h2[0])**2+(h1[1]-h2[1])**2+...(h1[n]-h2[n])**2)/(n+1))**0.5"""
        print "compare 2 pictures"
        h1 = self.redrawLhist(filepath1)
        h2 = self.redrawLhist(filepath2)
        print h1
        print h2

        rms = math.sqrt(reduce(operator.add, list(map(lambda a, b: (a - b) ** 2, h1, h2))) / len(h1))
        return rms

    def redrawLhist(self, filepath):
        """Caculate the image histogram by use PIL and return the Gray scale histogram."""
        image = Image.open(filepath)
        h = image.convert("L")
        width, height = h.size
        pix = h.load()
        hist = [0] * 256
        for w in xrange(width):
            for h in xrange(height):
                p = pix[w, h]
                hist[p] = hist[p] + 1
        return hist

    def calcAndDrawHist(self, image, color):
        """Caculate the image histogram by use cv2.calcHist func and draw the histogram."""
        hist = cv2.calcHist([image], [0], None, [256], [0.0, 255.0])
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(hist)
        histImg = np.zeros([256, 256, 3], np.uint8)
        hpt = int(0.9 * 256);

        for h in range(256):
            intensity = int(hist[h] * hpt / maxVal)
            cv2.line(histImg, (h, 256), (h, 256 - intensity), color)
        bin_counts, bin_edges = np.histogram(histImg)
        return bin_counts.astype('float32');

    def get_pic_blue(self, filepath):
        """calculate the B channel of the filepath pic,and draw it to histogram,return the histogram array"""
        image = cv2.imread(filepath)
        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        b, _, _ = cv2.split(img)
        histImgB = self.calcAndDrawHist(b, [255, 0, 0])
        return histImgB

    def get_pic_green(self, filepath):
        """calculate the G channel of the filepath pic,and draw it to histogram,return the histogram array"""
        img = cv2.imread(filepath)
        _, g, _ = cv2.split(img)
        histImgG = self.calcAndDrawHist(g, [0, 255, 0])
        return histImgG

    def get_pic_red(self, filepath):
        """calculate the R channel of the filepath pic,and draw it to histogram,return the histogram array"""
        img = cv2.imread(filepath)
        _, _, r = cv2.split(img)
        histImgR = self.calcAndDrawHist(r, [0, 0, 255])
        return histImgR

    def get_pic_RGB(self, filepath):
        img = cv2.imread(filepath)
        h = np.zeros((256, 256, 3))

        bins = np.arange(256).reshape(256, 1)
        color = [ (255, 0, 0), (0, 255, 0), (0, 0, 255) ]
        for ch, col in enumerate(color):
            originHist = cv2.calcHist([img], [ch], None, [256], [0, 256])
            cv2.normalize(originHist, originHist, 0, 255 * 0.9, cv2.NORM_MINMAX)
            hist = np.int32(np.around(originHist))
            pts = np.column_stack((bins, hist))
            cv2.polylines(h, [pts], False, col)

        h = np.flipud(h)

        bin_counts, bin_edges = np.histogram(originHist)
#         bin_counts, bin_edges = histogram(h)
        return bin_counts.astype('float32');

    def compare_pic_file_correlation(self, image1, image2):
        """compare 2 pictures, return correlation value range is -1~1;
        when the 2 pictures are totally same, correlation return 1.0
        the greater the diffrence image is,the smaller the correlation values
        totally different the correlation is -1;if correlation is 0,it means the 2 pictures is no association(radom)"""
        print "compare 2 pictures correlation"
        correlation = cv2.compareHist(image1, image2, cv2.cv.CV_COMP_CORREL)
        print "correlation is: %s" % (correlation)
        return correlation

    def compare_pic_file_chisquare(self, image1, image2):
        """compare 2 pictures, return chisquare value range is 0~max(max depends on histograms size);
        when the 2 pictures are totally same, correlation return 0.0
        the greater the deffrence image is,the larger the chisquare values"""
        print "compare 2 pictures chisquare"
        chisquare = cv2.compareHist(image1, image2, cv2.cv.CV_COMP_CHISQR)
        print "chisquare is: %s" % (chisquare)
        return chisquare

    def compare_pic_file_intersection(self, image1, image2):
        """compare 2 pictures, return chisquare value range is 0~max;
        when the 2 pictures are totally same, correlation return a value
        the greater the deffrence image is,the smaller the intersection values"""
        print "compare 2 pictures intersection"
        intersection = cv2.compareHist(image1, image2, cv2.cv.CV_COMP_INTERSECT)
        print "intersection is: %s" % (intersection)
        return intersection

    def compare_pic_file_bhattacharyya(self, image1, image2):
        """compare 2 pictures, return chisquare value range is 0~1;
        when the 2 pictures are totally same, correlation return 0.0
        the greater the deffrence image is,the larger the bhattacharyya values"""
        print "compare 2 pictures bhattacharyya"
        bhattacharyya = cv2.compareHist(image1, image2, cv2.cv.CV_COMP_BHATTACHARYYA)
        print "bhattacharyya is: %s" % (bhattacharyya)
        return bhattacharyya

    def extract_strings_from_screen_shot(self, displayId=0):
        try:
            import pytesseract
            from testlib.graphics.common import multi_display
        except ImportError:
            raise Exception("Fail to import tools")
        screenshot = multi_display.get_screenshot_forMultiDisplay(displayId)
        screenshot_img = Image.open(screenshot)
        screenshot_img.load()
        screenshot_text = pytesseract.image_to_string(screenshot_img)
        os.system('rm -rf %s' % screenshot)
        LOG.info("Got image strings from: %s\n%s" % (screenshot, screenshot_text))
        return screenshot_text

    def extract_strings_from_croped_screen_shot(self, displayId=0, sx=0, sy=0, ex=100, ey=100):
        try:
            import pytesseract
            from testlib.graphics.common import multi_display
        except ImportError:
            raise Exception("Fail to import tools")
        screenshot = multi_display.get_screenshot_forMultiDisplay(displayId)
        # new_screenshot = screenshot.split('.png')[0] + '_new' + '.png'
        img = Image.open(screenshot)
        img2 = img.crop((sx, sy, ex, ey))
        img2.save(screenshot)
        screenshot_img = Image.open(screenshot)
        screenshot_img.load()
        screenshot_text = pytesseract.image_to_string(screenshot_img)
        os.system('rm -rf %s' % screenshot)
        LOG.info("Got image strings from: %s\n%s" % (screenshot, screenshot_text))
        return screenshot_text

compare_pic = ComparePicImpl()
