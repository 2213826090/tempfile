#!/usr/bin/env python

#######################################################################
#
# @filename:    file_utils.py
# @description: File operations test utils
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################
from PIL import Image
from PIL import ImageChops
import math
import operator
import fileinput
import sys



def image_diff(im1, im2):

    image1 = Image.open(im1)
    image2 = Image.open(im2)

    return ImageChops.difference(image2, image1)


def rms_diff(im1, im2):
    """
        Calculates the root-mean-square difference between two images
    """
    image1 = Image.open(im1)
    image2 = Image.open(im2)
    h1 = image1.histogram()
    h2 = image2.histogram()

    return math.sqrt(reduce(operator.add,
                            map(lambda a,b: (a-b)**2, h1, h2))/len(h1))


def replace_string(old, new, file_to_search):
    file_content = open(file_to_search).read()
    if old in file_content:
        file_content = file_content.replace(old, new)
        f = open(file_to_search, 'w')
        f.write(file_content)
        f.flush()
        f.close()
