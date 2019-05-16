# Copyright (C) 2015  Zhang,RongX Z <rongx.z.zhang@intel.com>
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

'''
@summary: Class for PhotosImpl operation
@since: 08/27/2015
@author: Zhang,RongX Z
'''
import os
import sys
import time
import tempfile
import random
import math
import re
import string
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from PIL import Image
from testlib.graphics.special_actions_impl import special_actions
from testlib.graphics.photos_impl import CONFIG_FILE

try:
    import pyqrcode
    import qrtools
except ImportError:
    pyqrcode = None
    qrtools = None

class QRcode:
    """
    Battery Test Impl Class
    """
    def __init__(self):
        self.device = g_common_obj.get_device()

    def prepare_img(self, tag='', name=''):
        '''
        deploy photo contents before run tests.
        :param tag: input config file tag name. such as: Pictures / Videos / PicturesFolder ..
        :param name: input head name for each content. such as: picture_001 ~ 005 / jpg_10resolution ..
        :return: full path of the img.
        '''
        arti_ = TestConfig().read(section='artifactory').get('location')
        arti = Artifactory(arti_)
        if not 'Folder' in tag:
            photo = TestConfig().read(CONFIG_FILE, tag).get(name)
            return arti.get(photo)
        else:
            getPhotos = TestConfig().read(CONFIG_FILE, tag).get(name)
            isolatePath = getPhotos.split(';')[0]
            isolateConts = getPhotos.split(';')[1]
            repoPath = isolatePath.split('\"')[1].replace('Path:', '').strip(' ')
            contents = isolateConts.split('\"')[1].replace('Content:', '').strip(' ').split(',')
            all = []
            for _ in contents:
                cont = _.strip(' ')
                contentPath = repoPath + '/' + cont
                localPath = arti.get(contentPath)
                all.append(localPath)
            return all

    def mark_image_qrcode(self, org_img, code='9876543210', count=2, margin=5):
        """mark qrcode to image"""
        mark_png = tempfile.mktemp(suffix='.png', prefix='qrcode_', dir='/tmp')
        out_img = tempfile.mktemp(suffix=os.path.splitext(org_img)[1],
                                  prefix='qrcode_', dir='/tmp')

        def get_scale(img_w, img_h):
            image_wide = img_w if img_w > img_h else img_h
            scales = int(math.ceil(image_wide / 220))
            return scales

        base_img = Image.open(org_img)
        base_w, base_h = base_img.size

        qr = pyqrcode.create(code)
        qr.png(mark_png, scale=get_scale(base_w, base_h))
        mark_img = Image.open(mark_png)
        mark_w, mark_h = mark_img.size

        def in_area(x, y):
            return x >= 0 and y >= 0 \
                and x + mark_w <= base_w and y + mark_w <= base_h

        position_map = []
        row = int(math.ceil(math.sqrt(count)))
        for i in range(row):
            for j in range(row):
                position_map.append((j, i))
        random.shuffle(position_map)

        relative_x = ((base_w - mark_w) / 2) - ((mark_w + margin) * int(math.ceil(row / 2)))
        relative_y = ((base_h - mark_h) / 2) - ((mark_h + margin) * int(math.ceil(row / 2)))

        marked_count = 0
        for each in position_map:
            draw_x = relative_x + (mark_w + margin) * each[0]
            draw_y = relative_y + (mark_h + margin) * each[1]
            if in_area(draw_x, draw_y):
                base_img.paste(mark_img, (draw_x, draw_y))
                marked_count += 1
                if marked_count >= count:
                    break
        assert marked_count, \
            "[FAILURE] Failed mark qrcode"

        base_img.save(out_img)
        self.remove_temp_file(mark_png)
        return out_img

    def decode_image_qrcode(self, img_file):
        """decode image qrcode"""
        qr = qrtools.QR()
        ret = qr.decode(img_file)
        print "[Debug] qrdecode ret: %s" % (ret)
        return ret, qr.data

    def verify_qrcode_marked_image(self, qrcode, set_wallpaper=True):
        """
        campare qrcode with the qrcode decoded after image being pushed to device or setting to be wallpaper.
        args--set_wallpaper:value is True,False
            --qrcode:qrcode to be compared
        """
        home_screen = tempfile.mktemp(suffix='.png', prefix='screen_', dir='/tmp')
        decode = ''
        if set_wallpaper == True:
            self.device.press.home()
        for _ in range(5):
            self.device.screenshot(home_screen)
            ret, decode = self.decode_image_qrcode(home_screen)
            print "[Debug]", ret, decode
            if ret is False or decode == 'NULL':
                x = self.device.info.get("displayWidth")
                y = self.device.info.get("displayHeight")
                center_x = x / 2 ; center_y = y / 2
                print "QRcode not found, try swipe up."
                self.device.swipe(center_x, center_y, center_x, (center_y - 200))
                time.sleep(1)
                if ret is False or decode == 'NULL':
                    print "QRcode still not found, try zoom in."
                    self.device().gesture((x/2,y/2-10),(x/2,(y/2+10))).to((x/2, (y/2-20)),(x/2, (y/2+30)), steps=10)
                    time.sleep(2)
                continue
            break
        print "[Debug] verify_imageview qrcode:%s decode:%s" % (qrcode, decode)
        assert decode == qrcode, \
            "[FAILURE] Failed diff qrcode:%s decode:%s" % (qrcode, decode)
        self.remove_temp_file(home_screen)

    def remove_temp_file(self, fullname):
        """remove temp file"""
        try:
            if os.path.exists(fullname):
                os.remove(fullname)
        except os.error:
            print "[Warning] %s" % (sys.exc_info()[1])

    def id_generator(self, size=10, chars=string.ascii_uppercase + string.digits):
        """generate random id"""
        return ''.join(random.choice(chars) for _ in range(size))
