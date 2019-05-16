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
@summary: PhotosExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import os
import time
import difflib
import filecmp
import tempfile

from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.photos.photos_impl import PhotosImpl
from testlib.graphics.common import logcat
from testlib.graphics.extend_systemui_impl\
    import SystemUiExtendImpl as SystemUi
from testlib.graphics.common import qrcode_obj
from testlib.graphics.common import id_generator, remove_temp_file
from testlib.graphics.special_actions_impl import special_actions
from testlib.graphics.QRcode_impl import QRcode


class PhotosExtendImpl(PhotosImpl):

    """ Photos uiautomator """

    config_file = 'tests.common.photos.conf'

    HostPhotoPagerActivity = "com.google.android.apps.photos.viewer.pager.HostPhotoPagerActivity"

    class HomeUI(object):

        """Home UI Elements"""

        def __init__(self, device):
            self.device = device

        @property
        def list_item(self):
            return self.device(className='android.view.View')

        @property
        def first_item(self):
            return self.device(resourceId="com.google.android.apps.plus:id/tile_row")[0].child(index=0)

    class ViewUI(object):

        """View UI elements"""

        def __init__(self, device):
            self.device = device

        @property
        def play(self):
            return self.device(resourceId="com.google.android.apps.plus:"
                               "id/photo_view_pager")\
                .child(description="Play video")

        @property
        def is_video(self):
            return self.device(resourceId="com.google.android.apps.plus:"
                               "id/photo_view_pager")\
                .child(description="Play video")\
                .exists

        def show_more(self):
            if not self.device(description="More options").exists\
                    and not self.device(text="Help").exists:
                self.device().click()
            self.device(description="More options").click()

        @property
        def option_slideshow(self):
            self.show_more()
            return self.device(textContains="Slideshow")

        @property
        def option_set_as(self):
            self.show_more()
            return self.device(textContains="Set as")

    def __init__(self):
        super(PhotosExtendImpl, self).__init__()
        self.configer = TestConfig()
        self.config = self.configer.read(self.config_file, "Photos")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.dut_photos_dir = self.config.get("photos_dir")
        self.dut_video_dir = self.config.get("video_dir")
        self.arti = Artifactory(self.config.get("artifactory_location"))
        self._home = PhotosExtendImpl.HomeUI(self.d)
        self._view = PhotosExtendImpl.ViewUI(self.d)
        self.qrcodeImpl = QRcode()

    def launch(self):
        """launch APP"""
        g_common_obj.launch_app_am('com.google.android.apps.plus',
                                   'com.google.android.apps.photos.phone.PhotosHomeActivity')
        time.sleep(4)
        if self.d(text="Later").exists:
            self.d(text="Later").click()
            time.sleep(2)
        time.sleep(5)
        self.d.wait.update()

    def click_list_item(self, index):
        """click list item"""
        print "[Debug] click_list_item index:%s" % (index)
        self._home.list_item[index].click()

    def find_video(self):
        """find video item"""
        print "[Debug] find_video"
        for i in range(self._home.list_item.count):
            self._home.list_item[i].click()
            time.sleep(2)
            _, activity = SystemUi.get_current_focus()
            if activity != self.HostPhotoPagerActivity:
                continue
            if self._view.is_video:
                self.d.press.back()
                time.sleep(2)
                print "video index:%s" % (i)
                return i
            self.d.press.back()
        assert False, "There is no video items"

    def find_photo(self):
        print "[Debug] find_photo"
        for i in range(self._home.list_item.count):
            self._home.list_item[i].click()
            time.sleep(2)
            _, activity = SystemUi.get_current_focus()
            if activity != self.HostPhotoPagerActivity:
                continue
            if not self._view.is_video:
                self.d.press.back()
                time.sleep(2)
                print "photo index:%s" % (i)
                return i
            self.d.press.back()
        assert False, "There is no photo items"

    def setup(self):
        """setup environment"""
        self.launch()
        time.sleep(2)
        device = self.d
        if device(textContains="No thanks").exists:
            device(textContains="No thanks").click()

        special_actions.setup()

    def play_video(self, secs=None, timeout=None):
        """click play video"""

        print "[Debug] play_video"
        mark_time = logcat.get_device_time_mark()
        pre_dump = self.d.dump()
        pre_focus = SystemUi.get_current_focus()
        print pre_focus

        self._view.play.click.wait()
        if secs:
            time.sleep(secs)
        end_dump = self.d.dump()
        end_focus = SystemUi.get_current_focus()
        print end_focus

        seq_match = difflib.SequenceMatcher(None, pre_dump, end_dump)
        ratio = round(seq_match.ratio(), 2) * 100
        print "ratio:%s" % ratio

        if ratio == 100 and pre_focus == end_focus:
            assert False, "[FAILURE] Play video No any reply"
        if not secs:
            start_time = time.time()
            play_done = False
            while time.time() - start_time < timeout:
                time.sleep(10)
                if self._view.play.exists:
                    play_done = True
                    break
            assert play_done, "[FAILURE] Play video time out"

        errmsg = logcat.get_device_log(mark_time, filters='MediaPlayer:E *:S')
        assert not errmsg, "[FAILURE] Found Errors in Logcat during Play Video\n%s" % (errmsg)

    def check_video_icons(self):
        x = self.d.info.get("displayWidth")
        y = self.d.info.get("displayHeight")
        self.d.click(x / 2, y / 2)
        time.sleep(3)
        assert self.d(resourceId="com.google.android.apps.plus:id/videoplayer").exists, "Video is not shown on the screen"


    def push_pictures(self, count=1, exts=None):
        remote_pictures = []
        pictures = self.configer.read(self.config_file, "Pictures")
        if exts:
            pictures = {k: v for k, v in pictures.iteritems()
                        if v.lower().endswith(exts)}
        if count > len(pictures.items()):
            assert False, \
                "[FAILURE] There are not enough resource."

        for _, value in pictures.iteritems():
            pic_path = self.arti.get(value)
            ret = g_common_obj.push_file(pic_path, self.dut_photos_dir)
            remote_pictures.append(self.dut_video_dir + '/' + os.path.basename(pic_path))
            assert ret, 'Failed push %s' % (pic_path)
            count -= 1
            if count == 0:
                break

        cmd = \
            "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s"\
            % (self.dut_photos_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print "[Debug] pushed %s" % (remote_pictures)
        return remote_pictures

    def push_qrcode_pictures(self, count=1, exts=None, mark_count=1):
        remote_pictures = []
        pictures = self.configer.read(self.config_file, "Pictures")
        if exts:
            pictures = {k: v for k, v in pictures.iteritems()
                        if v.lower().endswith(exts)}
        if count > len(pictures.items()):
            assert False, \
                "[FAILURE] There are not enough resource."

        for _, value in pictures.iteritems():
            pic_path = self.arti.get(value)
            qrcode = id_generator()
            pic_path = self.qrcodeImpl.mark_image_qrcode(pic_path, code=qrcode, count=mark_count)
            ret = g_common_obj.push_file(pic_path, self.dut_photos_dir)

            remote_pictures.append({'qrcode': qrcode,
                                    'path': self.dut_video_dir + '/' + os.path.basename(pic_path)})
            assert ret, 'Failed push %s' % (pic_path)
            remove_temp_file(pic_path)
            count -= 1
            if count == 0:
                break

        cmd = \
            "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s"\
            % (self.dut_photos_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print "[Debug] pushed %s" % (remote_pictures)
        return remote_pictures

    def push_specified_qrcode_pictures(self, count=1, exts=None, mark_count=1):
        remote_pictures = []
        pictures = self.configer.read(self.config_file, "Pictures")
        if exts:
            pictures = {k: v for k, v in pictures.iteritems()
                        if v.lower().endswith(exts)}
        if count > len(pictures.items()):
            assert False, \
                "[FAILURE] There are not enough resource."

        for _, value in pictures.iteritems():
            pic_path = self.arti.get(value)
            qrcode = "ABCDEFGHIJ"
            pic_path = qrcode_obj.mark_image_qrcode(pic_path, code=qrcode, count=mark_count)
            ret = g_common_obj.push_file(pic_path, self.dut_photos_dir)

            remote_pictures.append({'qrcode': qrcode,
                                    'path': self.dut_video_dir + '/' + os.path.basename(pic_path)})
            assert ret, 'Failed push %s' % (pic_path)
            remove_temp_file(pic_path)
            count -= 1
            if count == 0:
                break

        cmd = \
            "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s"\
            % (self.dut_photos_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print "[Debug] pushed %s" % (remote_pictures)
        return remote_pictures

    def push_videos(self, count=1, like=None, exts=None):
        remote_videos = []
        videos = self.configer.read(self.config_file, "Videos")
        if like:
            videos = {k: v for k, v in videos.iteritems()
                      if v.lower().find(like.lower()) != -1}
        if exts:
            videos = {k: v for k, v in videos.iteritems()
                      if v.lower().endswith(exts)}
#         if like and exts:
#             videos = { k:v for k, v in videos.iteritems() if os.path.basename(v) == like + exts }

        if count > len(videos.items()):
            assert False, \
                '[FAILURE] The video is not in config file ' + self.config_file

        for _, value in videos.iteritems():
            pic_path = self.arti.get(value)
            ret = g_common_obj.push_file(pic_path, self.dut_video_dir + '/' + os.path.basename(pic_path))
            assert ret, 'Failed push %s' % (pic_path)
            remote_videos.append(self.dut_video_dir + '/' + os.path.basename(pic_path))
            count -= 1
            if count == 0:
                break

        cmd = \
            "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s"\
            % (self.dut_video_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        print "[Debug] pushed %s" % (remote_videos)
        return remote_videos

    def slideshow(self, duration):
        print "[Debug] slideshow"
        pre_screen = tempfile.mktemp(suffix='.png', prefix='screen_', dir='/tmp')
        end_screen = tempfile.mktemp(suffix='.png', prefix='screen_', dir='/tmp')

        def action():
            self._home.first_item.click()
            self.d.wait.update()
            self._view.option_slideshow.click()

        action()
        start_time = time.time()
        self.d.screenshot(pre_screen)
        while time.time() - start_time < duration:
            time.sleep(6)
            self.d.wait.idle(timeout=3000)
            self.d.screenshot(end_screen)
            if filecmp.cmp(pre_screen, end_screen):
                self.d.press.back()
                action()
            self.d.wait.idle(timeout=3000)
            self.d.screenshot(pre_screen)
        self.d.press.back()

    def setwallpaper(self):
        print "[Debug] setwallpaper"
        if self.d(text="More options").exists:
            self.d(text="More options").click()
        self._view.option_set_as.click()
        if self.d(text='More').exists:
            self.d(text='More').click()
            time.sleep(1)
        self.d(text='Wallpaper').click()
        time.sleep(3)
        self.d(text='Set wallpaper').click()
        time.sleep(3)
        if self.d(text="Set wallpaper").exists:
            self.d(text='Set wallpaper').click()
            time.sleep(6)

    def verify_wallpaper(self, qrcode):
        home_screen = tempfile.mktemp(suffix='.png', prefix='screen_', dir='/tmp')
        self.d.press.home()
        time.sleep(1)
        self.d.screenshot(home_screen)
        _, decode = qrcode_obj.decode_image_qrcode(home_screen)
        print "[Debug] verify_wallpaper qrcode:%s decode:%s" % (qrcode, decode)
        assert decode == qrcode, \
            "[FAILURE] Failed verify wallpaper qrcode:%s decode:%s" % (qrcode, decode)
        remove_temp_file(home_screen)

    def verify_imageview(self):
        home_screen = tempfile.mktemp(suffix='.png', prefix='screen_', dir='/tmp')
        decode = ''

        for _ in range(10):
            self.d.screenshot(home_screen)
            ret, decode = qrcode_obj.decode_image_qrcode(home_screen)
            print "[Debug]", ret, decode

            if ret is False or decode == 'NULL':
                special_actions.zoom_in()
                continue
            break

        remove_temp_file(home_screen)
        return decode

    def clean(self):
        for each in [self.dut_photos_dir, self.dut_video_dir]:
            cmd = \
                "rm -rf %s/*; sync; "\
                "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s"\
                % (each, each)
            g_common_obj.adb_cmd_capture_msg(repr(cmd))
        special_actions.clean()
