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
@since: 11/24/2015
@author: Zhao, Xiangyi
"""

import re
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import logcat

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def play_video(self):
        return self._device(textContains="* Play video (TextureView)")

    @property
    def play(self):
        return self._device(text="Play")

class grafika:
    '''
    classdocs
    '''
    apk_name = "grafika-debug.apk"
    pkg_name = "com.android.grafika"
    activity_name = "com.android.grafika.MainActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch grafika-debug via adb am command
        """
        print "Launch grafika-debug by adb am"
        g_common_obj.launch_app_am(\
            grafika.pkg_name, grafika.activity_name)
        self.time_mark = logcat.get_device_time_mark()
        time.sleep(10)
        if self._device(text="Unable to generate content").exists:
            self._device(text="OK").click.wait()

    @staticmethod
    def stop_app_am():
        """ Stop grafika-debug via adb am command
        """
        print "Stop grafika-debug by adb am"
        g_common_obj.stop_app_am(grafika.pkg_name)

    def play_movie(self):
        '''
        Play Movie and catch logcat.
        '''
        self._device.press.recent()
        time.sleep(2)
        self._device.press.back()
        time.sleep(2)
        self._locator.play_video.click.wait()
        time.sleep(1)
        if not self._device(text="big_buck_bunny_480p.mp4").exists:
            assert False, "[FAILURE] can't find the test video."
        print "[Info] ---Play Movie"
        self._locator.play.click.wait()
        time.sleep(20)
        msg = logcat.get_device_log(self.time_mark, 'Grafika *:S')
        pattern = r"SurfaceTexture ready"
        match = re.search(pattern, msg)
        assert match, \
            "[FAILURE] can't match %s result in % s log" % (pattern, msg)

        pattern = r"starting movie"
        match = re.search(pattern, msg)
        assert match, \
            "[FAILURE] can't match %s result in % s log" % (pattern, msg)

        pattern = r"video=864x480"
        match = re.search(pattern, msg)
        assert match, \
            "[FAILURE] can't match %s result in % s log" % (pattern, msg)

    def uninstall_app(self):
        """ uninstall grafika-debug
        """
        print "uninstall grafika-debug"
        cmd = 'uninstall %s' % grafika.pkg_name
        g_common_obj.adb_cmd_common(cmd)

