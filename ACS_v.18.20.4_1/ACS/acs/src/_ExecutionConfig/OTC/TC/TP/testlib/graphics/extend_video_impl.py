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
@since: 01/25/2016
@author: Zhao, Xiangyi
"""

import time
from testlib.util.config import TestConfig
# from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import *
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import artifactory, pkgmgr


class VideoExtendImpl(object):

    """multimedia_setting Video Extend Implement"""

    def __init__(self):
        self.d = g_common_obj.get_device()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

        test_config = TestConfig()
        conf_file = 'tests.tablet.artifactory.conf'
        self.config = test_config.read(conf_file, "content_video_apk")

    def appPrepare(self):

        file_path = self.config.get('file_path') + self.config.get('file_name')
        cache_path = artifactory.get(file_path)
        pkgmgr.apk_install(cache_path)

        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def launchVideoApp(self):
        for _ in range(3):
            g_common_obj.launch_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer", \
                                       "otc.intel.com.otcvideoplayer.InitActivity")
            time.sleep(3)
            if self.d(textContains="OtcVideoPlayer").exists:
                return
        assert self.d(textContains="OtcVideoPlayer").exists, "launch video app failed!"

    def videoPlayBack(self, push_path):
        self.d(packageName="videoplayer.app.instrument.otc.intel.com.otcvideoplayer", \
               className="android.widget.ImageButton").click()
        time.sleep(1)
        self.d(text="Open a local video").click()
        time.sleep(1)

        self.d(className="android.widget.EditText").set_text(push_path)
        time.sleep(1)

        self.d(text="OK").click()
        time.sleep(1)
        assert not self.d(text="Can't play this video.").exists, "show Can't play video."
        assert not self.d(textContains="Network not connected").exists, "Network not connected."
        assert not self.d(textContains="The remote media file is not reachable").exists, "The remote media file is not reachable."

    def get_play_time(self, s=60, flag=0):
        logger.debug("get_play_time start")
        for i in range(s):
            if self.d(resourceId="android:id/time_current").exists:
                try:
                    ct = self.d(resourceId="android:id/time_current").text
                    tt = self.d(resourceId="android:id/time").text
                    ct = setTimeToSec(ct)
                    tt = setTimeToSec(tt)
                    return ct, tt
                except Exception as e:
                    print "Error:", e
                    continue
            else:
                print str(i) + " times,don't find current time or total time"
                assert not self.d(textContains="Network not connected").exists, "Network not connected."
                assert not self.d(textContains="The remote media file is not reachable").exists, "The remote media file is not reachable."
                assert not self.d(textContains="error").exists, "Play error!"
                time.sleep(1)
                if flag == 0:
                    self.d.click(self.x / 2, self.y / 2)
                else:
                    self.d.click(self.y / 2, self.x / 2)
        assert not self.d(textContains="OTC Alarm is triggered").exists, "OTC Alarm is triggered! wait time=%d" % (s)
        assert 0, "Play error! playback timeout %s s, network problem." % (s)
        return -1

    def checkVideoPlayBackWithComparePicture(self, stoptime, bigfileskiptime=0):
        logger.debug("checkVideoPlayBackWithComparePicture start")
        stoptime = int(stoptime)
        video_current_time = "android:id/time_current"
        video_total_time = "android:id/time"
        s = 60
        for i in range(s):
            if self.d(resourceId=video_current_time).exists and self.d(resourceId=video_total_time).exists:
                print str(i) + " times, find current time and total time!!"
                tt = self.d(resourceId=video_total_time).text
                tt = setTimeToSec(tt)
                i = -1
                break
            else:
                print str(i) + " times,don't find current time or total time"
                time.sleep(1)
                self.d.click(self.x / 2, self.y / 2)
        if i != -1:
            assert 0, "Play error! playback timeout %s s, network problem." % (s)

        if stoptime > tt:
            if tt == 0:
                tt = 18
            stoptime = tt
        stoptime -= 5

        node = (0, self.y / 2 - 100, self.x, self.y / 2 + 100)
        img_0 = otcImage.cropScreenShot(self.d, node)

        timeNow = time.time()
        ct = 0
        while ct <= stoptime:
            time.sleep(5)
            if self.d(textContains="Completed").exists:
                break
            img_1 = otcImage.cropScreenShot(self.d, node)
            ssim = float(otcImage.calc_similar(img_0, img_1))
            img_0 = img_1
            print "ssim is ", ssim
            if ssim > 0.99:
                if bigfileskiptime == 0 or ct > bigfileskiptime:
                    assert ssim <= 0.99, "video playback has been False"

            if self.d(textContains="Completed").exists:
                break
            ct = time.time() - timeNow

    def checkVideoPlayBackComplete(self, s=900):
        tt = -1
        for _ in range(s / 10):
            time.sleep(10)
            if self.d(textContains="Completed").exists:
                return 1
            else:
                try:
                    if tt == -1:
                        for _ in range(10):
                            if self.d(resourceId="android:id/time").exists:
                                break
                            self.d.click(self.x / 2, self.y / 2)
                        tt = self.d(resourceId="android:id/time").text
                        tt = setTimeToSec(tt)
                    for _ in range(10):
                        if self.d(resourceId="android:id/time_current").exists:
                            break
                        self.d.click(self.x / 2, self.y / 2)
                    ct = self.d(resourceId="android:id/time_current").text
                    ct = setTimeToSec(ct)
                except Exception as e:
                    if self.d(textContains="Completed").exists:
                        return 1
                    else:
                        assert False, e
                if ct == tt:
                    assert not self.d(textContains="error").exists or not self.d(textContains="fail").exists, "Play back error! please check it."
                    return 1
        return 0
