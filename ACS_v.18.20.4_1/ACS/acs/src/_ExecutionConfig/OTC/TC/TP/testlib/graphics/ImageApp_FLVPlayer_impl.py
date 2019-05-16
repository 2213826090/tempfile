# Copyright (C) 2015  Zhao, XiangyiX <xiangyix.zhao@intel.com>
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
@summary: Class for ImageApp JotaTextEditor
@since: 09/07/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.util.common import g_common_obj

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def play_video(self):
        return self._device(resourceId="air.br.com.bitlabs.FLVPlayer:id/list_view")\
            .child(className="android.widget.LinearLayout")

class FLVVideoPlayer:
    '''
    classdocs
    '''

    pkg_name = "air.br.com.bitlabs.FLVPlayer"
    activity_name = "br.com.bitlabs.videoplayer.TabActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch FLVVideoPlayer via adb am command
        """
        print "Launch FLVVideoPlayer by adb am"
        g_common_obj.launch_app_am(\
            FLVVideoPlayer.pkg_name, FLVVideoPlayer.activity_name)
        time.sleep(2)

    @staticmethod
    def stop_app_am():
        """ Stop FLVVideoPlayer via adb am command
        """
        print "Stop FLVVideoPlayer by adb am"
        g_common_obj.stop_app_am(FLVVideoPlayer.pkg_name)

    @staticmethod
    def clean_workaround():
        """ Clean the workaround to avoid other file affect.
        """
        cmd = 'rm -rf sdcard/Pictures/*'
        g_common_obj.adb_cmd(cmd)

    def refresh_sdcard(self):
        """ refresh sdcard
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard'
        g_common_obj.adb_cmd(cmd)

    def play_video(self):
        """ Play video in FLV.
        """
        self._locator.play_video.click.wait()
        time.sleep(10)
        g_common_obj.assert_exp_happens()

    def uninstall_app(self):
        """ uninstall FLVVideoPlayer
        """
        print "uninstall FLVVideoPlayer"
        cmd = 'uninstall %s' % FLVVideoPlayer.pkg_name
        g_common_obj.adb_cmd_common(cmd)
