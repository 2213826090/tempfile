# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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
@summary: Class for CompositionFallback operation
@since: 03/10/2016
@author: ZhangRox
'''
import time
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import qrcode_obj
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.graphics.common import id_generator, remove_temp_file
from testlib.common.base import getTmpDir


class CompositionFallbackImpl:

    '''
    classdocs
    '''

    def __init__(self):
        self.tmpdir = getTmpDir()
        self.device = g_common_obj.get_device()

    def dumpsys_surface_flinger(self):
        """
        """
        output = g_common_obj.adb_cmd_capture_msg(" dumpsys SurfaceFlinger | grep 'GLES |'")
        return output

    def open_reset_window(self):
        if not self.device(text="Apps").exists:
            self.device(scrollable=True).scroll.to(text="Language & input")
        self.device(text="Apps").click.wait()
        self.device(index=1, className="android.widget.ImageButton").click.wait()
        self.device(text="Reset app preferences").click.wait()

    def enable_disable_other_input_methods(self, status):
        if not self.device(text="Language & input").exists:
            self.device(scrollable=True).scroll.to(text="Language & input")
        self.device(text="Language & input").click.wait()
        self.device(text="Current Keyboard").click.wait()
        time.sleep(1)
        self.device(text="Choose keyboards").click.wait()
        time.sleep(1)
        if status.upper() == "ON":
            if self.device(text="Google Japanese Input").right(resourceId="android:id/switchWidget", checked=True) is None:
                self.device(text="Google Japanese Input").right(resourceId="android:id/switchWidget").click.wait()
            assert self.device(text="Google Japanese Input").right(resourceId="android:id/switchWidget", checked=True).exists,\
                "Enable the other input methods failed."
        elif status.upper() == "OFF":
            if self.device(text="Google Japanese Input").right(resourceId="android:id/switchWidget", checked=False) is None:
                self.device(text="Google Japanese Input").right(resourceId="android:id/switchWidget").click.wait()
            assert self.device(text="Google Japanese Input").right(resourceId="android:id/switchWidget", checked=False).exists,\
                "Disable the other input methods failed."

    def touch_goolge_search_bar(self):
        self.device(resourceId="com.google.android.googlequicksearchbox:id/text_container").click.wait()
        time.sleep(1)
        g_common_obj.adb_cmd_capture_msg("input keyevent 63")
        time.sleep(1)
        assert self.device(text="Change keyboard").exists, "The window 'Change keyboard' popups failed "
