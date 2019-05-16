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
@since: 08/31/2015
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
    def sign_in(self):
        return self._device(text="Sign In")


class JotaTextEditor:
    '''
    classdocs
    '''

    pkg_name = "jp.sblo.pandora.jota"
    activity_name = "jp.sblo.pandora.jota.Main"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch JotaTextEditor via adb am command
        """
        print "Launch JotaTextEditor by adb am"
        g_common_obj.launch_app_am(\
            JotaTextEditor.pkg_name, JotaTextEditor.activity_name)
        time.sleep(2)
        if self._device(text="Introduction of Jota+.").exists:
            self._device(text="OK").click()
            time.sleep(1)

    @staticmethod
    def stop_app_am():
        """ Stop JotaTextEditor via adb am command
        """
        print "Stop JotaTextEditor by adb am"
        g_common_obj.stop_app_am(JotaTextEditor.pkg_name)

    @staticmethod
    def clean_workaround():
        """ Clean the workaround to avoid other file affect.
        """
        cmd = 'rm -rf sdcard/Pictures/*'
        g_common_obj.adb_cmd(cmd)

    def edit_text(self):
        """ Edit text in Jota.
        """
        cmd = "input text test"
        g_common_obj.adb_cmd(cmd)
        print "Excute:Select all"
        self._device(resourceId="jp.sblo.pandora.jota:id/menu_edit").click.wait()
        time.sleep(1)
        self._device(text="Select all").click.wait()
        time.sleep(1)
        print "Execute:Cut"
        self._device(text=" Cut ").click.wait()
        time.sleep(1)
        print "Execute:Undo"
        self._device(text="Undo").click.wait()
        time.sleep(1)
        print "Execute:Save"
        self._device(text="Save").click.wait()
        time.sleep(1)
        if not self._device(text="Pictures/").exists:
            self._device(scrollable=True).scroll.to(text="Pictures/")
            self._device(text="Pictures/").click.wait()
            time.sleep(1)
        else:
            self._device(text="Pictures/").click.wait()
            time.sleep(1)
        self._device(text="OK").click.wait()
        time.sleep(2)
        print "Check text file"
        assert self.check_text(), "Check text file failed!"

    def check_text(self):
        """ check text which made by Jota.
        """
        cmd = "cat sdcard/Pictures/test.txt | grep test; echo $?"
        result = g_common_obj.adb_cmd_capture_msg(cmd)
        print "check result:", result
        if result[-1] == '0':
            return True
        else:
            return False


    def uninstall_app(self):
        """ uninstall JotaTextEditor
        """
        print "uninstall JotaTextEditor"
        cmd = 'uninstall %s' % JotaTextEditor.pkg_name
        g_common_obj.adb_cmd_common(cmd)
