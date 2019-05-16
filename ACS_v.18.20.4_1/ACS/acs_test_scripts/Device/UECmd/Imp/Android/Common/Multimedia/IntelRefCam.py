"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: camera implementation
:since: 10/04/2015
:author: pblunie
"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Multimedia.IIntelRefCam import IIntelRefCam
import time


class IntelRefCam(BaseV2, IIntelRefCam):
    """
    Class that handles all RefCamera operations

    For adding settings, refer to document "Full Supported Features" on wiki:
        https://wiki.ith.intel.com/display/VIEDglob/ICG+Reference+Camera
    """

    def __init__(self, device):
        BaseV2.__init__(self, device)
        self._camera_pkg = self._device.get_config("CameraPackageName", "").lower()
        self._logger.debug("%s camera requested" % self._camera_pkg)

    def get_camera_package(self):
        return self._camera_pkg

    def stop(self):
        """
        Close RefCam
        """
        cmd = "am force-stop %s" % self._camera_pkg
        self._exec("adb shell %s" % cmd, 10)

    def launch(self):
        """
        Launch RefCam in Video mode with configuration
        """
        # Grant runtime permissions (if any)
        self._device.grant_runtime_permissions(self._camera_pkg)

        # -W to wait for completion and -S to kill the camera first
        cmd = "am start -W -S %s/.CameraActivity" % self._camera_pkg
        self._exec("adb shell %s" % cmd, 10)

    def capture(self):
        """
        Push camera button:
            - Start video recording in video mode
            - Take picture in Single mode
        """
        self._exec("adb shell input keyevent CAMERA")

    def user_mode_image_capture(self, delay, sleep_between_capture, nb_capture):
        """
        Launch script to control camera with delay
        Camera must have been started in Single mode with launch API.
        """

        usermodedir = "/data/usermode"
        self._exec("adb shell mkdir %s" % usermodedir)
        tmpscript = "%s/user_mode_image_capture.sh" % usermodedir

        cmd_list = "sleep %d;\n" \
                   "for i in `seq 1 %d`;\n"\
                   "do input keyevent ENTER;\n"\
                   "sleep %d;\n"\
                   "done" % (delay, nb_capture, sleep_between_capture)

        cmd = "echo '%s' > %s" % (cmd_list, tmpscript)
        self._exec("adb shell %s" % cmd, 10)

        self._exec("adb shell nohup sh %s" % tmpscript,
                   wait_for_response=False, timeout=1)
