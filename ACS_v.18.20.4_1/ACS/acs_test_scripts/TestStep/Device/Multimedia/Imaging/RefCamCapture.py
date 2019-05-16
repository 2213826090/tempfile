"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This file implements the Test Steps to deal with User mode image capture
:since:08/06/2015
:author: tchourrx
"""
import time

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class RefCamCapture(DeviceTestStepBase):
    """
    Test steps to control intel refcam camera
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._camera_handle = None

    def _run(self):
        """
        Specific code for Intel RefCam test step
        """

    def run(self, context):
        """
        Run test step
        """
        DeviceTestStepBase.run(self, context)

        self._camera_handle = self._context.get_info("IntelRefCam_Instance")
        if self._camera_handle is None:
            self._camera_handle = self._device.get_uecmd("IntelRefCam")
            self._context.set_info("IntelRefCam_Instance", self._camera_handle)

        self._run()


class RefCamLaunch(RefCamCapture):
    """
    Launch camera test step
    """
    def _run(self):
        """
        Run the specific code of the test step
        """
        camera_pkg_path = self._pars.config_file.split("/shared_prefs")[0]
        filename = self._pars.config_file.split("/shared_prefs")[1]
        shared_pref_path = self._pars.config_file.split(filename)[0]
        usable_config_file = shared_pref_path + "/test_pref.xml"

        # Set camera configuration name to test_pref.xml
        #  to match with bkm and be loaded by camera application
        self._device.run_cmd("adb shell mv %s %s" % (self._pars.config_file, usable_config_file), 5)

        # Set camera configuration file
        # rights to allow application to load it
        _, check_rights = self._device.run_cmd("adb shell stat -c '%%U' %s" % camera_pkg_path, 5)
        app_username = check_rights.strip()
        ret, msg = self._device.run_cmd("adb shell chown -R %s %s" % (app_username, shared_pref_path), 5)
        if ret != 0:
            self._logger.error("Failed to change rights on camera configuration file %s" % usable_config_file)
        else:
            self._logger.debug("User %s set for camera configuration file %s" % (app_username, usable_config_file))

        # Launch Intel camera
        self._camera_handle.launch()


class RefCamStop(RefCamCapture):
    """
    Stop camera test step
    """
    def _run(self):
        """
        Run the specific code of the test step
        """
        self._camera_handle.stop()


class RefCamPushCaptureButton(RefCamCapture):
    """
    Start/stop capture (Video mode) or capture image (Single mode)
    """
    def _run(self):
        """
        Run the specific code of the test step
        """
        self._camera_handle.capture()


class RefCamUserModeImageCapture(RefCamCapture):
    """
    Push the user mode image capture script
    """
    def _run(self):
        """
        Run test step
        """
        nb_capture = (int)(float(self._pars.test_duration) /
                           float(self._pars.sleep_between_capture))

        camera_api = self._context.get_info("IntelRefCam_Instance")
        camera_api.user_mode_image_capture(self._pars.sleep_duration + self._pars.delay,
                                           self._pars.sleep_between_capture,
                                           nb_capture)

        self._context.set_info(self._pars.save_start_date_as, time.time() +
                               self._pars.sleep_duration + self._pars.delay/2)
