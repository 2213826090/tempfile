"""
@summary: This file implements a Test Step to run CV_JB_AndAR.apk on the device.  This
    application can be found in Artifactory under acs_test_artifacts/CONCURRENCY/TESTS/AugmentedReality.
    It plays a video(L_marker.wmv) on the host machine, opens the camera app and adjusts the focus based on the L-marker movement in the video.
@since 5 October 2014
@author: Santhosh Reddy D
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import acs_test_scripts.Utilities.OSBVUtilities as osbv
import time

class RunAugmentReality(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.system_api = self._device.get_uecmd("System")
        self.app_api = self._device.get_uecmd("AppMgmt")
        self.phonesystem_api=self._device.get_uecmd("PhoneSystem")
        self.display_api=self._device.get_uecmd("Display")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.debug("AugmentReality: Run")

        # Launch the video on the host
        self.process = osbv.open_file(self._pars.video_file_path)

        # Launch the augmented reality app using the adb am command
        PackageActivity = self._pars.aug_package+"/"+self._pars.aug_activity
        time.sleep(5.5)

        self.app_api.launch_app(intent=PackageActivity, extras="--es name sofa.obj --ei type 0")
        time.sleep(self._pars.duration*60)

        # Check that an augmented reality app failure wasn't detected
        no_failures = self.phonesystem_api.check_process(self._pars.aug_package)
        if no_failures:
            self._logger.debug("Augmented Reality App OK!")
        else:
            self._logger.error("Augmented Reality App Crashed!")
            raise DeviceException(DeviceException.OPERATION_FAILED, "Augmented Reality App Crashed!")

        # Check for system UI
        systemui_ok =self.phonesystem_api.check_process("systemui")
        if systemui_ok:
            self._logger.debug("AugmentReality: Systemui is active")
        else:
            self._logger.error("AugmentReality: Systemui is not active.")
            raise DeviceException(DeviceException.OPERATION_FAILED, "AugmentReality: Systemui is not active.")


        # Close out the application
        self.system_api.stop_app(self._pars.aug_package)
        # Close out the video that is playing
        osbv.stop_file(self.process)

        self._logger.debug("AugmentReality: Done")
