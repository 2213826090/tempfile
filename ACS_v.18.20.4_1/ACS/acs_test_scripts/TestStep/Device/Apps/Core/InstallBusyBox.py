"""
@summary: This file implements a Test Step to run the GLBenchmark application (either GLBenchmark 2.7 or GLBench3.0) for a specified duration.
Prerequisites for running GLbenchmark 2.7: the following script must be located in the path specified by SCRIPTS_PATH:
    run_glbenchmark_27.sh
This can be found in Artifactory at acs_test_artifacts/CONCURRENCY/TESTS/GLBenchmark.
In addition, the following APK should be installed on the DUT:
    GLBenchmark_2_7_0_Release_a68901_Android_Corporate_Package.apk or GFXBenchGL30.apk.
They can be found in the Artifactory at acs_test_artifacts/BENCHMARKS/GLBENCHMARK and acs_test_artifacts/CONCURRENCY/TESTS/GFXBENCH30
respectively.@since 26 June 2014
@since 1 June 2015
@author: Thierry CHOURRY
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
import os
import platform


class InstallBusyBox(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._system_api = self._device.get_uecmd("System")
        self._bootTimeout = self._device.get_config("bootTimeout", 500, float)

    def run(self, context):

        """
        Runs the test step
        """
        DeviceTestStepBase.run(self, context)
        self._logger.debug(self._pars.id +" : Test starting")

        timeout_60 = 60
        timeout_5 = 5

        _, custom_installed = self._device.run_cmd("adb shell ls -l /system/xbin/ | grep busybox | wc -l", timeout_5)
        if (custom_installed.isdigit() and int(custom_installed) > 0):
            self._logger.info("Busybox is already installed, nothing to do")
            return

        _, symlink = self._device.run_cmd("adb shell ls -l /system/bin | grep /data/bb | head -n1",
                                          timeout_5)

        try:
            _, msg = self._device.run_cmd("adb shell readlink -eq /system/bin/" + symlink.split()[-3]
                                          + "&& echo 1 || echo 0", timeout_5)
        except IndexError:
            msg = "0"

        if (msg.isdigit() and int(msg) > 0):
            self._logger.info("Busybox is already installed, nothing to do")
            return

        if not os.path.exists(self._pars.file):
            raise OSError(OSError.FileNotFoundError, "File %s not found" % self._pars.file)

        self._device.run_cmd("adb root", timeout_5)
        _, output = self._device.run_cmd("adb remount", timeout_60)

        if "dm_verity is enabled" in output:
            self._device.run_cmd("adb disable-verity", timeout_60)
            self._device.reboot()
            self._device.run_cmd("adb root", timeout_5)
            self._device.run_cmd("adb remount", timeout_60)

        self._device.run_cmd('adb shell mount -oremount,rw /', timeout_60)
        self._device.run_cmd('adb shell mkdir /data/bb/', timeout_5)
        self._device.run_cmd('adb push %s /data' % self._pars.file, timeout_60)
        self._device.run_cmd('adb shell chmod 777 /data/busybox', timeout_5)
        self._device.run_cmd('adb shell /data/busybox --install /data/bb', timeout_60)
        self._device.run_cmd('adb shell mount -oremount,rw /system', timeout_60)

        self._device.run_cmd('adb shell ls /data/bb | while read i; do type $i >/dev/null|| ln -s /data/bb/$i /system/bin/$i; done', timeout_60)

        self._logger.info("End of Busybox Step")





