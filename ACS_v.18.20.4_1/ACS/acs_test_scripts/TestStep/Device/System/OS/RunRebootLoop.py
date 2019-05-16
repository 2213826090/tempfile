"""
@summary: This file implements a Test Step to reboot the platform. and checks for
the system status.

@since 6 October 2014
@author: Santhosh Reddy Dubbaka
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
import time
import datetime as dt
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class RunRebootLoop(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.phonesystem_api=self._device.get_uecmd("PhoneSystem")

        #####################################
        ####  HELPER FUNCTIONS   ############
        #####################################

    # This function checks the the system UI is up and then issues a reboot.
    # If the usage of adb it so be avoided, then use reboot_only
    def systemui_check_and_reboot(self):
        if self.check_sysui():
            self._logger.debug(dt.datetime.now().strftime("%Y %b %d - %I:%M:%S%p") +"RunRebootLoop Test: Rebooting the system")
            time.sleep(1)
            self._device.reboot()
        else:
            self._logger.debug(dt.datetime.now().strftime("%Y %b %d - %I:%M:%S%p") + " RunRebootLoop Test:Sys UI is not present, system didn't boot properly")
            self._logger.debug(dt.datetime.now().strftime("%Y %b %d - %I:%M:%S%p") + " RunRebootLoop Test Exiting from the script.")
            self._logger.error("RunRebootLoop Test : Sys UI is not present, system didn't boot properly")
            raise DeviceException(DeviceException.OPERATION_FAILED, "RunRebootLoop Test:Sys UI is not present, system didn't boot properly")

    def check_sysui(self):
        # Failure flag
        sysui_ok = False

        # Check that the system UI is alive
        sysui_ok=self.phonesystem_api.check_process("systemui")
        return sysui_ok

        ##############################################
        #### "MAIN" EXECUTION STARTS HERE ############
        ##############################################
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.debug("RunRebootLoop Test: RUN ")

        # Print test start
        self._logger.debug(dt.datetime.now().strftime("%Y %b %d - %I:%M:%S%p") +"RunRebootLoop Test started on: " + str(dt.datetime.now()))
        reboot_cycle_count = 1

        end_time = float(time.time()) + (float(self._pars.duration)) * 60
        while (time.time() < end_time):
            self._logger.debug(dt.datetime.now().strftime("%Y %b %d - %I:%M:%S%p") + " RunRebootLoop Test: ******* STARTING CYCLE: " + str(reboot_cycle_count) + " *******")

            # adb device check flag
            adb_dev_ok = False
            # Check for adb connection
            # Reboot the system
            adb_dev_ok = self.phonesystem_api.read_phone_device("MOS")
            if adb_dev_ok:
                self.systemui_check_and_reboot()
                reboot_cycle_count += 1

        self._logger.debug("RunRebootLoop Test Completed.")
