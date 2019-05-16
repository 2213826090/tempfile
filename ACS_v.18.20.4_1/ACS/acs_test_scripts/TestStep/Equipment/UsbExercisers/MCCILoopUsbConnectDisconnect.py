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

:organization: INTEL PEG-SVE-DSV
:summary: This test step will use the MCCI USB Connection Exerciser to connect/disconnect a USB connection.
             PREREQUISITES:
             MCCI USB Connection Exerciser (Model 2101) devices must be properly setup and connected to the host PC.
             mcci2101.exe should be installed at the location given from "SCRIPTS_PATH" onto host PC from Artifactory under:  acs_test_artifacts:CONCURRENCY/TESTS/usb_unplug_plug
:since: 05/09/2014
:author: sasmith2
"""
import time
import os
import sys
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsBaseException import AcsBaseException
import acs_test_scripts.Utilities.OSBVUtilities as osbv_utils


class MCCILoopUsbConnectDisconnect(EquipmentTestStepBase):
    """
    Disconnect and connect USB connection for test duration.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.local_computer = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        self._logger.info(self._pars.id + ": Test step starting.")
        EquipmentTestStepBase.run(self, context)
        # Timeout value to use that determines amount of time to wait on gui focus lock file to be removed.
        # We don't need the GUI, but we are using this mechanism to coordinate with test steps that might generate USB traffic.
        # We do not want them to try using USB while it is disconnected.
        self.usb_lock_wait_time = self._pars.usb_lock_wait_time
        self.appSignature = 'mcci_loop_usb_connect_disconnect'
        self.mcciappPath = self._pars.scripts_path + " -dev 1"

        self.local_computer = self._equipment_manager.get_computer(self._pars.eqt)

        try:
            # Delete any focus-lock file that may not have been released during the previous test run.
            osbv_utils.cleanup_focus_lock(self.appSignature)
        except:
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, self._pars.id + ": Issue trying to remove previous focus lock file.")

        # Let's try to set the usb speed to be used during test and stop test if hardware is improperly set up or bad self._pars.usb_speed option supplied.
        if self._pars.usb_speed == 1:
            ret = self.USB_HS_On()
            if ret == -1:
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, self._pars.id + ": Issue using MCCI USB Connection Exerciser.  Try checking whether the hardware is correctly set up.")
        elif self._pars.usb_speed == 2:
            ret = self.USB_SS_On()
            if ret == -1:
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, self._pars.id + ": Issue using MCCI USB Connection Exerciser.  Try checking whether the hardware is correctly set up.")

        try:
            self.plug_unplug()
        except:
            import traceback
            self._logger.error(self._pars.id + ":  Unexpected exception -> " + str(sys.exc_info()[0]))
            self._logger.error(traceback.format_exc())
            self.ts_verdict_msg = self._pars.id + ":  Unexpected exception being raised"
            raise
        finally:
            # Check if focus lock still exists with self.appSignature and remove if so.
            if self.appSignature == osbv_utils.get_lock_signature():
                if not osbv_utils.release_focus_lock(self.appSignature):
                    # Unable to release the focus-lock
                    raise AcsBaseException(AcsBaseException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
            # Try to leave USB connection on after test.
            self.USB_On(self._pars.usb_speed)
        self._logger.info(self._pars.id + ": Test step finished.")


    def USB_SS_On(self):
        ret=os.system(self.mcciappPath + ' -ssattach')
        return ret

    def USB_HS_On(self):
        ret=os.system(self.mcciappPath + ' -hsattach')
        return ret


    def USB_Off(self):
        ret=os.system(self.mcciappPath + ' -detach')
        return ret

    def USB_On(self, usb_speed = 1):
        if self._pars.usb_speed == 1:
            # High Speed
            self.USB_HS_On()
        elif self._pars.usb_speed == 2:
            # Super Speed
            self.USB_SS_On()

    def check_device_list(self):
        count = 0
        result = self.local_computer.run_cmd("adb devices")
        for item in result['std'].split('\r'):
            if '\tdevice' in item:
                count += 1
        return count

    def plug_unplug (self):
        start_time = time.time()
        self._logger.info(self._pars.id + ": ============================================================================")
        self._logger.info(self._pars.id + ":                          MCCI USB Plug/Unplug test start")
        self._logger.info(self._pars.id + ": ============================================================================")
        iteration_count = 0
        pass_count = 0
        verdict = True
        self._logger.debug(self._pars.id + ":  test duration (minutes) - " + str(self._pars.duration))
        #We need to end thread earlier than self._pars.duration by at least the time it will take for one pass of this while loop.
        #That way, (start of last unplug/plug loop + length of time for one loop) should be < self._pars.duration and we can finish loop gracefully.
        #This should be ok, as we generally run our tests for 2 hours or more and worse case, we won't unplug/plug USB the last ~5 minutes.
        timeTest = (float(self._pars.duration)*60) - (float(self._pars.time_unplug_duration)*60 + float(self._pars.time_between_unplug)*60 + 60);	#give it an extra 60 seconds for reconnecting and any other necessary calls that are happening.
        while time.time()-start_time < timeTest:
            iteration_count += 1
            self._logger.info("{0}: [Iteration {1}] ".format(self._pars.id, iteration_count))
            num_dev = self.check_device_list()
            if not osbv_utils.set_focus_lock(self.appSignature, timeout_sec=self.usb_lock_wait_time):
                # Could not set the focus-lock
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED, self._pars.id + ": Failed to set focus lock.")
            self.USB_Off()
            self._logger.info("{0}: [Iteration {1}] USB cable is unplugged".format(self._pars.id, iteration_count))
            time.sleep(float(self._pars.time_unplug_duration)*60)
            if self.check_device_list() != (num_dev - 1):
                self._logger.error("{0}: [Iteration {1}] No device seems to have detached after disconnecting USB.  The test step has failed.".format(self._pars.id, iteration_count))
                verdict = False
            self.USB_On(self._pars.usb_speed)
            if not osbv_utils.release_focus_lock(self.appSignature):
                # Could not release the focus-lock
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED, self._pars.id + ": Failed to release focus lock.")
            self._logger.info("{0}: [Iteration {1}] USB cable is plugged in".format(self._pars.id, iteration_count))
            # Give a little time before checking USB connected state.
            time.sleep(6)
            if self.check_device_list() != num_dev:
                self._logger.error("{0}: [Iteration {1}] No device seems to have attached after connecting USB.  The test step has failed.".format(self._pars.id, iteration_count))
                verdict = False
            if not verdict:
                self.ts_verdict_msg = self._pars.id + ":  Failed to connect/disconnect USB!"
                raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, self._pars.id + ": Failed to execute correctly.")
            else:
                pass_count +=1
            time.sleep(float(self._pars.time_between_unplug)*60)
        self._logger.info("{0}: [Reference] Test finished at {1}".format(self._pars.id, time.strftime("%m\\%d\\%Y %H:%M:%S", time.localtime())))
        self._logger.info("{0}: [Reference] Total PASS count : {1}".format(self._pars.id, pass_count) )
        self._logger.info(self._pars.id + ": ============================================================================")
        self._logger.info(self._pars.id + ": ============================================================================")
        self._logger.info(self._pars.id + ":                          MCCI USB Plug/Unplug test end")
        self._logger.info(self._pars.id + ": ============================================================================")
