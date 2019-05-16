"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: Determine what, if any, BlueTooth audio device is paired with the system.
:since: 28 July 2014
:author: Jongyoon Choi
"""
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class CheckBTAudioDevice(DeviceTestStepBase):
    SINGLE_RUN_STOP_ON_FAIL = 1
    SINGLE_RUN_CONTINUE_ON_FAIL = 2
    MULTI_RUN_STOP_ON_FAIL = 3

    """
    Set screen properties.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._api = self._device.get_uecmd("PhoneSystem")
        self.bt_api = self._device.get_uecmd("LocalConnectivity")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info("CheckBTAudioDevice: Run")

        if self._pars.check_mode != CheckBTAudioDevice.MULTI_RUN_STOP_ON_FAIL:
            (is_bt_connected, bt_device_name) = self.bt_api.is_bt_audio_connected()

            if not is_bt_connected and self._pars.check_mode == CheckBTAudioDevice.SINGLE_RUN_STOP_ON_FAIL:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Failed to find a BT audio device")

            context.set_info(self._pars.bt_audio_device, bt_device_name)

        else:
            # Get Parameters
            test_duration = self._pars.duration * 60
            start_time = time.time()

            #Start checking loop
            while time.time()-start_time < test_duration:
                (is_bt_connected, bt_device_name) = self.bt_api.is_bt_audio_connected()

                if is_bt_connected:
                    self._logger.debug("BT audio device {0} is connected".format(bt_device_name))
                else:
                    raise DeviceException(DeviceException.OPERATION_FAILED, "BT audio device disconnected")

                time.sleep(self._pars.check_interval)

            context.set_info(self._pars.bt_audio_device, bt_device_name)

        self._logger.info("CheckBTAudioDevice: Done")
