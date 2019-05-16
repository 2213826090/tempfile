"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@summary: This file implements a Test Step to disconnect USB tethering
@since 25 Aug 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""
import time
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager

class DisconnectUsbTethering(DeviceTestStepBase):
    """
    Implements a Test Step to disconnect USB tethering
    """

    def run(self, context):
        """
        Connect USB tethering

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Start the USB tether to get a virtual ethernet connection from phone
        self._hostpc = EquipmentManager().get_computer(eqt_name="COMPUTER1")
        self._timeout = self._pars.time_out_sec

        netAPI = self._device.get_uecmd("Networking")

        usb_tethering_start_time = time.time()
        usb_tethering_unavailable = False
        while (time.time() - usb_tethering_start_time < self._timeout) and usb_tethering_unavailable == False:
            time.sleep(1)

            try:
                netAPI.stop_usb_tethering(unplug=True)
                usb_tethering_unavailable = True
            except Exception, e:
                usb_tethering_unavailable = False
                msg = "Failed to stop the USB tether on the device: {0}. Retry to stop USB tether after 1 second".format(e)
                self._logger.info(msg)

        if usb_tethering_unavailable == False:
            msg = "Failed to stop the USB tether on the device. Aborting TestStep execution"
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
