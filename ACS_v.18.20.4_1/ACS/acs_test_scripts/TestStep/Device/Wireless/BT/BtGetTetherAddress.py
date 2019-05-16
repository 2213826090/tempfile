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

:organization: INTEL PEG-SVE-DSV
:summary: This file implements a Test Step to get the PAN address for a Bluetooth tether connection
:since 28/04/2014
:author: Val Peterson
"""

from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from ErrorHandling.DeviceException import DeviceException
import time

class BtGetTetherAddress(BtBase):

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        BtBase.run(self, context)

        self._logger.info("Getting the Wifi interface address...")
        networking_api = self._device.get_uecmd("Networking")
        polling_start = time.time()
        current_time = polling_start
        no_addr = True
        while (current_time - polling_start < 75) and no_addr:
            try:
                no_addr = False
                pan_address = networking_api.get_interface_ipv4_address("bt-pan")
            except DeviceException as e:
                self._logger.info("BtGetTetherAddress - error on this attempt to obtain address:" + str(e))
                no_addr = True
                time.sleep(5)
                current_time = time.time()
        if no_addr:
            raise(DeviceException(DeviceException.OPERATION_FAILED, "BtGetTetherAddress: Could not get PAN address."))

        self._logger.info("Setting context variable %s to PAN address %s"%(self._pars.save_as, pan_address))
        context.set_info(self._pars.save_as, pan_address)
