"""
@summary: Make system to stay in S0i3 state for the specified amount of time.
This test step depends on a relay switch which is used to connect/disconnect power.
@since 20 April 2015
@author: Jongyoon Choi
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
import re
import time

class ResidentSState(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")

        # Get IO card instance. Default value is 'IO_CARD'
        self._equipment_manager = self._factory.create_equipment_manager()
        self._io_card = self._equipment_manager.get_io_card("IO_CARD")

        '''
        Getting all needed ACS APIs
        '''
        self._residency_api = self._device.get_uecmd("Residencies")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._network_api = self._device.get_uecmd("Networking")

        '''
        Setting up DUT to sleep after 15 secs
        '''
        self._phone_system_api.set_stay_on_while_plugged_in(False)
        self._phone_system_api.set_screen_timeout(15)
        self._network_api.set_flight_mode(1)

        if self._pars.feature == "S0I3":
            self._residency_api.disable_s3()

        '''
        clearing the states info
        '''
        self._residency_api.clear()
        self._logger.debug("Unplug USB")
        self._io_card.usb_host_pc_connector(False)
        self._device.disconnect_board()
        self._logger.debug("SLEEPING FOR %s MINS" % self._pars.duration)
        timeout = (float(self._pars.duration) * 60.0)

        while timeout > 0:

            t0 = time.time()
            time.sleep(30)
            t1 = time.time()
            timeout -= (t1 - t0)
            self._logger.info("SLEEPING")

        self._logger.debug("Plug in USB")
        self._io_card.usb_host_pc_connector(True)
        self._device.connect_board()

        self._logger.info(self._pars.id + ": Done")
