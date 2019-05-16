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
:summary: This file implements the LIVE_DUAL_PHONE_BT_CHECK_DISCOVERABLE_MODE
:author: npan2
:since:20/10/2011
"""

import time
from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTCheckDiscoverableMode(LiveDualPhoneBTBase):

    """
    Live BT Check Discoverable Mode test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Read DIS_MODE from test case xml file
        self._dis_mode = \
            str(self._tc_parameters.get_param_value("DIS_MODE")).lower()

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # Set screen always on and unlock phone (Since LLP , DUT need to be unlocked to be visible)
        self._phone_system_api.set_phone_screen_lock_on("on")
        self._phone_system_api.set_phone_lock(0)

        if self._dis_mode in ("on", "1"):
            self._bt_api.\
                set_bt_discoverable('both', 0)
        elif self._dis_mode in ("off", "0"):
            self._bt_api.\
                set_bt_discoverable('none', 0)
        else:
            msg = "Set wrong dis_mode. could only be on(1) or off(0)"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Remove pairing from DEVICE_BT_ADDRESS
        time.sleep(self._wait_btwn_cmd)
        if (self._dis_mode in ("on", "1")
                and not self._bt_api2.bt_find_device(self._phone1_addr)) or \
            (self._dis_mode in ("off", "0")
             and self._bt_api2.bt_find_device(self._phone1_addr)):
            msg = "phone in wrong discoverable mode"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
