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
:summary: This file implements the LIVE_BT_AND_WIFI_SWITCH_POWER
:author: jfranchx
:since:08/08/2013
"""

from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
import Queue
import time


class LiveBTAndWiFiSwitchPower(LiveBTBase):
    """
    Live BT and WiFi switch power iteratively
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        self._duration = str(self._tc_parameters.get_param_value("DURATION"))

        self._thread_bt_switch = None
        self._thread_wifi_switch = None
        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        if not str(self._duration).isdigit():
            msg = "Bad value on duration parameter : %s" % self._duration
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._duration = int(self._duration)

        self._networking_api.set_wifi_power(0)
        self._bt_api.set_bt_power(0)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        start = time.time()
        while (start + self._duration) > time.time():
            self._bt_api.set_bt_power(1)
            time.sleep(self._wait_btwn_cmd)
            self._check_bt_wifi_state(str(BT_STATE.STATE_ON), 0)

            self._networking_api.set_wifi_power(1)
            time.sleep(self._wait_btwn_cmd)
            self._check_bt_wifi_state(str(BT_STATE.STATE_ON), 1)

            self._bt_api.set_bt_power(0)
            time.sleep(self._wait_btwn_cmd)
            self._check_bt_wifi_state(str(BT_STATE.STATE_OFF), 1)

            self._networking_api.set_wifi_power(0)
            time.sleep(self._wait_btwn_cmd)
            self._check_bt_wifi_state(str(BT_STATE.STATE_OFF), 0)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """

        # Call UseCase base tear_down function
        LiveBTBase.tear_down(self)

        # Disable WiFi
        self._networking_api.set_wifi_power(0)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _check_bt_wifi_state(self, bt_state, wifi_state):
        """
        Check if BT and WiFi power are as expected.

        :type bt_state : str
        :param bt_state : BT state expected
        :type wifi_state : str or integer
        :param wifi_state : WiFi state expected - can be ('on', '1', 1) to enable or ('off', '0', 0) to disable
        """

        if self._bt_api.get_bt_power_status() != bt_state:
            msg = "Bluetooth status must be %s but is %s" % (bt_state, self._bt_api.get_bt_power_status())
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if wifi_state not in [0, 1]:
            if wifi_state in ['on', '1']:
                wifi_state = 1
            elif wifi_state in ['off', '0']:
                wifi_state = 0

        if self._networking_api.get_wifi_power_status() != wifi_state:
            msg = "WiFi status must be %s but is %s" % (wifi_state, self._networking_api.get_wifi_power_status())
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
