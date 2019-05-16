"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA

:since: 2/19/15
:author: mmaraci
"""

import time

from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.BluetoothConnectivity import BluetoothConnectivity
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE

class BluetoothPanAirplane(LiveBTBase):
    """
    Bluetooth PAN Usecase class
    """
    _SLEEP_TIME_SECS = 1
    _DEFAULT_TIMEOUT_SECS = 5
    WAIT_TIME_IN_FLIGHT_MODE = 5.0

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveBTBase.__init__(self, tc_name, global_config)

        #Get TC parameters
        self._wifi_access_point = str(self._tc_parameters.get_param_value("WIFI_ACCESS_POINT", ""))
        self._connection_to_share = "WIFI"
        self.__global_config = global_config
        # Initialize data
        self._wait_btwn_cmd = 5.0
        self._bluetooth_connectivity_obj = BluetoothConnectivity(self._device, self._wait_btwn_cmd)


    def set_up(self):
        """
        Initialize the test
        """

        LiveBTBase.set_up(self)

        #locally initialize phone for PAN - turn tethering ON
        self._bt_api.set_bt_tethering_power("on")
        result, output = Global.SUCCESS, ""
        return result, output

    def run_test(self):
        """
        As of right now, at the beginning of the RUN part, we have connected the 2 phones in
        a PAN-NAP way and we are also connected to the internet.
        """
        LiveBTBase.run_test(self)

        # Handle External connection (WIFI or CELLULAR)
        self._bluetooth_connectivity_obj.handle_external_connection(self._connection_to_share, self._networking_api, self.__global_config, self._wifi_access_point)
        time.sleep(self._wait_btwn_cmd)

        #turn airplane mode ON
        self._networking_api.set_flight_mode(1)
        time.sleep(BluetoothPanAirplane.WAIT_TIME_IN_FLIGHT_MODE)

        #check BT, BT Tethering and Wifi are deactivated
        #check BT status is OFF
        time.sleep(self._wait_btwn_cmd)
        bt_power_air_on = self._bt_api.get_bt_power_status()
        if bt_power_air_on != str(BT_STATE.STATE_OFF):
            msg = "After setting Airplane mode to ON, Bluetooth is still ON. Expected state is OFF." \
                  "the actual state is: " + bt_power_air_on
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        #check BT tethering is turned off
        if self._bt_api.get_bt_tethering_power_status() != str(BT_STATE.STATE_OFF):
            msg = "After setting Airplane mode to ON, Bluetooth TETHERING is still ON. Expected state is OFF."
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        #check Wifi status is OFF
        if self._networking_api.get_wifi_power_status == 1:
            msg = "After setting Airplane mode to ON, WIFI is still ON. Expected state is OFF."
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        #turn airplane mode OFF
        self._networking_api.set_flight_mode(0)
        time.sleep(BluetoothPanAirplane.WAIT_TIME_IN_FLIGHT_MODE)

        #check BT, BT Tethering and Wifi are reactivated
        #check BT status is ON
        time.sleep(self._wait_btwn_cmd)
         #locally initialize phone for PAN - turn tethering ON
        self._bt_api.set_bt_tethering_power("on")

        bt_power_air_off = self._bt_api.get_bt_power_status()
        if bt_power_air_off != str(BT_STATE.STATE_ON):
            msg = "After setting Airplane mode to OFF, Bluetooth is still OFF. Expected state is ON." \
                  "the actual state is: " + bt_power_air_off
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        #check BT tethering is turned ON
        if self._bt_api.get_bt_tethering_power_status() != str(BT_STATE.STATE_ON):
            msg = "After setting Airplane mode to OFF, Bluetooth TETHERING is still OFF. Expected state is ON."
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        #check Wifi status is ON
        if self._networking_api.get_wifi_power_status == 1:
            msg = "After setting Airplane mode to OFF, WIFI is still OFF. Expected state is ON."
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "Usecase successful"


    def tear_down(self):

        # Disconnect and clear WiFi networks
        if self._connection_to_share == "WIFI":
            self._networking_api.wifi_remove_config("all")
            self._networking_api.set_wifi_power("off")

        LiveBTBase.tear_down(self)

        result, output = Global.SUCCESS, ""
        return result, output