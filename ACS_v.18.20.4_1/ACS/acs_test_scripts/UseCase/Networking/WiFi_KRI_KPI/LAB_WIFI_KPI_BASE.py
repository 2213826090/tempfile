"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements the LIVE WIFI KPI Base UC
@since: 16/06/2014
@author: jfranchx
"""

import time
from acs_test_scripts.Equipment.ConfigurableAP.Cisco1250.Cisco1250 import Cisco1250
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies
from acs_test_scripts.UseCase.Networking.WiFi_KRI_KPI.LIVE_WIFI_KPI_BASE import LiveWifiKPIBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LabWifiKPIBase(LiveWifiKPIBase):
    """
    Lab WiFi KPI Base Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Generic initialization for KPI TCs
        """
        LiveWifiKPIBase.__init__(self, tc_name, global_config)

    def set_up(self):
        """
        Generic set_up for Lab KPI TCs. Should be executed after LabWifiBase.
        """
        LiveWifiKPIBase.set_up(self)

        # Configure RSSI
        self._configure_rssi()

        return Global.SUCCESS, "No errors"

    # ------------------------------------------------------------------------------------------------------------------
    def _configure_rssi(self):
        """
        Configure the RSSI to be in the good range
        """
        rssi_check = False
        if self._standard in AcsWifiFrequencies.WIFI_STANDARD_5G:
            power_values = Cisco1250.POWER_VALUES_5G
        else:
            power_values = Cisco1250.POWER_VALUES_2G

        # Initiate connection to the equipment
        self._ns.init()

        try:
            # Configure power on AP
            for power in power_values:
                self._ns.set_wifi_power(self._standard, power)
                time.sleep(3.0)
                # Check WiFi RSSI - require between -40dBm and -45dBm
                self._rssi_value = self._networking_api.get_wifi_rssi(self._ssid)
                if self._rssi_value <= self.RSSI_MAX_VALUE and self._rssi_value >= self.RSSI_MIN_VALUE:
                    self._logger.debug("Check RSSI : %s - Power used : %s" % (self._rssi_value, power))
                    rssi_check = True
                    break
        finally:
            # Close the connection to AP
            self._ns.release()

        if not rssi_check:
            # Bad RSSI configuration, raise an exception
            msg = "Bad RSSI to run the test : %sdBm - require between %sdBm and %sdBm" % (
            self._rssi_value, self.RSSI_MIN_VALUE, self.RSSI_MAX_VALUE)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
