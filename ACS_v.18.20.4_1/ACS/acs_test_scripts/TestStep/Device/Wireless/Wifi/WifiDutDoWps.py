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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to set key exchange mode
:since 28/11/2014
:author: Gangx, Lu
"""

from TestStep.Device.Wireless.Wifi.WifiBase import WifiBase
from acs_test_scripts.Equipment.ConfigurableAP.Common.Common import WifiKeyExchangeTypes
from ErrorHandling.DeviceException import DeviceException


class WifiDutDoWps(WifiBase):
    """
    Sets key exchange mode (WPS for instance) for secured WIFI network
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        WifiBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.wps_ap_pin = None

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiBase.run(self, context)

        if self._pars.wps_method in [WifiKeyExchangeTypes.WPS_PIN_FROM_AP, WifiKeyExchangeTypes.WPS_PIN_FROM_DUT]:
            if str(self._pars.wps_ap_pin).isdigit():
                self.wps_ap_pin = str(self._pars.wps_ap_pin)
            else:
                msg = "Can't set key exchange mode, wps pin from ap is a invalid number"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # WiFi must be On to use this function
        if self._api.get_wifi_power_status() == 0:
            msg = "Can't set key exchange mode, WiFi is currently OFF"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._api.wifi_setkeyexchange(self._pars.ssid,
                                      self._pars.wps_method,
                                      self.wps_ap_pin)
