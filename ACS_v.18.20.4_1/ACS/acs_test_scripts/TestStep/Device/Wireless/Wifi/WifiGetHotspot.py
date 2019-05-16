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
:summary: This file implements a Test Step to get Hotspot status
:since 18/07/2014
:author: jfranchx
"""
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase

from UtilitiesFWK.Utilities import str_to_bool_ex


class WifiGetHotspot(WifiBase):
    """
    Implements the get Hotspot test step for Wifi
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiBase.run(self, context)
        self._logger.info("Getting the Wifi Hotspot parameters state...")

        result_power = self._api.get_wifi_hotspot_status()
        if str_to_bool_ex(result_power):
            result_power = "on"
        else:
            result_power = "off"

        result_parameters = self._api.get_wifi_hotspot_parameters()

        context.set_nested_info([self._pars.wifi_hotspot_parameters, "POWER"], result_power)
        context.set_nested_info([self._pars.wifi_hotspot_parameters, "SSID"], result_parameters["SSID"])
        context.set_nested_info([self._pars.wifi_hotspot_parameters, "SECURITY"], result_parameters["SECURITY"])
        context.set_nested_info([self._pars.wifi_hotspot_parameters, "PASSPHRASE"], result_parameters["PASSPHRASE"])
        context.set_nested_info([self._pars.wifi_hotspot_parameters, "STANDARD"], result_parameters["STANDARD"])
        context.set_nested_info([self._pars.wifi_hotspot_parameters, "CHANNEL"], result_parameters["CHANNEL"])
        context.set_nested_info([self._pars.wifi_hotspot_parameters, "HIDDEN"], result_parameters["HIDDEN"])

        self._ts_verdict_msg = "VERDICT: %s stored as {0}:POWER".format(
            str(self._pars.wifi_hotspot_parameters)) % result_power
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:SSID".format(
            str(self._pars.wifi_hotspot_parameters)) % result_parameters["SSID"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:SECURITY".format(
            str(self._pars.wifi_hotspot_parameters)) % result_parameters["SECURITY"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:PASSPHRASE".format(
            str(self._pars.wifi_hotspot_parameters)) % result_parameters["PASSPHRASE"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:STANDARD".format(
            str(self._pars.wifi_hotspot_parameters)) % result_parameters["STANDARD"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:CHANNEL".format(
            str(self._pars.wifi_hotspot_parameters)) % result_parameters["CHANNEL"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:HIDDEN".format(
            str(self._pars.wifi_hotspot_parameters)) % result_parameters["HIDDEN"]
        self._logger.debug(self._ts_verdict_msg)
