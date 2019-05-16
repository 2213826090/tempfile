"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the step to get the expected safe WIFI/BLE/WiFi channels and frequencies ranges for the current DUT.
:since: 2014-12-11
:author: emarchan

"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from Utilities.CommunicationUtilities import TelephonyConfigsParser
from ErrorHandling.AcsConfigException import AcsConfigException


class GetExpectedSafeRangesForLte(DeviceTestStepBase):
    """
    Gets the expected safe WIFI/BLE/WiFi channels and frequencies ranges for the current DUT.
    """


    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        dest_variable = self._pars.expected_safe_ranges

        try:
            result = TelephonyConfigsParser("LTE_Expected_Safe_Ranges").\
                parse_lte_expected_safe_ranges(self._device.get_phone_model(), self._pars.lte_channel)
        except IndexError:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "LTE_Expected_Safe_Ranges does not have targets for %s model and %s channel" % (self._device.get_phone_model(), self._pars.lte_channel))

        if result is None:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "Could not obtain LTE Safe ranges.")

        context.set_nested_info([dest_variable, "WIFI_MIN_CHANNEL"], result.wifi_min_channel)
        context.set_nested_info([dest_variable, "WIFI_MAX_CHANNEL"], result.wifi_max_channel)
        context.set_nested_info([dest_variable, "WIFI_MIN_FREQ"], result.wifi_min_freq)
        context.set_nested_info([dest_variable, "WIFI_MAX_FREQ"], result.wifi_max_freq)
        context.set_nested_info([dest_variable, "BLE_MIN_CHANNEL"], result.ble_min_channel)
        context.set_nested_info([dest_variable, "BLE_MAX_CHANNEL"], result.ble_max_channel)
        context.set_nested_info([dest_variable, "BLE_MIN_FREQ"], result.ble_min_freq)
        context.set_nested_info([dest_variable, "BLE_MAX_FREQ"], result.ble_max_freq)
        context.set_nested_info([dest_variable, "BT_MIN_CHANNEL"], result.bt_min_channel)
        context.set_nested_info([dest_variable, "BT_MAX_CHANNEL"], result.bt_max_channel)
        context.set_nested_info([dest_variable, "BT_MIN_FREQ"], result.bt_min_freq)
        context.set_nested_info([dest_variable, "BT_MAX_FREQ"], result.bt_max_freq)



        self.ts_verdict_msg = "VERDICT: %s stored as {0}\n".format(result.wifi_min_channel) % "WIFI_MIN_CHANNEL"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.wifi_max_channel) % "WIFI_MAX_CHANNEL"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.wifi_min_freq) % "WIFI_MIN_FREQ"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.wifi_max_freq) % "WIFI_MAX_FREQ"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.ble_min_channel) % "BLE_MIN_CHANNEL"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.ble_max_channel) % "BLE_MAX_CHANNEL"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.ble_min_freq) % "BLE_MIN_FREQ"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.ble_max_freq) % "BLE_MAX_FREQ"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.bt_min_channel) % "BT_MIN_CHANNEL"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.bt_max_channel) % "BT_MAX_CHANNEL"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.bt_min_freq) % "BT_MIN_FREQ"
        self.ts_verdict_msg += "%s stored as {0}\n".format(result.bt_max_freq) % "BT_MAX_FREQ"
        self._logger.debug(self.ts_verdict_msg)


