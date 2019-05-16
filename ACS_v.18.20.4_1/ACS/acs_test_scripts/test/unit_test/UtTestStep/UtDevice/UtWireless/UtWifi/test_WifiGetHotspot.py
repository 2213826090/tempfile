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
:summary: unit test
:since 28/07/2014
:author: jfranchx
"""

import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiGetHotspot import WifiGetHotspot


class WifiGetHotspotTest(UTestTestStepBase):
    """
    WifiGetHotspot test cases
    """

    VERDICT_MSG_PARAMS = "VERDICT: on stored as HOTSPOT_CURRENT_PARAMS:POWER\nVERDICT: HOTSPOT_SSID stored as HOTSPOT_CURRENT_PARAMS:SSID\nVERDICT: WPA2 stored as HOTSPOT_CURRENT_PARAMS:SECURITY\nVERDICT: HOTSPOT_PASSPHRASE stored as HOTSPOT_CURRENT_PARAMS:PASSPHRASE\nVERDICT: 2_4GHZ_20MHZ stored as HOTSPOT_CURRENT_PARAMS:STANDARD\nVERDICT: 6 stored as HOTSPOT_CURRENT_PARAMS:CHANNEL\nVERDICT: off stored as HOTSPOT_CURRENT_PARAMS:HIDDEN"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()

    def _return_wifi_get_hotspot_status(self):
        """
        Stub method
        """
        return "on"

    def _return_wifi_get_hotspot_parameters(self):
        """
        Stub method
        """

        hotspot_parameters = {'SSID': "HOTSPOT_SSID", 'SECURITY': "WPA2",
                              'PASSPHRASE': "HOTSPOT_PASSPHRASE", 'STANDARD': "2_4GHZ_20MHZ",
                              'CHANNEL': "6", 'HIDDEN': "off"}
        return hotspot_parameters

    def test_run_parameters_ok(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"WIFI_HOTSPOT_PARAMETERS": "HOTSPOT_CURRENT_PARAMS"})

        self._context = TestStepContext()
        self._assert_run_succeeded_with_msg(sut, self.VERDICT_MSG_PARAMS)

        self.assertEqual("on", self._context.get_info("HOTSPOT_CURRENT_PARAMS:POWER"))
        self.assertEqual("HOTSPOT_SSID", self._context.get_info("HOTSPOT_CURRENT_PARAMS:SSID"))
        self.assertEqual("WPA2", self._context.get_info("HOTSPOT_CURRENT_PARAMS:SECURITY"))
        self.assertEqual("HOTSPOT_PASSPHRASE", self._context.get_info("HOTSPOT_CURRENT_PARAMS:PASSPHRASE"))
        self.assertEqual("2_4GHZ_20MHZ", self._context.get_info("HOTSPOT_CURRENT_PARAMS:STANDARD"))
        self.assertEqual("6", self._context.get_info("HOTSPOT_CURRENT_PARAMS:CHANNEL"))
        self.assertEqual("off", self._context.get_info("HOTSPOT_CURRENT_PARAMS:HIDDEN"))

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = WifiGetHotspot(None, None, test_step_pars, mock.Mock())
        sut._api.get_wifi_hotspot_status = self._return_wifi_get_hotspot_status
        sut._api.get_wifi_hotspot_parameters = self._return_wifi_get_hotspot_parameters
        return sut
