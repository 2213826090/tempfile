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
:since 17/09/2014
:author: jfranchx
"""
import mock

from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtWireless.UtCellular.UtCellularBase import UtCellularBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Device.Wireless.Cellular.GetPreferredNetwork import GetPreferredNetwork


class GetPreferredNetworkTest(UtCellularBase):
    """
    GetPreferredNetwork test cases
    """

    CONTEXT_SAVE_GET = "DUT_PREFERRED_NETWORK"
    INVALID_PREFERRED_NETWORK = "INVALID_PREFERRED_NETWORK"
    MSG_INVALID_VALUE = "get_preferred_network_type returned an unknown value : INVALID_PREFERRED_NETWORK"

    def setUp(self):
        """
        Set up
        """
        UtCellularBase.setUp(self)
        self._context = TestStepContext()
        self._preferred_network = None
        self._flight_mode = 0

    def test_get_preferred_network_2g_only_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_2G_ONLY
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_2G_ONLY) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_2G_ONLY, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_3g_only_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_3G_ONLY
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_3G_ONLY) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_3G_ONLY, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_4g_only_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_4G_ONLY
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_4G_ONLY) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_4G_ONLY, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_3g_pref_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_3G_PREF
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_3G_PREF) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_3G_PREF, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_4g_pref_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_4G_PREF
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_4G_PREF) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_4G_PREF, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_2g_3g_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_2G_3G
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_2G_3G) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_2G_3G, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_cdma_pref_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_CDMA_PREF
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_CDMA_PREF) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_CDMA_PREF, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_cdma_only_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_CDMA_ONLY
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_CDMA_ONLY) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_CDMA_ONLY, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_evdo_only_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_EVDO_ONLY
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_EVDO_ONLY) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_EVDO_ONLY, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_global_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_GLOBAL
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_GLOBAL) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_GLOBAL, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_4g_pref_us_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_4G_PREF_US
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_4G_PREF_US) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_4G_PREF_US, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_world_mode_ok(self):
        self._preferred_network = self.PREFERRED_NETWORK_WORLD_MODE
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_succeeded_with_msg(sut, "VERDICT: %s stored as {0}".format(self.PREFERRED_NETWORK_WORLD_MODE) % self.CONTEXT_SAVE_GET)
        self.assertEqual(self.PREFERRED_NETWORK_WORLD_MODE, self._context.get_info(self.CONTEXT_SAVE_GET))

    def test_get_preferred_network_flight_mode_on_fail(self):
        self._preferred_network = self.PREFERRED_NETWORK_2G_ONLY
        self._flight_mode = self.FLIGHT_MODE_ON
        sut = self._create_sut({"SAVE_AS": self.CONTEXT_SAVE_GET})
        self._assert_run_throw_device_exception(sut, "Can't set preferred network, Flight mode is currently ON")

    def _return_get_preferred_network_type(self):
        """
        Stub method
        """
        return self._preferred_network

    def _return_flight_mode(self):
        """
        Stub method
        """
        return self._flight_mode

    # pylint: disable=W0212
    def _create_sut(self, args=None):
        """
        Create the SUT with only test step pars
        """
        sut = GetPreferredNetwork(None, None, args, mock.Mock())
        sut._networking_api.get_preferred_network_type = self._return_get_preferred_network_type
        sut._networking_api.get_flight_mode = self._return_flight_mode
        return sut
