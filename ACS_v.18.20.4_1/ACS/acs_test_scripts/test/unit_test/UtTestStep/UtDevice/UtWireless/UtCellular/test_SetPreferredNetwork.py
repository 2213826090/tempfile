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
:since 07/08/2014
:author: jfranchx
"""
import mock

from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtWireless.UtCellular.UtCellularBase import UtCellularBase
from acs_test_scripts.TestStep.Device.Wireless.Cellular.SetPreferredNetwork import SetPreferredNetwork
from Core.TestStep.TestStepContext import TestStepContext


class SetPreferredNetworkTest(UtCellularBase):
    """
    SetPreferredNetwork test cases
    """

    def setUp(self):
        """
        Set up
        """
        UtCellularBase.setUp(self)
        self._context = TestStepContext()
        self._set_timeout = 0
        self._flight_mode_status = self.FLIGHT_MODE_OFF
        self._responsed = []

    def _return_get_flight_mode(self):
        """
        Stub method
        """
        return self._flight_mode_status

    def test_set_preferred_network_2g_only_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_2G_ONLY})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_2G_ONLY])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_2G_ONLY)

    def test_set_preferred_network_3g_only_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_3G_ONLY})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_3G_ONLY])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_3G_ONLY)

    def test_set_preferred_network_4g_only_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_4G_ONLY})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_4G_ONLY])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_4G_ONLY)

    def test_set_preferred_network_3g_pref_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_3G_PREF})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_3G_PREF])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_3G_PREF)

    def test_set_preferred_network_4g_pref_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_4G_PREF})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_4G_PREF])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_4G_PREF)

    def test_set_preferred_network_2g_3g_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_2G_3G})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_2G_3G])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_2G_3G)

    def test_set_preferred_network_cdma_pref_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_CDMA_PREF})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_CDMA_PREF])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_CDMA_PREF)

    def test_set_preferred_network_cdma_only_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_CDMA_ONLY})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_CDMA_ONLY])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_CDMA_ONLY)

    def test_set_preferred_network_evdo_only_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_EVDO_ONLY})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_EVDO_ONLY])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_EVDO_ONLY)

    def test_set_preferred_network_global_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_GLOBAL})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_GLOBAL])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_GLOBAL)

    def test_set_preferred_network_4g_pref_us_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_4G_PREF_US})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_4G_PREF_US])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_4G_PREF_US)

    def test_set_preferred_network_world_mode_ok(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_WORLD_MODE})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_WORLD_MODE])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_WORLD_MODE)

    def test_set_preferred_network_fail(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_4G_PREF})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_3G_PREF])
        self._assert_run_throw_device_exception(sut, "Set preferred network %s failure" % self.PREFERRED_NETWORK_4G_PREF)

    def test_set_preferred_network_flight_mode_on_fail(self):
        self._flight_mode_status = self.FLIGHT_MODE_ON
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_4G_PREF})
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_4G_PREF])
        self._assert_run_throw_device_exception(sut, "Can't set preferred network, Flight mode is currently ON")

    def test_set_preferred_network_ok_after_several_tries(self):
        sut = self._create_sut({"PREFERRED_NETWORK": self.PREFERRED_NETWORK_4G_PREF})
        sut._timeout = 5
        self._simulate_get_preferred_network_returning(sut, [self.PREFERRED_NETWORK_2G_ONLY,
                                                               self.PREFERRED_NETWORK_2G_ONLY,
                                                               self.PREFERRED_NETWORK_4G_PREF])
        self._assert_run_returned(sut, self.PREFERRED_NETWORK_4G_PREF)

    # ------------------------------------------------------------------------------------------------------------------

    def _side_effect_get_preferred_network_response(self):
        return self._responsed.pop(0)

    def _simulate_get_preferred_network_returning(self, sut, expected_state):
        self._responsed = expected_state
        sut._networking_api.get_preferred_network_type.side_effect = self._side_effect_get_preferred_network_response
        return sut

    def _assert_run_returned(self, sut, state):
        sut.run(None)
        sut._networking_api.set_preferred_network_type.assert_called_once_with(state)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = SetPreferredNetwork(None, None, test_step_pars, mock.Mock())
        sut._networking_api.get_flight_mode = self._return_get_flight_mode
        sut._timeout = 0
        return sut

