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

:organization: UMG PSI Validation
:summary: This file implements the test of CheckWifiDirectGroups
:since: 2014-08-05
:author: emarchan

"""

import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.WifiDirect.CheckWifiDirectGroups import CheckWifiDirectGroups

from acs_test_scripts.Device.UECmd.Imp.Android.KK.Networking.WifiDirect import WifiDirect

GO_NAME = "ACS_DUT1"
GROUP_OWNER = "GROUP_OWNER"
REMEMBERED_GROUPS_LIST = "REMEMBERED_GROUPS_LIST"

class CheckWifiDirectGroupsTest(UTestTestStepBase):
    """
    Checks if the name of the group (regarding the group owner) is included in a list of groups.
    """

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)

    def test_wheck_wifi_direct_groups_alpha_min_ok(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-aa-%s" % GO_NAME})
        sut.run(self._context)
        self._assert_run_succeeded(sut)

    def test_wheck_wifi_direct_groups_alpha_caps_ok(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-AA-%s" % GO_NAME})
        sut.run(self._context)
        self._assert_run_succeeded(sut)

    def test_wheck_wifi_direct_groups_alphanum_min_ok(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-a1-%s" % GO_NAME})
        sut.run(self._context)
        self._assert_run_succeeded(sut)

    def test_wheck_wifi_direct_groups_alphanum_caps_ok(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-1A-%s" % GO_NAME})
        sut.run(self._context)
        self._assert_run_succeeded(sut)

    def test_wheck_wifi_direct_groups_num_ok(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-11-%s" % GO_NAME})
        sut.run(self._context)
        self._assert_run_succeeded(sut)

    def test_wheck_wifi_direct_groups_bad_go_name_ko(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-aa-xx%s" % GO_NAME})
        self._assert_run_throw_device_exception(sut, "Can't find a matching group for peer %s" % GO_NAME)

    def test_wheck_wifi_direct_groups_letters_after_go_name_ko(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-aa-%sX" % GO_NAME})
        self._assert_run_throw_device_exception(sut, "Can't find a matching group for peer %s" % GO_NAME)


    def test_wheck_wifi_direct_groups_bad_random_numbers_ko(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIRECT-xxx-%s" % GO_NAME})
        self._assert_run_throw_device_exception(sut, "Can't find a matching group for peer %s" % GO_NAME)

    def test_wheck_wifi_direct_groups_bad_prefix_ko(self):
        sut = self._create_sut({GROUP_OWNER: GO_NAME, REMEMBERED_GROUPS_LIST: "DIREC-aa-%s" % GO_NAME})
        self._assert_run_throw_device_exception(sut, "Can't find a matching group for peer %s" % GO_NAME)


    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = CheckWifiDirectGroups(None, None, test_step_pars, mock.Mock())
        sut._api.check_wifi_direct_groups = WifiDirect(mock.MagicMock()).check_wifi_direct_groups
        return sut
