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
:summary: This file implements a wait for profile connection test step
:since:08/01/2014
:author: fbongiax
"""
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtEnableProfile(BtBase):
    """
    Implements the connect profile test step
    """

    _HSP_PROFILE = 'HSP'
    _A2DP_PROFILE = 'A2DP'
    _HID_PROFILE = 'HID'
    _PAN_PROFILE = 'PAN'
    PY_UIAUTOMATOR = "PyUiAutomator"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._bt_ui_actions = self._device.get_uecmd(self.PY_UIAUTOMATOR)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)

        self._bt_ui_actions._bt_open_pairing_menu(self._pars.device_name)

        if self._pars.enable is None or self._pars.enable is True:
            self._enable()
        else:
            self._disable()

        self._bt_ui_actions._bt_dismiss_pairing_menu(self._pars.enable)

    def _enable(self):
        """
        Enable the profile
        """
        if self._pars.profile == self._A2DP_PROFILE:
            self._bt_ui_actions._bt_enable_profile(self._bt_ui_actions.A2DP_PROFILE,
                                                   fail_if_already_enabled=self._pars.fail_if_already)
        elif self._pars.profile == self._HSP_PROFILE:
            self._bt_ui_actions._bt_enable_profile(self._bt_ui_actions.HSP_PROFILE,
                                                   fail_if_already_enabled=self._pars.fail_if_already)
        elif self._pars.profile == self._HID_PROFILE:
            self._bt_ui_actions._bt_enable_profile(self._bt_ui_actions.HID_PROFILE,
                                                   fail_if_already_enabled=self._pars.fail_if_already)
        elif self._pars.profile == self._PAN_PROFILE:
            self._bt_ui_actions._bt_enable_profile(self._bt_ui_actions.PAN_PROFILE,
                                                   fail_if_already_enabled=self._pars.fail_if_already)

    def _disable(self):
        """
        Disable the profile
        """
        if self._pars.profile == self._A2DP_PROFILE:
            self._bt_ui_actions._bt_disable_profile(self._bt_ui_actions.A2DP_PROFILE,
                                                    fail_if_already_disabled=self._pars.fail_if_already)
        elif self._pars.profile == self._HSP_PROFILE:
            self._bt_ui_actions._bt_disable_profile(self._bt_ui_actions.HSP_PROFILE,
                                                    fail_if_already_disabled=self._pars.fail_if_already)
        elif self._pars.profile == self._HID_PROFILE:
            self._bt_ui_actions._bt_disable_profile(self._bt_ui_actions.HID_PROFILE,
                                                    fail_if_already_disabled=self._pars.fail_if_already)
        elif self._pars.profile == self._PAN_PROFILE:
            self._bt_ui_actions._bt_disable_profile(self._bt_ui_actions.PAN_PROFILE,
                                                    fail_if_already_disabled=self._pars.fail_if_already)
