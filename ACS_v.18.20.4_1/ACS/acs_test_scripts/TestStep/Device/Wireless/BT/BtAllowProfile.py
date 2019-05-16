"""
:copyright: (c)Copyright 2016, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC SQE
:summary: This teststep checks a checkbox profile into pair request window
:since:03/21/2016
:author: mmaraci
"""
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtAllowProfile(BtBase):
    """
    Implements the allow profile into pair request window test step
    """

    _PBAP_PROFILE = 'PBAP'
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

        profile = self._pars.profile
        if profile == self._PBAP_PROFILE:
            self._bt_ui_actions.bt_pbap_allow_contact_sharing(self._pars.fail_if_already, self._pars.must_exist)
