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

:organization: INTEL OTC Android
:summary: This file implements a Test Step that refreshes the Bluetooth scan from the Settings UI
:since:3/12/2015
:author: mmaraci
"""
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase


class BtRefreshScan(BtBase):
    """
    Test Step that refreshes the Bluetooth scan from the Settings UI
    """
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

        if self._bt_ui_actions.bt_refresh_scan_ui():
            self._bt_ui_actions.bt_wait_for_scan_end()
            self._bt_ui_actions.bt_look_for_device_in_list(self._pars.device_name_to_find)
