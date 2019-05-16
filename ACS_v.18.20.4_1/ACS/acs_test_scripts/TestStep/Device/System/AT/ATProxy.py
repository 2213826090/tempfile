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

:organization: INTEL NDG
:summary: AT Proxy tools
:since: 13/01/2014
:author: gcharlex
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.ModemFlashing import ModemFlashing


class ATProxy(DeviceTestStepBase):
    """
    AT Proxy class
    """
    def __init__(self, tc_name, global_config, ts_conf, factory):
        """
        Constructor
        """
        # Call DeviceTestStepBase base Init function
        DeviceTestStepBase.__init__(self, tc_name, global_config, ts_conf, factory)

        # Get UECmdLayer
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # Check input parameter ACTION
        if self._pars.action not in ["START", "STOP"]:
            self._raise_config_exception("ACTION argument's value (%s) is invalid" %
                                         self._pars.action)

        # Check input parameter BOOT_MODE
        if self._pars.boot_mode not in ["POS", "MOS"]:
            self._raise_config_exception("BOOT_MODE argument's value (%s) is invalid" %
                                         self._pars.boot_mode)

        # Check input parameter PROXY_MODE
        possible_proxy_mode = {"NORMAL": ModemFlashing.AT_PROXY_NORMAL_MODE,
                                "TUNNELING": ModemFlashing.AT_PROXY_TUNNELING_MODE}
        if self._pars.proxy_mode in possible_proxy_mode.keys():
            proxy_mode = possible_proxy_mode[self._pars.proxy_mode]
        else:
            proxy_mode = None

        if self._pars.action == "START":
            self._logger.info("Starting AT proxy")
            if self._pars.boot_mode == "POS":
                at_proxy_tty = self._modem_flashing_api.start_at_proxy_from_pos()
            elif self._pars.boot_mode == "MOS":
                at_proxy_tty = self._modem_flashing_api.start_at_proxy_from_mos(proxy_mode)

            # Save the command result in the context variable
            context.set_info(self._pars.save_at_proxy_tty_as, at_proxy_tty)

        elif self._pars.action == "STOP":
            self._logger.info("Stop AT proxy")
            if self._pars.boot_mode == "POS":
                self._modem_flashing_api.stop_at_proxy_from_pos()
            elif self._pars.boot_mode == "MOS":
                self._modem_flashing_api.stop_at_proxy_from_mos()