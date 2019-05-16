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
:summary: This file implements the Modem UEcmd for Android devices
:since: 2014-11-04
:author: emarchan

"""
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Communication.Modem import Modem as DadModem
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import TestConst
from acs_test_scripts.Device.UECmd.UECmdDecorator import need


class Modem(DadModem):

    """
    :summary: Modem UEcommands for Android platform
    """
    @need('modem')
    def __init__(self, device):
        DadModem.__init__(self, device)
        self._coex_manager_version = None

    def set_lte_coex_manager_messages(self, mode="VERBOSE"):
        """
        Enables or disables the LTE coex manager messages.

        @param mode : VERBOSE to enable the logs, empty to disable them.
        @type mode: string
        """
        if self._coex_manager_version == None:
            self._coex_manager_version = self.get_lte_coex_manager_version()
        if self._coex_manager_version == 1:
            self._exec("adb shell stop", force_execution=True, wait_for_response=True)
            self._exec("adb shell setprop log.tag.CWS_SERVICE_MGR-CoexMgr %s" % mode, force_execution=True, wait_for_response=True)
            self._exec("adb shell start", force_execution=True, wait_for_response=True)
        else:
            pass

    def get_lte_coex_manager_messages_state(self):
        """
        Gets the LTE coex manager state.

        @return : TestConst.STR_ON if enabled, TestConst.STR_OFF else.
        @rtype: string
        """
        if self._coex_manager_version == None:
            self._coex_manager_version = self.get_lte_coex_manager_version()
        if self._coex_manager_version == 1:
            cur_state = self._exec("adb shell getprop log.tag.CWS_SERVICE_MGR-CoexMgr")
            if "VERBOSE" in cur_state:
                result = TestConst.STR_ON
            else:
                result = TestConst.STR_OFF
        else:
            cur_state = self._exec("adb shell getprop config.disable_cellcoex")
            if "0" in cur_state:
                result = TestConst.STR_ON
            else:
                result = TestConst.STR_OFF
        return result

    def get_lte_coex_manager_version(self):
        is_coex_mgr_v1_enabled = False
        is_coex_mgr_v2_enabled = False
        coex_mgr_version = 2

        # Get V1 status
        cmd = "adb shell getprop persist.service.cwsmgr.coex"
        result = self._exec(cmd)
        if result == "1":
            is_coex_mgr_v1_enabled = True

        # Get V2 status
        cmd = "adb shell getprop config.disable_cellcoex"
        result = self._exec(cmd)
        if result == "0":
            is_coex_mgr_v2_enabled = True

        # Compute result
        if is_coex_mgr_v1_enabled == is_coex_mgr_v2_enabled:
            msg = "Coex MGR V1 and V2 can't have the same status!"
            self._logger.error(msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, "msg")

        if is_coex_mgr_v1_enabled:
            coex_mgr_version = 1

        self._logger.debug("Detected Cellular coex manager version %d" % coex_mgr_version)
        return coex_mgr_version

    def get_cell_power(self):
        """
        Gets the serving cell power in dBm

        @return : the registered cell power in dBm.
        @rtype: int
        """
        method = "getSignalStrength"

        self._logger.info("Get cell power")
        output = self._internal_exec_v2(self.__telephony_module, method, is_system=True)
        return int(output["Signal_Strength_Dbm"])
