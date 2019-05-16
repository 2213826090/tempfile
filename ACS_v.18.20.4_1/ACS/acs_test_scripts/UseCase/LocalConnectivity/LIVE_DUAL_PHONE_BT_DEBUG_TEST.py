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
:summary: This file implements the LIVE_DUAL_PHONE_BT_DEBUG_TEST
:author: apairex
:since:27/03/2013
"""
import time
from LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import BtProfile
from acs_test_scripts.UseCase.Misc.UECMD_TEST_TOOL import UECmdTestTool


class LiveDualPhoneBTDebugTest(LiveDualPhoneBTBase, UECmdTestTool):

    """
    Live Dual phone BT TEST UECmd UseCase
    """
    __MSG = "HELLO WORLD"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)
        UECmdTestTool.__init__(self, self.get_name(),
                               self._logger, self._device)

        self._fcts2test = list()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)
        UECmdTestTool.set_up(self, self.tc_order)

        """
        List of tests to perform. Each test is described as followed:
        [Label to print in the secondary report,
         UECmd to test,
         parameter(s) for the UECmd to test,
         Depends on the test names in the list]
        """
        bt_api = self._bt_api
        self._fcts2test = [
            {self._FCT: bt_api.get_bt_adapter_address},
            {self._FCT: bt_api.get_bt_power_status},
            {self._NAME: "set_bt_power OFF", self._FCT: bt_api.set_bt_power, self._PARMS: ["off"]},
            {self._NAME: "set_bt_power ON", self._FCT: bt_api.set_bt_power, self._PARMS: ["on"]},
            {self._FCT: bt_api.get_bt_scan_mode},
            {self._FCT: bt_api.bt_reset_device},
            {self._FCT: bt_api.bt_scan_devices},
            {self._NAME: "wait_for_pairing_canceled", self._FCT: self.__wait_for_pairing_canceled},
            {self._FCT: bt_api.set_bt_discoverable, self._PARMS: ["both", 120]},
            {self._NAME: "bt_find_device", self._FCT: self.__find_device},
            {self._NAME: "pair_to_device", self._FCT: self.__pair_device},
            {self._NAME: "bt_send_msg", self._FCT: self.__send_message, self._PARMS: [self.__MSG]},
            {self._NAME: "bt_receive_msg", self._FCT: self.__receive_message, self._PARMS: [self.__MSG]},
            {self._FCT: bt_api.bt_check_msg, self._PARMS: [self.__MSG]},
            {self._FCT: bt_api.bt_service_browsing, self._PARMS: [self._phone2_addr]},
            {self._NAME: "unpair_bt_device", self._FCT: self.__unpair_bt_device},
            {self._NAME: "wait_for_pairing", self._FCT: self.__wait_for_pairing},
            {self._FCT: bt_api.list_paired_device, self._DEP: ["wait_for_pairing"]},
            # {self._FCT: bt_api.bt_l2cap_ping, self._PARMS: [self._phone2_addr]},
            {self._FCT: bt_api.set_bt_tethering_power, self._PARMS: ["on"]},
            {self._FCT: bt_api.connect_bt_device, self._PARMS: [self._phone2_addr, BtProfile.PAN]},
            {self._FCT: bt_api.get_bt_connection_state, self._PARMS: [self._phone2_addr, BtProfile.HSP]},
            {self._FCT: bt_api.disconnect_bt_device, self._PARMS: [self._phone2_addr, BtProfile.PAN]},
            {self._NAME: "unpair_bt_device final", self._FCT: self.__unpair_bt_device}
        ]

        # UECmd to add later
        # ------------------
        # bt_opp_init
        # bt_opp_send_file
        # bt_opp_receive_file
        # connect_bt_hid_device
        # get_bt_audio_state

        # Not used EUCmd
        # --------------
        # set_agent_property
        # set_bt_authentication
        # set_bt_scanning
        # get_bt_autoconnect_status
        # set_bt_autoconnect
        # set_bt_pairable
        # get_bt_pairable_status
        # get_bt_pairable_timeout

        # set_bt_ctrl_event_mask
        # set_bt_ctrl_event_filter
        # activate_bt_test_mode
        # set_bt_default_link_policy

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LiveDualPhoneBTBase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # Check every EUCmd
        for fct2test in self._fcts2test:
            self._check_uecmd(fct2test)
            time.sleep(self._wait_btwn_cmd)

        # Raise an Exception in case of all tests do not pass
        self._compute_general_verdict()

        return Global.SUCCESS, "All UECmds OK"

#------------------------------------------------------------------------------

    def __find_device(self):
        """
        Function to test bt_find_device UECmd
        """
        # Set phone2 as discoverable
        self._bt_api2.set_bt_discoverable("both", 120)
        time.sleep(self._wait_btwn_cmd)

        # Test find device function
        self._bt_api.bt_find_device(self._phone2_addr)

    def __pair_device(self):
        """
        Function to test pair_to_device UECmd
        """
        # Set PHONE2 as discoverable
        self._bt_api2.set_bt_discoverable("both", 0)
        time.sleep(self._wait_btwn_cmd)
        self._bt_api2.wait_for_pairing(self._phone1_addr, 1, 1)

        # Pair to PHONE2
        self._bt_api.pair_to_device(self._phone2_addr, 1)

    def __unpair_bt_device(self):
        """
        Function to test unpair_bt_device UECmd
        """
        self._bt_api.unpair_bt_device(self._phone2_addr)
        self._bt_api2.unpair_bt_device(self._phone1_addr)

    def __wait_for_pairing(self):
        """
        Function to test wait_for_pairing UECmd
        """
        # Set PHONE1 as discoverable
        self._bt_api.set_bt_discoverable("both", 0)
        time.sleep(self._wait_btwn_cmd)
        self._bt_api.wait_for_pairing(self._phone2_addr, 1, 1)

        # Pair PHONE2 to PHONE1
        self._bt_api2.pair_to_device(self._phone1_addr, 1)

    def __wait_for_pairing_canceled(self):
        """
        Function to test wait_for_pairing_canceled UECmd
        """
        self._bt_api2.set_bt_discoverable("noscan", 0)
        try:
            self._bt_api.wait_for_pairing(self._phone2_addr, 1, 1)
        except:  # pylint: disable=W0702
            # Prevent for an exception to occurs.
            # The objective is well to fail the pairing
            pass
        self._bt_api.wait_for_pairing_canceled()

    def __receive_message(self, msg):
        """
        Function to test bt_receive_msg UECmd

        :type msg: str
        :param msg: message to receive
        """
        self._bt_api.bt_receive_msg()
        time.sleep(self._wait_btwn_cmd)
        self._bt_api2.bt_send_msg(self._phone1_addr, msg)

    def __send_message(self, msg):
        """
        Function to test bt_send_msg UECmd

        :type msg: str
        :param msg: message to send
        """
        self._bt_api2.bt_receive_msg()
        time.sleep(self._wait_btwn_cmd)
        self._bt_api.bt_send_msg(self._phone2_addr, msg)
