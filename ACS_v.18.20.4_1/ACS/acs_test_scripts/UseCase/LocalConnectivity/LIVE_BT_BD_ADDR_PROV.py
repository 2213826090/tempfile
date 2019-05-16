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
:summary: This file implements the live BT BD ADDRESS PROVISIONING UC (Changing BT MAC address)
:since: 27/09/2012
:author: cmichelx
"""

import time
import random
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from LIVE_BT_BASE import LiveBTBase
from string import hexdigits  # pylint: disable=W0402
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
import acs_test_scripts.Utilities.LocalConnectivityUtilities as LocalConnectivityUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveBTBdProv(LiveBTBase):

    """
    Live BT BD Address Provisioning test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LiveBTBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._prov_mode = \
            str(self._tc_parameters.get_param_value("PROV_MODE"))

        # Read New BD address to Set from test case xml file
        self._original_addr = \
            str(self._tc_parameters.get_param_value("NEW_ADDR"))

        self._default_addr = ""

        # initial phonesystem api
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

    def set_up(self):
        """
        Initialize the test
        """
        # pylint: disable=E1101
        UseCaseBase.set_up(self)

        # Check sw release in DUT is engineering build
        if "eng" not in str(self._device.device_properties.sw_release):
            msg = "TC blocked by userdebug sw release"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Check prov_mode value
        self._prov_mode = str(self._prov_mode).lower()
        if self._prov_mode not in ["no_prov", "prov"]:
            msg = "PROV_MODE value <%s> is not supported" % str(self._prov_mode)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # turn on bluetooth
        self._logger.info("Turn on Bluetooth adapters")
        self._bt_api.set_bt_power("on")
        time.sleep(self._wait_btwn_cmd)
        if self._bt_api.get_bt_power_status() != str(BT_STATE.STATE_ON):
            msg = "set BT ON failure"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Check address format
        if self._prov_mode == "prov":
            self._original_addr = self._original_addr.upper().strip()
            bdaddr_z = self._original_addr.replace('X', '0')
            if not NetworkingUtil.is_valid_mac_address(bdaddr_z):
                new_msg = "BAD format of NEW_ADDR parameter: <%s>" % self._original_addr
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, new_msg)
        else:
            # Check default BD_address after BT is ON
            self._default_addr = self._bt_api.get_default_addr()
            if self._default_addr == "":
                new_msg = "Unable to get default BD address <%s>" % self._default_addr
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, new_msg)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        LiveBTBase.run_test(self)
        time.sleep(self._wait_btwn_cmd)

        # Read BD address prior to update it
        prev_addr = self._bt_api.get_bt_adapter_address()
        new_addr = self._original_addr

        if self._prov_mode == "no_prov":
            # 1. Write null addr into chaabi
            self._bt_api.set_mac_addr('bt', LocalConnectivityUtil.NULL_ADDRESS)
            # 2. reboot
            self._device.reboot()
            # 3. Compare BDaddress with default one
            cur_addr = self._bt_api.get_bt_adapter_address()
            if self._compare_addr(cur_addr, self._default_addr):
                msg = "SUCCESS: BD address compliant with default address"
                self._logger.info(msg)
                msg = "addr=%s vs default=%s" % (cur_addr, self._default_addr)
                self._logger.info(msg)
            else:
                msg = "BD addr provisioned failure: previous=%s, current=%s, default=%s"\
                    % (prev_addr, cur_addr, self._default_addr)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            # 1. Check if there's a random
            if 'X' in self._original_addr:
                # Replace next x by a random char
                while 'X' in new_addr:
                    random_char = random.choice(hexdigits).upper()
                    new_addr = new_addr.replace('X', random_char, 1)
            # 2. Write new address into chaabi
            self._bt_api.set_mac_addr('bt', new_addr)
            # 3. reboot
            self._device.reboot()
            # 4. Compare BDaddress with expected one
            cur_addr = self._bt_api.get_bt_adapter_address()
            # current BD address shall be the BD address set
            if self._compare_addr(cur_addr, new_addr):
                msg = "SUCCESS: BD address is well updated"
                self._logger.info(msg)
                msg = "previous=%s, current=%s, new=%s" % (prev_addr, cur_addr, new_addr)
                self._logger.info(msg)
            else:
                msg = "BD addr provisioned failure: previous=%s, current=%s, new=%s"\
                    % (prev_addr, cur_addr, new_addr)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """
        # pylint: disable=E1101
        UseCaseBase.tear_down(self)
        time.sleep(self._wait_btwn_cmd)

        # turn off bluetooth
        self._logger.info("Turn off Bluetooth adapters")
        self._bt_api.set_bt_power("off")
        time.sleep(self._wait_btwn_cmd)
        if self._bt_api.get_bt_power_status() != str(BT_STATE.STATE_OFF):
            msg = "set BT OFF failure"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._prov_mode != "no_prov":
            self._logger.info("Restore default BD address")
            # 1. Write null addr into chaabi
            self._bt_api.set_mac_addr('bt', LocalConnectivityUtil.NULL_ADDRESS)
            # 2. reboot
            self._device.reboot()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _compare_addr(self, bdaddr1, bdaddr2):
        """
        Compare 2 BD addresses Byte per Byte
        Ignoring 'XX' bytes
        :type bdaddr1: str, format '00:00:00:00:00:00'
        :param bdaddr1: BD address to be compared to bdaddr2
        :type bdaddr2: str, format '00:00:00:00:00:00'
        :param bdaddr2: BD address to be compared to bdaddr1

        :rtype: Boolean
        :return: True = bd addresses are identical
        :return: False = bd addresses are different

        """

        # if "XX" value does not care
        bdaddr1 = str(bdaddr1).upper()
        laddr1 = bdaddr1.split(":")

        bdaddr2 = str(bdaddr2).upper()
        laddr2 = bdaddr2.split(":")

        result = True
        for i in range(len(laddr1)):
            if (laddr1[i] == "XX") or (laddr2[i] == "XX"):
                continue
            if laddr1[i] != laddr2[i]:
                result = False
                break

        return result
