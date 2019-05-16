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
:summary: This file implements the LIVE DUAL PHONE BT Base UC
:since: 12/07/2012
:author: apairex
"""
import time

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Utilities.LocalConnectivityUtilities import establish_bt_pairing
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveDualPhoneBTBase(LiveBTBase):

    """
    Live BT Test base class.
    """

    # Constants
    STR_PHONE_2 = "PHONE2"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LiveBTBase.__init__(self, tc_name, global_config)

        # Get PHONE2
        self._phone2 = DeviceManager().get_device(self.STR_PHONE_2)

        # Check if we have the second phone available
        if self._phone2 is not None:
            if not self._phone2.is_available():
                DeviceManager().boot_device(self.STR_PHONE_2)
            self._bt_api2 = self._phone2.get_uecmd("LocalConnectivity")

            # Get PHONE2 networking interface
            self._networking_api2 = self._phone2.get_uecmd("Networking")

            # Get BT device parameters
            self._dut2_config = DeviceManager().get_device_config(self.STR_PHONE_2)

        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        # Initialize phones address
        self._phone1_addr = ""
        self._phone2_addr = ""

        self._original_flight_mode2 = 0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        if self._phone2 is None:
            msg = "PHONE2 is not specified in the bench configuration"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        # Get original phone mode
        self._original_flight_mode2 = self._networking_api2.get_flight_mode()
        # Flight mode
        if self._use_flightmode != self._original_flight_mode2:
            self._networking_api2.set_flight_mode(self._use_flightmode)

        self._logger.info("Reset phone Bluetooth adapter")
        self._bt_api2.bt_reset_device()

        # Get phones address
        self._phone1_addr = self._bt_api.get_bt_adapter_address()
        self._phone2_addr = self._bt_api2.get_bt_adapter_address()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        LiveBTBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        # Recover to initial flight mode
        self._logger.info("Set PHONE2 to original flight mode")
        self._networking_api2.set_flight_mode(self._original_flight_mode2)
        time.sleep(self._wait_btwn_cmd)

        self._logger.info("Close phone 2 adapter devices")
        self._bt_api2.set_bt_power("off")
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

    def _establish_bt_pairing(self, pairing_initiator):
        """
        Establish bluetooth pairing procedure

        :type pairing_initiator: String
        :param pairing_initiator: PHONE1 or PHONE2 the device that requests for pairing first
        """
        # Defines BT pairing roles
        if pairing_initiator == "PHONE2":
            paired_api = self._bt_api
            paired_addr = self._phone1_addr
            requester_api = self._bt_api2
            requester_addr = self._phone2_addr

        else:
            paired_api = self._bt_api2
            paired_addr = self._phone2_addr
            requester_api = self._bt_api
            requester_addr = self._phone1_addr

        establish_bt_pairing(requester_api, requester_addr,
                             paired_api, paired_addr,
                             self._wait_btwn_cmd)
