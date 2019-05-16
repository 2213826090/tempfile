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
:summary: This file implements the LIVE BT Pairing UC
:author: ssavrimoutou
:since:30/09/2010
"""

import time

from LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LiveBTPairing(LiveBTBase):

    """
    Live BT Pairing test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        # Read CODE_PIN from test case xml file
        self._code_pin = \
            str(self._tc_parameters.get_param_value("CODE_PIN"))

        # Read PASSPHRASE from test case xml file
        self._passphrase = \
            str(self._tc_parameters.get_param_value("PASSPHRASE"))

        # Read DEVICE_BT_ADDRESS from test case xml file or BenchConfig
        self._device_bt_address = \
            str(self._tc_parameters.get_param_value("DEVICE_BT_ADDRESS"))
        if self._device_bt_address is None or self._device_bt_address == "" \
                or self._device_bt_address.lower() == "none":
            # Then search for the BT MAC address into the BenchConfig
            self._bench_bt_device = global_config.benchConfig.\
                get_parameters("BT_DEVICE")
            self._device_bt_address = self._bench_bt_device.\
                get_param_value("MacAddress")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        # Start a first BT scan on both devices to list visible remote
        self._bt_api.bt_scan_devices()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        # Pair to the device DEVICE_BT_ADDRESS, remove it if it is already paired
        time.sleep(self._wait_btwn_cmd)
        self._bt_api.pair_to_device(self._device_bt_address, 1)

        # Check if DUT is paired to DEVICE_BT_ADDRESS
        time.sleep(self._wait_btwn_cmd)
        list_paired_devices = \
            self._bt_api.list_paired_device()

        time.sleep(self._wait_btwn_cmd)
        remote_paired = False
        for element in list_paired_devices:
            if str(element.address).upper() == \
                    str(self._device_bt_address).upper():
                self._logger.info("Pair to device %s success"
                                  % self._device_bt_address)

                # Remove pairing from DEVICE_BT_ADDRESS
                time.sleep(self._wait_btwn_cmd)
                self._bt_api.unpair_bt_device(self._device_bt_address)
                remote_paired = True
                break

        if not remote_paired:
            msg = "Pairing to device %s failed" % self._device_bt_address
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
