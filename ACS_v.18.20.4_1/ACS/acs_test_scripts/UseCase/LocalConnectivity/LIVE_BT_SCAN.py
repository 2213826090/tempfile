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
:summary: This file implements the LIVE BT Scan UC in flight/normal mode
:since: 29/09/2010
:author: skgurusX , vgombert , npan2
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveBTScan(LiveBTBase):

    """
    Live BT Scan test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LiveBTBase.__init__(self, tc_name, global_config)

        # Get TC Parameters

        # Read DEVICE_BT_ADDRESS from test case xml file or BenchConfig
        self._device_to_search = \
            str(self._tc_parameters.get_param_value("DEVICE_TO_SEARCH"))
        if self._device_to_search.lower() in ["none", ""]:
            # Then search for the BT MAC address into the BenchConfig
            bench_bt_device = global_config.benchConfig\
                .get_parameters("BT_DEVICE")
            if bench_bt_device is not None:
                if str(bench_bt_device.get_param_value("MacAddress")).lower()\
                        not in ["none", "", "00:00:00:00:00:00"]:
                    self._device_to_search = str(bench_bt_device
                                                 .get_param_value("MacAddress"))
                else:
                    msg = "No BD addr defined in the TC and/or in bench_config"
                    self._logger.error(msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            else:
                msg = "No BD address defined in the TC and/or in bench_config"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LiveBTBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # Split to create a list of BD-ADDR in case of multiple search
        bd_addr_list = self._device_to_search.strip().split(";")
        for bdaddr in bd_addr_list:

            # Remove leading and trailing whitespace
            bdaddr = str(bdaddr).strip()

            # start scan , and find specific device
            self._logger.info("Looking for " + bdaddr)
            if not self._bt_api.bt_find_device(bdaddr):
                msg = "Device %s not found" % bdaddr
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"
