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
:summary: This file implements the LIVE_DUAL_PHONE_BT_TRANSFER_SWITCH_BT
:author: jfranchx
:since:09/07/2013
"""

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadOPPTransferFile, opp_init_configure, opp_terminate
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
import time
import Queue
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveDualPhoneBTTransferSwitchBT(LiveDualPhoneBTBase):
    """
    Live BT Transfer with disable BT
    """
    WAIT_BETWEEN_BT_SWITCH = 15.0

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        # Read DUT_STATE from test case xml file
        self._dut_state = str(self._tc_parameters.get_param_value("DUT_STATE"))
        # Read OPP_LOCAL_FILE from test case xml file
        self._opp_local_file = str(self._tc_parameters.get_param_value("OPP_LOCAL_FILE"))

        # Initialize data
        self._thread_opp_transfer = None
        self._thread_opp_transfer_retry = None

        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        if self._dut_state not in ["DUT_CLIENT", "DUT_SERVER"]:
            msg = "DUT state configuration unknown - DUT should be client or server"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        opp_init_configure(self._device, self._phone2)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101

        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # Configure BT OPP Transfer
        if self._dut_state == "DUT_CLIENT":
            self._thread_opp_transfer = ThreadOPPTransferFile(self._queue, self._device, self._phone2,
                                                              self._opp_local_file, self._device.multimedia_path)
            self._thread_opp_transfer_retry = ThreadOPPTransferFile(self._queue, self._device, self._phone2,
                                                                    self._opp_local_file, self._device.multimedia_path)
        else:
            self._thread_opp_transfer = ThreadOPPTransferFile(self._queue, self._phone2, self._device,
                                                              self._opp_local_file, self._phone2.multimedia_path)
            self._thread_opp_transfer_retry = ThreadOPPTransferFile(self._queue, self._phone2, self._device,
                                                                    self._opp_local_file, self._phone2.multimedia_path)

        self._thread_opp_transfer.start()
        time.sleep(LiveDualPhoneBTTransferSwitchBT.WAIT_BETWEEN_BT_SWITCH)

        # Switch On/Off Bluetooth
        self._bt_api.set_bt_power(str(BT_STATE.STATE_OFF))
        time.sleep(LiveDualPhoneBTTransferSwitchBT.WAIT_BETWEEN_BT_SWITCH)
        if self._bt_api.get_bt_power_status() != str(BT_STATE.STATE_OFF):
            msg = "error switch off Bluetooth adapter"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._bt_api.set_bt_power(str(BT_STATE.STATE_ON))
        time.sleep(LiveDualPhoneBTTransferSwitchBT.WAIT_BETWEEN_BT_SWITCH)

        self._thread_opp_transfer.join()
        try:
            current_exception = self._queue.get(True, 3)

        except Queue.Empty:
            current_exception = None

        if current_exception is None:
            msg = "no error return after switch on/off BT during transfer"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._thread_opp_transfer_retry.start()
        time.sleep(self._wait_btwn_cmd)
        self._exception_reader(self._queue, [self._thread_opp_transfer_retry])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """

        # Call UseCase base tear_down function
        LiveDualPhoneBTBase.tear_down(self)

        # Clear OPP transfer
        opp_terminate(self._device, self._phone2)

        # Delete sent file
        if self._dut_state == "DUT_CLIENT":
            self._bt_api2.bt_opp_init(self._opp_local_file)
        else:
            # DUT_SERVER case
            self._bt_api.bt_opp_init(self._opp_local_file)

        return Global.SUCCESS, "No errors"
