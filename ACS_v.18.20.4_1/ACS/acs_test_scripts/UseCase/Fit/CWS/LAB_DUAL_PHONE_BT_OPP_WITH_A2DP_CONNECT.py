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
:summary: This file implements the LAB_DUAL_PHONE_BT_OPP_WITH_A2DP_CONNECT
:author: jfranchx
:since:12/08/2013
"""

import time
import Queue
import threading

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadOPPTransferFile, \
a2dp_pair_connect_to_headset, a2dp_unpair_headset, opp_init_configure, opp_terminate
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabDualPhoneBTOppWithA2DPConnect(LiveDualPhoneBTBase):
    """
    Lab BT OPP transfer with BT pairing to headset with A2DP
    """

    WAIT_A2DP_ACTIVITY = 10.0

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)

        self._dut_state = str(self._tc_parameters.get_param_value("DUT_STATE"))
        self._opp_local_file = str(self._tc_parameters.get_param_value("OPP_LOCAL_FILE"))
        self._duration = str(self._tc_parameters.get_param_value("DURATION"))

        self._thread_opp_transfer = None
        self._thread_headset_activity = None
        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        # Configure iterations
        if not str(self._duration).isdigit() or int(self._duration) < 1:
            msg = "DURATION parameter error - must be >= 1 but is %s" % self._duration
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._duration = int(self._duration)

        if self._dut_state not in ["DUT_CLIENT", "DUT_SERVER"]:
            msg = "DUT state configuration unknown - DUT should be client or server"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Configure OPP Transfer
        opp_init_configure(self._device, self._phone2)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # Configure BT OPP Transfer thread
        # Files used are big, use bigger timeout than default
        opp_timeout = 300 + self._duration
        if self._dut_state == "DUT_CLIENT":
            self._thread_opp_transfer = ThreadOPPTransferFile(self._queue, self._device, self._phone2,
                                                              self._opp_local_file, self._device.multimedia_path,
                                                              1, 0, opp_timeout)
        else:
            self._thread_opp_transfer = ThreadOPPTransferFile(self._queue, self._phone2, self._device,
                                                              self._opp_local_file, self._phone2.multimedia_path,
                                                              1, 0, opp_timeout)

        # Configure Pair/Unpair activity thread
        self._thread_headset_activity = ThreadPairConnectA2DPHeadset(self._queue, self._duration, self._bt_api,
                                                                     self._em.get_bluetooth_headset("BT_HEADSET"))

        # Launch threads
        self._thread_opp_transfer.start()
        time.sleep(LabDualPhoneBTOppWithA2DPConnect.WAIT_A2DP_ACTIVITY)
        self._thread_headset_activity.start()
        time.sleep(self._wait_btwn_cmd)
        self._exception_reader(self._queue, [self._thread_opp_transfer, self._thread_headset_activity])

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
            self._bt_api.bt_opp_init(self._opp_local_file)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------


class ThreadPairConnectA2DPHeadset(threading.Thread):
    """
    This thread pair/connect and unpair/disconnect A2DP headset to an other device
    """

    WAIT_TIME_BETWEEN_COMMAND = 20.0

    def __init__(self, exceptions_queue, duration, phone_bt, headset_bt):
        """
        Constructor

        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type duration : integer
        :param duration : time of activity
        :type phone_bt : Bluetooth API Object
        :param phone_bt : API for Bluetooth functions
        :type headset_bt : Bluetooth Headset API Object
        :param headset_bt : API of the Bluetooth headset
        """
        threading.Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._duration = duration
        self._phone_bt = phone_bt
        self._headset_bt = headset_bt

    def run(self):
        try:
            start = time.time()
            while(start + self._duration) > time.time():
                a2dp_pair_connect_to_headset(self._phone_bt, self._headset_bt)
                time.sleep(self.WAIT_TIME_BETWEEN_COMMAND)
                a2dp_unpair_headset(self._phone_bt, self._headset_bt)
                time.sleep(self.WAIT_TIME_BETWEEN_COMMAND)

        except AcsBaseException as acs_exception:
            self._exceptions_queue.put(acs_exception)
