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
:summary: This file implements the LAB_BT_SCAN_WITH_A2DP_PLAYING
:author: jfranchx
:since:12/08/2013
"""

import time
import Queue

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadScanBTRemoteDevice, ThreadA2DPCheckMusic, \
a2dp_pair_connect_to_headset, a2dp_unpair_headset, a2dp_switch_music_state
from acs_test_scripts.Device.UECmd.UECmdTypes import BtAudioCmd, BtAudioState
from ErrorHandling.AcsConfigException import AcsConfigException


class LabBTScanWithA2DPPlaying(LiveBTBase):
    """
    Lab BT OPP transfer with BT pairing to headset with A2DP.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)

        self._ref_bt_addr = str(self._tc_parameters.get_param_value("REF_BT_ADDR"))
        self._duration = str(self._tc_parameters.get_param_value("DURATION"))

        self._bench_config = global_config.benchConfig.get_parameters("BT_DEVICE")
        self._bt_headset = self._em.get_bluetooth_headset("BT_HEADSET")
        self._bt_headset_addr = None
        self._thread_bt_scan = None
        self._thread_headset_check_music = None
        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        self._bt_headset_addr = self._bt_headset.get_bdaddress()

        if self._ref_bt_addr.lower() in ["none", ""] and self._bench_config is not None:
            self._ref_bt_addr = str(self._bench_config.get_param_value("MacAddress"), "")

        if self._ref_bt_addr.lower() in ["none", "", "00:00:00:00:00:00"]:
            msg = "No BD address found in the TC and/or in bench_config"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        if not str(self._duration).isdigit():
            msg = "Bad value on duration parameter : %s" % self._duration
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._duration = int(self._duration)

        # Configure Headset
        a2dp_pair_connect_to_headset(self._bt_api, self._bt_headset)

        # Launch audio
        a2dp_switch_music_state(self._bt_api, self._bt_headset, BtAudioCmd.PLAY)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        # Configure BT Scan
        self._thread_bt_scan = ThreadScanBTRemoteDevice(self._queue, self._bt_api, self._ref_bt_addr, self._duration)

        # Configure A2DP check music
        self._thread_headset_check_music = ThreadA2DPCheckMusic(self._queue, BtAudioState.PLAYING,
                                                                self._duration, self._bt_api, self._bt_headset_addr)

        # Launch threads
        self._thread_headset_check_music.start()
        self._thread_bt_scan.start()
        time.sleep(self._wait_btwn_cmd)
        self._exception_reader(self._queue, [self._thread_bt_scan, self._thread_headset_check_music])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """
        # Call UseCase base tear_down function
        UseCaseBase.tear_down(self)

        # Stop audio
        a2dp_switch_music_state(self._bt_api, self._bt_headset, BtAudioCmd.PAUSE)

        # Unpair device from BT Headset
        a2dp_unpair_headset(self._bt_api, self._bt_headset)

        # Disable BT
        self._bt_api.set_bt_power(0)

        return Global.SUCCESS, "No errors"
