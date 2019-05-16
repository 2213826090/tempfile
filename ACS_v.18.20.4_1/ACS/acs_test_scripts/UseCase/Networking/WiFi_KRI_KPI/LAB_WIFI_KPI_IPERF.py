"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements the LAB WIFI KPI IPERF UC
@since: 27/03/2014
@author: jfranchx
"""
import time
import Queue
import re

from acs_test_scripts.UseCase.Networking.LAB_WIFI_IPERF import LabWifiIperf
from acs_test_scripts.UseCase.Networking.WiFi_KRI_KPI.LAB_WIFI_KPI_BASE import LabWifiKPIBase
from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadA2DPCheckMusic, \
    a2dp_pair_connect_to_headset, a2dp_unpair_headset, a2dp_switch_music_state
from acs_test_scripts.Device.UECmd.UECmdTypes import BtAudioCmd, BtAudioState


class LabWifiKPIIperf(LabWifiKPIBase, LabWifiIperf, LiveBTBase):
    """
    Lab WiFi KPI Iperf Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabWifiIperf.__init__(self, tc_name, global_config)
        LiveBTBase.__init__(self, tc_name, global_config)
        LabWifiKPIBase.__init__(self, tc_name, global_config)

        self._bt_play_music = self._tc_parameters.get_param_value("BT_PLAY_MUSIC")
        self._bt_headset = None
        self._bt_headset_addr = None
        self._thread_headset_check_music = None
        self._queue = Queue.Queue()

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiIperf.set_up(self)
        LiveBTBase.set_up(self)

        # Check values
        if self._bt_play_music is not None and self._bt_play_music not in ["on", "off"]:
            msg = "Bad value for BT_PLAY_MUSIC : %s" % self._bt_play_music
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Setup DUT for KPI TC
        LabWifiKPIBase.set_up(self)

        if self._bt_play_music == "on":
            LiveBTBase.set_up(self)
            # Configure Headset and launch audio
            self._bt_headset = self._em.get_bluetooth_headset("BT_HEADSET")
            self._bt_headset_addr = self._bt_headset.get_bdaddress()
            a2dp_pair_connect_to_headset(self._bt_api, self._bt_headset)
            a2dp_switch_music_state(self._bt_api, self._bt_headset, BtAudioCmd.PLAY)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run test
        """

        LabWifiKPIBase.run_test(self)
        # Flush TCP records
        self._networking_api.clean_tcp_records_device()
        self._computer.clean_tcp_records_computer()

        if self._bt_play_music == "on":
            # Search duration in iperf options
            duration = 20 + int((re.search("(-t[ ]*)([0-9]+)", self._iperf_options)).group(2))
            # Configure and launch check music thread
            self._thread_headset_check_music = ThreadA2DPCheckMusic(self._queue, BtAudioState.PLAYING,
                                                                    duration, self._bt_api, self._bt_headset_addr)
            self._thread_headset_check_music.start()

        time.sleep(self._wait_btwn_cmd)
        iperf_result, iperf_msg = LabWifiIperf.run_test(self)

        if self._bt_play_music == "on":
            self._exception_reader(self._queue, [self._thread_headset_check_music])

        complete_msg = "RSSI value : %s -- %s" % (self._rssi_value, iperf_msg)
        return iperf_result, complete_msg

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiIperf.tear_down(self)

        # Disable BT if required
        if self._bt_play_music == "on":
            a2dp_switch_music_state(self._bt_api, self._bt_headset, BtAudioCmd.PAUSE)
            a2dp_unpair_headset(self._bt_api, self._bt_headset)
            self._bt_api.set_bt_power(0)

        # Reconfigure default device state
        LabWifiKPIBase.tear_down(self)

        return Global.SUCCESS, "No errors"
