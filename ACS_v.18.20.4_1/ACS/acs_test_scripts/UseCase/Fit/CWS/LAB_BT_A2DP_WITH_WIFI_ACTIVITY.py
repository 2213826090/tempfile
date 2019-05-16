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
:summary: This file implements the LAB_BT_A2DP_WITH_WIFI_ACTIVITY
:author: jfranchx
:since:07/08/2013
"""

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import a2dp_pair_connect_to_headset, a2dp_unpair_headset, \
a2dp_switch_music_state, ThreadA2DPCheckMusic, ThreadA2DPSwitchPlaying, ThreadA2DPSwitchVolume, ThreadA2DPSwitchSong
from acs_test_scripts.Utilities.NetworkingUtilities import ThreadWebBrowsing, ThreadDownloadFileFromFTP
from acs_test_scripts.Device.UECmd.UECmdTypes import BtAudioCmd, BtAudioState
import Queue
import time
from ErrorHandling.AcsConfigException import AcsConfigException


class LabBTA2DPWithWiFiActivity(LiveBTBase, LiveWifiBase):
    """
    Live BT A2DP play music with web browsing or FTP download
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)
        LiveWifiBase.__init__(self, tc_name, global_config)

        self._audio_file = str(self._tc_parameters.get_param_value("AUDIO_FILE"))
        self._duration = str(self._tc_parameters.get_param_value("DURATION"))
        self._a2dp_activity = str(self._tc_parameters.get_param_value("A2DP_ACTIVITY")).upper()
        self._wifi_activity = str(self._tc_parameters.get_param_value("WIFI_ACTIVITY")).upper()

        if self._wifi_activity == "WIFI_WEB_BROWSING":
            self._wifi_url = str(self._tc_parameters.get_param_value("URL_TO_BROWSE"))
        elif self._wifi_activity == "WIFI_FTP_DOWNLOAD":
            self._ftp_remote_file = str(self._tc_parameters.get_param_value("FTP_REMOTE_FILE"))
            self._timeout = 300

        self._bt_headset = self._em.get_bluetooth_headset("BT_HEADSET")
        self._bt_headset_addr = None

        self._thread_wifi_activity = None
        self._thread_a2dp_activity = None
        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)
        LiveWifiBase.set_up(self)

        self._bt_headset_addr = self._bt_headset.get_bdaddress()

        if not str(self._duration).isdigit():
            msg = "Bad value on duration parameter : %s" % self._duration
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._duration = int(self._duration)

        # Connect to BT Headset
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

        # Configure threads
        if self._wifi_activity == "WIFI_WEB_BROWSING":
            self._thread_wifi_activity = ThreadWebBrowsing(self._queue,
                                                           self._networking_api,
                                                           self._duration,
                                                           self._wifi_url)
        elif self._wifi_activity == "WIFI_FTP_DOWNLOAD":
            self._thread_wifi_activity = ThreadDownloadFileFromFTP(self._queue, self._networking_api,
                                                                   self._server_ip_address, self._username,
                                                                   self._password, self._ftp_remote_file,
                                                                   self._ftp_path,
                                                                   str(self._device.get_ftpdir_path()), self._timeout)
        else:
            msg = "Unknown WIFI_ACTIVITY : %s" % self._wifi_activity
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._a2dp_activity == "A2DP_CHECK_AUDIO":
            self._thread_a2dp_activity = ThreadA2DPCheckMusic(self._queue,
                                                              BtAudioState.PLAYING, self._duration,
                                                              self._bt_api, self._bt_headset_addr)
        elif self._a2dp_activity == "A2DP_SWITCH_VOLUME":
            self._thread_a2dp_activity = ThreadA2DPSwitchVolume(self._queue,
                                                                self._duration,
                                                                self._bt_api, self._bt_headset)
        elif self._a2dp_activity == "A2DP_SWITCH_SONG":
            self._thread_a2dp_activity = ThreadA2DPSwitchSong(self._queue,
                                                              self._duration,
                                                              self._bt_api, self._bt_headset)
        elif self._a2dp_activity == "A2DP_SWITCH_PLAYING":
            self._thread_a2dp_activity = ThreadA2DPSwitchPlaying(self._queue, BtAudioState.PLAYING,
                                                                 self._duration, 1.0,
                                                                 self._bt_api, self._bt_headset)
        else:
            msg = "Unknown A2DP_ACTIVITY : %s" % self._a2dp_activity
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Launch threads
        self._thread_wifi_activity.start()
        self._thread_a2dp_activity.start()
        time.sleep(self._wait_btwn_cmd)

        # This function waits all threads are dead or timeout is over before finish
        self._exception_reader(self._queue, [self._thread_wifi_activity, self._thread_a2dp_activity])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """
        # Call UseCase base tear_down function
        LiveWifiBase.tear_down(self)

        # Stop audio
        a2dp_switch_music_state(self._bt_api, self._bt_headset, BtAudioCmd.STOP)

        # Unpair device from BT Headset
        a2dp_unpair_headset(self._bt_api, self._bt_headset)

        # Disable BT
        self._bt_api.set_bt_power(0)

        return Global.SUCCESS, "No errors"
