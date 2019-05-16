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
:summary: This file implements the LAB_DUAL_PHONE_BT_OPP_A2DP_WITH_WIFI_ACTIVITIES
:author: jfranchx
:since:08/08/2013
"""

import time
import Queue
from threading import Thread

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadOPPTransferFile, opp_init_configure, opp_terminate
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadA2DPCheckMusic, a2dp_pair_connect_to_headset, \
a2dp_switch_music_state, a2dp_unpair_headset
from acs_test_scripts.Utilities.NetworkingUtilities import ThreadDownloadFileFromFTP
from acs_test_scripts.Device.UECmd.UECmdTypes import BtAudioState, BtAudioCmd
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabDualPhoneBTOppA2DPWithWiFiActivities(LiveDualPhoneBTBase, LiveWifiBase):
    """
    Lab BT OPP transfer with A2DP playing music and WiFi activities (switch On/Off and scans)
    """

    WAIT_OPP_ACTIVITY_START = 10.0
    OFFSET_TIMEOUT = 300

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)
        LiveWifiBase.__init__(self, tc_name, global_config)

        self._duration = str(self._tc_parameters.get_param_value("DURATION"))
        self._dut_state = str(self._tc_parameters.get_param_value("DUT_STATE"))
        self._opp_local_file = str(self._tc_parameters.get_param_value("OPP_LOCAL_FILE"))
        self._wifi_activity = str(self._tc_parameters.get_param_value("WIFI_ACTIVITY"))
        if self._wifi_activity == "WIFI_FTP_DOWNLOAD":
            self._ftp_remote_file = str(self._tc_parameters.get_param_value("FTP_REMOTE_FILE"))

        self._bt_headset = None
        self._bt_headset_addr = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        self._bt_headset = self._em.get_bluetooth_headset("BT_HEADSET")
        self._bt_headset_addr = self._bt_headset.get_bdaddress()

        if self._wifi_activity == "WIFI_FTP_DOWNLOAD":
            LiveWifiBase.set_up(self)
        else:
            self._networking_api.set_wifi_power(0)

        if not str(self._duration).isdigit():
            msg = "Bad value on duration parameter : %s" % self._duration
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._duration = int(self._duration)

        if self._dut_state not in ["DUT_CLIENT", "DUT_SERVER"]:
            msg = "DUT state configuration unknown - DUT should be client or server"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._wifi_activity not in ["WIFI_FTP_DOWNLOAD", "WIFI_SWITCH_SCAN_CONNECT"]:
            msg = "Unknown WiFi activity : %s" % self._wifi_activity
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Configure OPP transfer
        opp_init_configure(self._device, self._phone2)

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
        LiveDualPhoneBTBase.run_test(self)

        queue = Queue.Queue()

        # Configure BT OPP Transfer
        opp_timeout = LabDualPhoneBTOppA2DPWithWiFiActivities.OFFSET_TIMEOUT + self._duration
        if self._dut_state == "DUT_CLIENT":
            thread_opp_transfer = ThreadOPPTransferFile(queue, self._device, self._phone2,
                                                        self._opp_local_file, self._device.multimedia_path, 1, 0, opp_timeout)
        else:
            thread_opp_transfer = ThreadOPPTransferFile(queue, self._phone2, self._device,
                                                        self._opp_local_file, self._phone2.multimedia_path, 1, 0, opp_timeout)

        # Configure WiFi Activity
        if self._wifi_activity == "WIFI_FTP_DOWNLOAD":
            thread_wifi_activity = ThreadDownloadFileFromFTP(queue, self._networking_api,
                                                             self._server_ip_address, self._username,
                                                             self._password, self._ftp_remote_file, self._ftp_path,
                                                             str(self._device.get_ftpdir_path()),
                                                             LabDualPhoneBTOppA2DPWithWiFiActivities.OFFSET_TIMEOUT + self._duration)
        else:
            thread_wifi_activity = ThreadWiFiSwitchPowerScanAndConnect(queue, self._networking_api,
                                                                       self._duration, self._ssid,
                                                                       self._passphrase, self._security)

        # Configure A2DP Activity
        thread_a2dp_activity = ThreadA2DPCheckMusic(queue, BtAudioState.PLAYING, self._duration,
                                                    self._bt_api, self._bt_headset_addr)

        # Launch threads
        thread_opp_transfer.start()
        time.sleep(LabDualPhoneBTOppA2DPWithWiFiActivities.WAIT_OPP_ACTIVITY_START)
        thread_a2dp_activity.start()
        thread_wifi_activity.start()
        time.sleep(self._wait_btwn_cmd)
        self._exception_reader(queue, [thread_a2dp_activity,
                                       thread_opp_transfer, thread_wifi_activity])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """

        # Call LiveWifiBase tear_down function
        LiveWifiBase.tear_down(self)

        # Stop audio
        a2dp_switch_music_state(self._bt_api, self._bt_headset, BtAudioCmd.STOP)

        # Unpair device from BT Headset
        a2dp_unpair_headset(self._bt_api, self._bt_headset)

        # Clear OPP transfer
        opp_terminate(self._device, self._phone2)

        # Turn Off BT Adapters
        self._bt_api.set_bt_power(0)
        self._bt_api2.set_bt_power(0)

        # Delete sent file
        if self._dut_state == "DUT_CLIENT":
            self._bt_api2.bt_opp_init(self._opp_local_file)
        else:
            self._bt_api.bt_opp_init(self._opp_local_file)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------


class ThreadWiFiSwitchPowerScanAndConnect(Thread):
    """
    This thread switch WiFi power, do WiFi scans and connections.
    """

    # Time between "Macro Action" : turn On, scan, connect, turn Off
    WAIT_BETWEEN_ACTIONS = 4.0
    # Time between commands sent to DUT
    WAIT_BETWEEN_COMMANDS = 2.0

    def __init__(self, exceptions_queue, api_wifi, duration, wifi_ssid, wifi_password, wifi_security):
        """
        :type exceptions_queue : Queue Object
        :param exceptions_queue : used to send exceptions to the main thread
        :type api_wifi : WiFi API Object
        :param api_wifi : API to control WiFi features
        :type duration : integer
        :param duration : duration of the activity
        :type wifi_ssid : str
        :param wifi_ssid : Name of the WiFi network
        :type wifi_password : str
        :param wifi_password : password for the WiFi network
        :type wifi_security : str
        :param wifi_security : type of security of the WiFi network
        """
        Thread.__init__(self)
        self._exceptions_queue = exceptions_queue
        self._api_wifi = api_wifi
        self._duration = duration
        self._wifi_ssid = wifi_ssid
        self._wifi_password = wifi_password
        self._wifi_security = wifi_security

    def run(self):
        """
        Execute the thread.
        """

        start = time.time()
        try:
            while (start + self._duration) > time.time():
                # Power Up WiFi
                self._api_wifi.set_wifi_power("on")
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_ACTIONS)

                # Do WiFi scan
                self._api_wifi.request_wifi_scan()
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_COMMANDS)
                list_scanned_wifi = self._api_wifi.list_ssids("wifi", "all")

                device_found = False
                for element in list_scanned_wifi:
                    if str(element) == str(self._wifi_ssid):
                        device_found = True
                        break
                if not device_found:
                    msg = "WiFi Scan - device %s not found" % self._wifi_ssid
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_ACTIONS)

                # Connect to WiFi network
                self._api_wifi.wifi_remove_config("all")
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_COMMANDS)
                if self._wifi_security not in ("NONE", "WEP", "WPA", "WPA2", "OPEN"):
                    msg = "WiFi security not supported : %s" % self._wifi_security
                    raise DeviceException(DeviceException.INVALID_PARAMETER, msg)

                if self._wifi_security in ("WEP", "WPA", "WPA2"):
                    self._api_wifi.set_wificonfiguration(self._wifi_ssid, self._wifi_password, self._wifi_security)
                    time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_COMMANDS)
                self._api_wifi.wifi_connect(self._wifi_ssid)
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_ACTIONS)

                # Disconnect and power off WiFi
                self._api_wifi.wifi_remove_config(self._wifi_ssid)
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_COMMANDS)
                self._api_wifi.set_wifi_power("off")
                time.sleep(ThreadWiFiSwitchPowerScanAndConnect.WAIT_BETWEEN_ACTIONS)

        except AcsBaseException as local_exception:
            self._exceptions_queue.put(local_exception)
