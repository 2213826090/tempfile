"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: UMG WSIV System Tests
:summary: This file implements LAB SYSTEM SLEEP WIFI ON UC
    i.e. Test S3/S0i3 residency with wifi on/connected:
    -Check that platform enters/exits S3/S0i3 with wifi on and scan offloaded
:since: 26/11/2012
:author: jpstierlin RTC24950 Add S3/S0i3 tests
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import str_to_bool
from ErrorHandling.DeviceException import DeviceException


class LabSystemSleepWifiOn(LabWifiBase):

    """
    S3/S0i3 WiFi On class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters

        # Sleep mode
        self._sleep_mode = self._tc_parameters.get_param_value("SLEEP_MODE")

        # Settle time
        self._settle_time = \
            int(self._tc_parameters.get_param_value("SETTLE_TIME"))

        # Sleep time duration in seconds
        self._duration = int(self._tc_parameters.get_param_value("DURATION"))

        # Auto Connect
        auto_connect = self._tc_parameters.get_param_value("AUTO_CONNECT")
        if auto_connect in ["", None]:
            self._auto_connect = False
        else:
            self._auto_connect = str_to_bool(auto_connect)

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._residency_api = self._device.get_uecmd("Residencies")

        # Residency rate (sleep duration / test length)
        self._target_residency_rate = \
            self._tc_parameters.get_param_value("TARGET_RESIDENCY_RATE")
        if self._target_residency_rate is not None:
            self._target_residency_rate = float(self._target_residency_rate)
        else:
            self._target_residency_rate = 0.0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        # Do not call LabWifiBase.set_up as we don't want to connect
        UseCaseBase.set_up(self)

        # Initiate connection to the equipment
        self._ns.init()

        # Regulatory Domain management
        self._set_regulatory_domain('AP')

        # Configure the equipment
        self._ns.set_wifi_config(self._ssid,
                                 self._hidden,
                                 self._standard,
                                 self._security,
                                 self._passphrase,
                                 self._channel,
                                 self._dtim,
                                 self._beacon,
                                 self._wmm,
                                 self._bandwidth,
                                 self._wifi_mimo,
                                 self._radiusip,
                                 self._radiusport,
                                 self._radiussecret)

        if self._auto_connect:
            self._ns.disable_wireless()
        else:
            self._ns.enable_wireless()

        # Turn Wifi interface ON
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)

        # Regulatory Domain management
        self._set_regulatory_domain('DUT')
        time.sleep(self._wait_btwn_cmd)

        # Close the connection to AP
        self._ns.release()

        # Configure the DUT
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Put display on. If not done, some regression (wifi broken after power off couldn't be seen)
        self._phone_system_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        self._networking_api.request_wifi_scan()
        time.sleep(self._wait_btwn_cmd)

        if self._auto_connect:
            # Create real SSID
            self._networking_api.set_wificonfiguration(self._ssid,
                                                       self._passphrase,
                                                       self._security,
                                                       self._ip_setting,
                                                       self._ip_address,
                                                       self._netmask,
                                                       self._gateway,
                                                       self._dns1,
                                                       self._dns2)

        # Create dummy SSID1
        self._networking_api.set_wificonfiguration(self._ssid + "1",
                                                   self._passphrase,
                                                   self._security,
                                                   self._ip_setting,
                                                   self._ip_address,
                                                   self._netmask,
                                                   self._gateway,
                                                   self._dns1,
                                                   self._dns2)

        # Create dummy SSID2
        self._networking_api.set_wificonfiguration(self._ssid + "2",
                                                   self._passphrase,
                                                   self._security,
                                                   self._ip_setting,
                                                   self._ip_address,
                                                   self._netmask,
                                                   self._gateway,
                                                   self._dns1,
                                                   self._dns2)

        self._phone_system_api.display_off()

        # Package deactivation
        if self._disabled_packages_list is not None:
            self._disable_packages(self._disabled_packages_list)

        # get initial flight mode
        self._init_flight_mode = self._networking_api.get_flight_mode()

        # Set flight mode
        if self._use_flight_mode != self._init_flight_mode:
            self._networking_api.set_flight_mode(self._use_flight_mode)

        self._sleep_mode_api.init(self._sleep_mode)

        # Wait for the DUT to settle down
        self._logger.info("wait " + str(self._settle_time) +
                          " seconds for DUT to settle down")
        time.sleep(self._settle_time)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
# pylint: disable=E1101

        LabWifiBase.run_test(self)

        # It's time to sleep !
        self._phonesystem_api.sleep_mode("on")

        if self._auto_connect:
            # Enable Wifi autoconnect on our SSID
            self._networking_api.set_autoconnect_mode(self._ssid,
                                                      self._uecmd_types.AUTO_CONNECT_STATE.on)
            # first, we wait for self._duration without AP up
            self._unplug_usb(self._duration)
            # then we turn wifi on, DUT should connect automatically
            self._ns.init()
            self._ns.enable_wireless()
            self._ns.release()

        self._residency_api.clear()

        self._unplug_usb(self._duration)

        residency = self._residency_api.get_value("residency", self._sleep_mode_api.get_sleep_mode())

        if self._auto_connect:
            # check that wifi has been connected during s3
            connected_wifi_list = self._networking_api.list_connected_wifi()
            if self._ssid not in connected_wifi_list:
                msg = "Wifi NOT connected to network %s" % self._ssid
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        result = self._sleep_mode + " residency: %f" % residency
        if residency <= self._target_residency_rate:
            self._logger.error(result)
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        return Global.SUCCESS, result

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)
        time.sleep(self._wait_btwn_cmd)
        self._sleep_mode_api.clear()
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _unplug_usb(self, duration):
        """
        Disconnect USB during "duration" seconds

        :type duration: integer
        :param duration: duration for USB disconnected in seconds

        :return: None
        """
        if self._io_card is not None:
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
        self._logger.info("sleep " + str(duration) + " seconds")
        time.sleep(duration)
        if self._io_card is not None:
            self._io_card.usb_host_pc_connector(True)
            # wait x seconds
            time.sleep(self._device._usb_sleep_duration)  # pylint: disable=W0212
            # connect board
            self._device.connect_board()
            time.sleep(self._wait_btwn_cmd)
