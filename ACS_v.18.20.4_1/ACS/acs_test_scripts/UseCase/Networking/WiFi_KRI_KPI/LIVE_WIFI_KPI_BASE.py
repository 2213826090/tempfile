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
@summary: This file implements the LIVE WIFI KPI Base UC
@since: 10/04/2014
@author: jfranchx
"""

import time
import os
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveWifiKPIBase(UseCaseBase):
    """
    Live WiFi KPI Base Test class.
    """

    # Constants
    SCREEN_TIMEOUT = 1800
    BATTERY_CAPACITY = 90.0
    RSSI_MAX_VALUE = -40.0
    RSSI_MIN_VALUE = -45.0

    def __init__(self, tc_name, global_config):
        """
        Generic initialization for KPI TCs
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self.em_api = self._device.get_uecmd("EnergyManagement")
        self._location_api = self._device.get_uecmd("Location")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._batt_capacity = -1
        self._gps_state = None
        self._rssi_value = None

        self._country_regrev = self._tc_parameters.get_param_value("COUNTRY_REGREV")
        # Get the regulatory domain to use for the test
        self._user_reg_domain = self._tc_parameters.get_param_value("REGULATORY_DOMAIN")

    def set_up(self):
        """
        Generic set_up for KPI TCs. Should be executed after LiveWifiBase/LabWifiBase.
        """
        UseCaseBase.set_up(self)

        # Force regulatory domain and configure NVRAM file
        self._networking_api.set_regulatorydomain(self._user_reg_domain, self._dut_wlan_iface)
        self._configure_nvram_file()

        # Configure screen
        self._phone_system_api.set_phone_screen_lock_on(1)
        self._phone_system_api.set_phone_lock(0)
        self._phone_system_api.set_screen_timeout(self.SCREEN_TIMEOUT)

        # Configure "Stay awake" developper option
        self._phone_system_api.set_stay_on_while_plugged_in(True)

        # GoTo the Home menu
        self._phone_system_api._exec("adb shell input keyevent 3")

        # Deactivate modem, Bluetooth, WiFi, NFC, GPS, Network notifications, Mobile Data
        self._networking_api.deactivate_pdp_context()
        self._networking_api.set_network_notification('off')
        self._networking_api.set_flight_mode("on")
        self._gps_state = self._location_api.get_gps_power_status()
        self._location_api.set_gps_power("off")

        # WiFi is off, enable WiFi and check reconnection to the AP
        self._networking_api.set_wifi_power(1)
        self._networking_api.check_connection_state(self._ssid)

        # Check battery capacity - over 90% required
        result = self.em_api.get_msic_registers()
        self._batt_capacity = result["BATTERY"]["CAPACITY"][0]
        if self._batt_capacity < self.BATTERY_CAPACITY:
            msg = "Too low battery to run the test : %s%% - over 90%% required" % self._batt_capacity
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        Generic teardown for KPI TCs
        """
        UseCaseBase.tear_down(self)

        # Reconfigure default device state
        self._networking_api.set_flight_mode("off")
        if self._gps_state is not None:
            self._location_api.set_gps_power(self._gps_state)

        # Set default screen state
        self._phone_system_api.set_phone_lock(1)
        self._phone_system_api.set_phone_screen_lock_on(0)

        return Global.SUCCESS, "No errors"

# ----------------------------------------------------------------------------------------------------------------------
    def _configure_nvram_file(self):
        """
        Configure the NVRAM file to force the country code and regrev
        """
        # Get nvram file used
        used_nvram_file = None
        dmesg_result = self._phone_system_api._exec("adb shell \"dmesg | grep -i nvram\"")
        dmesg_result_list = dmesg_result.splitlines()
        for line in dmesg_result_list:
            # The line is constructed like this "xxx xxx xxx nvram path=/the/path/of/used_nvram_file"
            # We search "nvram path", get the last string "path=xxxx" of the line, and keep only the string of the path
            if line.find("nvram path") != -1:
                used_nvram_file = (str(line).split())[-1].split("=")[-1]
                break

        if used_nvram_file is not None and not self._phone_system_api.check_file_exist_from_shell(used_nvram_file):
            msg = "NVRAM File %s doesn't exist !" % used_nvram_file
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)
        self._logger.debug("Found nvram file : %s" % used_nvram_file)

        nvram_file = self._phone_system_api.read_file(used_nvram_file)
        # Config file modifications
        nvram_file = nvram_file.splitlines()
        output_line = ""
        output_file = self._device.get_ftpdir_path() + os.path.basename(used_nvram_file)

        self._phone_system_api._exec("adb remount")
        # Generate file with new parameters
        self._phone_system_api.write_file(output_file,output_line)
        for line in nvram_file:
            if line.lower().find("ccode=") != -1:
                output_line = "ccode=%s" % self._user_reg_domain
            elif line.lower().find("regrev=") != -1:
                output_line = "regrev=%s" % self._country_regrev
            else:
                output_line = line
            self._phone_system_api.write_file(output_file,output_line,"add")
        # Rewrite configuration file
        self._phone_system_api._exec("adb shell chmod 666 " + output_file)

        # Reboot WiFi to apply changes
        self._networking_api.set_wifi_power(0)
        time.sleep(0.5)
        self._networking_api.set_wifi_power(1)

        # Check Country code value
        if self._networking_api.get_regulatorydomain() != self._user_reg_domain:
            msg = "Country code : %s - ERROR, %s expected ! Can't configure country code" \
                  % (self._networking_api.get_regulatorydomain(),self._user_reg_domain)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

