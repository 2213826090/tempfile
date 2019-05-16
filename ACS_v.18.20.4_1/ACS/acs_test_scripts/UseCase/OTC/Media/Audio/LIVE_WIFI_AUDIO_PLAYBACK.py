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

:organization: INTEL OTC
:summary: This use case is used to playback an audio file in different formats.
Extension of LiveMumAudioPlayback in order to add streaming over Wifi
capabilities and download file from artifactory capability.
:since: 06/04/2014
:author: agdobrex
"""

import time
import os
from UtilitiesFWK.Utilities import Global, is_number
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UseCase.Multimedia.LIVE_MUM_AUDIO_PLAYBACK import LiveMumAudioPlayback
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveWifiAudioPlayback(LiveMumAudioPlayback):

    """
    Class Live Wifi Audio Playback.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        LiveMumAudioPlayback.__init__(self, tc_name, global_config)

        use_artifactory = str(self._tc_parameters.get_param_value("USE_ARTIFACTORY"))

        if use_artifactory == "True":
            self._use_artifactory = True
        elif use_artifactory == "False":
            self._use_artifactory = False
        else:
            error_msg = "Use artifactory parameter is not configured. Please use only True or False"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Initialize variable for data connection
        self._use_wifi_connection = False


#------------------------------------------------------------------------------
    def _connect_to_wifi(self):
        """
        Connects DUT to wifi only if self._use_wifi_connection is true
        """

        # Disable flight mode
        if self._networking_api.get_flight_mode() == 1:
            self._networking_api.set_flight_mode("off")

        # Get parameters dictionary for wifi router equipment
        if self._em.get_global_config().benchConfig.has_parameter("NO_SECURITY_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("NO_SECURITY_WIFI_ROUTER")
        elif self._em.get_global_config().benchConfig.has_parameter("WEP_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("WEP_WIFI_ROUTER")
        elif self._em.get_global_config().benchConfig.has_parameter("WPA2_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("WPA2_WIFI_ROUTER")
        elif self._em.get_global_config().benchConfig.has_parameter("WPA_WIFI_ROUTER"):
            bench_dic = self._em.get_global_config().benchConfig.get_parameters("WPA_WIFI_ROUTER")
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Not any wifi router found in Bench Config")
        # Get SSID for Wifi from benchconfig
        ssid = bench_dic.get_param_value("SSID")
        # Get passphrase for Wifi from benchconfig
        passphrase = bench_dic.get_param_value("passphrase")
        # Get security for Wifi from benchconfig
        wifi_security = bench_dic.get_param_value("WIFI_SECURITY")
        # Validate SSID parameter
        if not ssid:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "SSID for wifi router is not defined in Bench Config")
        # Validate passphrase parameter
        if not passphrase:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Passphrase for wifi router is not defined in Bench Config")
        # Validate security parameter
        if not wifi_security:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                      "Security for wifi router is not defined in Bench Config")
        # Append '' to SSID if it contains white spaces
        if " " in ssid:
            ssid = "'"+ssid+"'"
        # Append '' to passphrase if it contains white spaces
        if " " in passphrase:
            passphrase = "'"+passphrase+"'"
        # Open Wifi on the DUT and remove current configuration
        #wifi_network = Networking(device=self._device)
        self._networking_api.set_wifi_power("on")
        self._networking_api.wifi_remove_config('all')
        # Set the configuration of Wifi connection on the DUT
        self._networking_api.set_wificonfiguration(ssid, passphrase, wifi_security)
        # Connect DUT to the Wifi
        self._networking_api.wifi_connect(ssid)

        return Global.SUCCESS, "No errors"


#------------------------------------------------------------------------------
    def _retrieve_file_from_artifactory(self):
        """
            Retrieves a file from the artifactory, if the USE_ARTIFACTORY parameter is set to true.
            Also updates self._audio_filename to use the name of the file downloaded from artifactory.
        """
        artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")
        local_artifact = artifact_manager.get_artifact(artifact_name=self._audio_filename, transfer_timeout=10)
        self._device.push(local_artifact, self._device.multimedia_path + self._audio_filename)
        self._audio_filename = os.path.basename(local_artifact)

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        # Check value of FILENAME parameter
        if self._audio_filename == "" or is_number(self._audio_filename):
            error_msg = "The parameter FILENAME must a string containing the local audio file or an url !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check if needed to enable data for audio streaming
        if self._audio_filename.startswith("http://"):
            self._use_wifi_connection = True
        elif self._audio_filename.startswith("https://"):
            error_msg = "Please use a non secured http format to your urls."
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # Check value of VOLUME parameter
        if self._multimedia_volume < 0 or self._multimedia_volume > 100:
            error_msg = "The parameter VOLUME must be an integer between 0 and 100 !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        # FILENAME is an url, enable wifi
        if self._use_wifi_connection:
            self._connect_to_wifi()
            # also set the use_artifactory parameter to false.
            self._use_artifactory=False

        # If the artifactory parameter is set, then get the file form artifactory
        # and set self._audio_filename to the correct file pushed on the device
        if self._use_artifactory:
            self._retrieve_file_from_artifactory()

        # Get audio file duration
        audio_file_duration = self._multimedia_api.get_media_file_duration(self._audio_filename)

        # Check value of DURATION parameter
        if self._audio_duration == "":
            self.get_logger().warning("The parameter DURATION is empty! The whole audio file will be played.")
            self._audio_duration = audio_file_duration
            # Add a delay to make sure the use case play whole the file
            self._audio_duration += 5

        elif self._audio_duration <= 0:
            error_msg = "The parameter DURATION must be an integer strictly upper than 0 !"
            self.get_logger().error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        else:
            if self._audio_duration > audio_file_duration:
                self.get_logger().warning("The parameter DURATION is upper than audio file duration ! "
                                          "The whole audio file will be played.")
                self._audio_duration = audio_file_duration
                # Add a delay to make sure the use case play whole the file
                self._audio_duration += 5

        # Disable device lock screen
        self._phonesystem_api.disable_lockscreen(True)
        # Set phone to keep the device display on and wake it up
        self._phonesystem_api.display_on()
        self._phonesystem_api.wake_screen()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        if self._phonesystem_api.get_screen_status():
            # Set phone screen off
            self._phonesystem_api.display_off()

        if self._use_wifi_connection:
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.wifi_disconnect_all()
            self._networking_api.wifi_remove_config('all')
            self._networking_api.set_wifi_power("off")

        return Global.SUCCESS, "No errors"
