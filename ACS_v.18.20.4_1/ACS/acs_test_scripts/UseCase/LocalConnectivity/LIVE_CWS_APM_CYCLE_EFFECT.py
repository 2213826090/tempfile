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
:summary: This file implements the LIVE CWS AIRPLANE MODE CYCLE EFFECT
:author: apairex
:since:12/07/2013
"""
import time

from UtilitiesFWK.Utilities import Global, str_to_bool_ex
from acs_test_scripts.Device.UECmd.UECmdTypes import BT_STATE
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser


class LiveCwsApmCycleEffect(UseCaseBase):

    """
    Live BT APM Cycle effect tests.
    Description:
      Initialize CWS feature states to what specified in the TC XML file
      Enable APM and check all features are OFF
      Disable APM and check features planed to switch back ON, switches back ON
                        and features planed to remains OFF, remains OFF
    """
    _WIFI_TETHERING_SSID = "LIVE_CWS_APM_CYCLE_EFFECT"
    _WIFI_TETHERING_SECURITY = "WPA2-PSK"
    _WIFI_TETHERING_PASSPHRASE = "1234567890123"

    _FEATURE_NAME = 'feature_name'
    _INIT_VALUE = 'init_value'
    _SET_ON = 'set_feature_on'
    _SET_ON_PARAM = 'set_feature_on_param'
    _SET_OFF = 'set_feature_off'
    _SET_OFF_PARAM = 'set_feature_off_params'
    _GET_STATE = 'get_feature_state'
    _EXPECTED_ON = 'expected_feature_state_on'
    _EXPECTED_OFF = 'expected_feature_state_off'

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get UECmdLayer
        self._bt_api = self._device.get_uecmd("LocalConnectivity")
        self._networking_api = self._device.get_uecmd("Networking")

        # Get TestCase parameters
        self._wifi_init_mode = self._tc_parameters.get_param_value("WIFI_INIT_MODE")
        self._bt_init_mode = self._tc_parameters.get_param_value("BT_INIT_MODE")
        self._nfc_init_mode = self._tc_parameters.get_param_value("NFC_INIT_MODE")
        self._nfc_beam_init_mode = self._tc_parameters.get_param_value("NFC_BEAM_INIT_MODE")
        self._hotspot_wifi_init_mode = self._tc_parameters.get_param_value("HOTSPOT_WIFI_INIT_MODE")
        self._hotspot_bt_init_mode = self._tc_parameters.get_param_value("HOTSPOT_BT_INIT_MODE")

        #  Format TC parameter in order to have defined boolean or None values
        self._wifi_init_mode = str_to_bool_ex(self._wifi_init_mode)
        self._bt_init_mode = str_to_bool_ex(self._bt_init_mode)
        self._nfc_init_mode = str_to_bool_ex(self._nfc_init_mode)
        self._nfc_beam_init_mode = str_to_bool_ex(self._nfc_beam_init_mode)
        self._hotspot_wifi_init_mode = str_to_bool_ex(self._hotspot_wifi_init_mode)
        self._hotspot_bt_init_mode = str_to_bool_ex(self._hotspot_bt_init_mode)

        # Database of all functions to control and check features states
        self._DB = [{self._FEATURE_NAME: "WIFI",
                     self._INIT_VALUE: self._wifi_init_mode,
                     self._SET_ON: self._networking_api.set_wifi_power,
                     self._SET_ON_PARAM: ['1'],
                     self._SET_OFF: self._networking_api.set_wifi_power,
                     self._SET_OFF_PARAM: ['0'],
                     self._GET_STATE: self._networking_api.get_wifi_power_status,
                     self._EXPECTED_ON: 1,
                     self._EXPECTED_OFF: 0},
                    {self._FEATURE_NAME: "BT",
                     self._INIT_VALUE: self._bt_init_mode,
                     self._SET_ON: self._bt_api.set_bt_power,
                     self._SET_ON_PARAM: ['1'],
                     self._SET_OFF: self._bt_api.set_bt_power,
                     self._SET_OFF_PARAM: ['0'],
                     self._GET_STATE: self._bt_api.get_bt_power_status,
                     self._EXPECTED_ON: str(BT_STATE.STATE_ON),  # pylint: disable=E1101
                     self._EXPECTED_OFF: str(BT_STATE.STATE_OFF)},  # pylint: disable=E1101
                    {self._FEATURE_NAME: "NFC",
                     self._INIT_VALUE: self._nfc_init_mode,
                     self._SET_ON: self._bt_api.nfc_enable,
                     self._SET_ON_PARAM: [],
                     self._SET_OFF: self._bt_api.nfc_disable,
                     self._SET_OFF_PARAM: [],
                     self._GET_STATE: self._bt_api.get_nfc_status,
                     self._EXPECTED_ON: "ON",
                     self._EXPECTED_OFF: "OFF"},
                    {self._FEATURE_NAME: "NFC_BEAM",
                     self._INIT_VALUE: self._nfc_beam_init_mode,
                     self._SET_ON: self._bt_api.enable_nfc_beam,
                     self._SET_ON_PARAM: [],
                     self._SET_OFF: self._bt_api.disable_nfc_beam,
                     self._SET_OFF_PARAM: [],
                     self._GET_STATE: self._bt_api.get_nfc_beam_status,
                     self._EXPECTED_ON: True,
                     self._EXPECTED_OFF: False},
                    {self._FEATURE_NAME: "HOTSPOT_WIFI",
                     self._INIT_VALUE: self._hotspot_wifi_init_mode,
                     self._SET_ON: self._networking_api.set_wifi_hotspot,
                     self._SET_ON_PARAM: ["on", self._WIFI_TETHERING_SSID,
                                          self._WIFI_TETHERING_SECURITY,
                                          self._WIFI_TETHERING_PASSPHRASE],
                     self._SET_OFF: self._networking_api.set_wifi_hotspot,
                     self._SET_OFF_PARAM: ['off'],
                     self._GET_STATE: self._networking_api.get_wifi_hotspot_status,
                     self._EXPECTED_ON: 1,
                     self._EXPECTED_OFF: 0},
                    {self._FEATURE_NAME: "HOTSPOT_BT",
                     self._INIT_VALUE: self._hotspot_bt_init_mode,
                     self._SET_ON: self._bt_api.set_bt_tethering_power,
                     self._SET_ON_PARAM: ['1'],
                     self._SET_OFF: self._bt_api.set_bt_tethering_power,
                     self._SET_OFF_PARAM: ['0'],
                     self._GET_STATE: self._bt_api.get_bt_tethering_power,
                     self._EXPECTED_ON: True,
                     self._EXPECTED_OFF: False}
                    ]

        self._apm_cycle_effect_config = dict()
        self._features_on = list()

        # Instantiate the Secondary Report object
        self._secondary_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())

        self._initial_flight_mode_state = 0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Save the initial flight mode state
        self._initial_flight_mode_state = self._networking_api.get_flight_mode()

        # set flight mode
        self._logger.info("Disable flight mode")
        self._networking_api.set_flight_mode(0)
        if self._networking_api.get_flight_mode():
            msg = "Unable to disable flight mode"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Parse CWS_AirPlaneMode_Cycle_Effect.xml config file
        self._apm_cycle_effect_config = \
            ConfigsParser("CWS_AirPlaneMode_Cycle_Effect").parse_cws_airplanemode_cycle_effect()

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Initialize the CWS feature states
        self._init_cws_features()

        # Enable flight mode
        self._networking_api.set_flight_mode(1)
        time.sleep(self._wait_btwn_cmd)

        # Check all tested features are turned OFF
        self._check_all_off()

        # Disable flight mode
        self._networking_api.set_flight_mode(0)
        time.sleep(self._wait_btwn_cmd)

        # Check all tested feature state after flight mode back OFF
        self._check_all_features_after_apm_cycle()

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Restore the flight mode status as when entering in this UC
        self._networking_api.set_flight_mode(self._initial_flight_mode_state)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _init_cws_features(self):
        """
        Initialize the state of each CWS technos specified in the XML TC parameter file.
        Ignore others.
        """
        # For each feature
        for feature_params in self._DB:
            # Test if the feature is handled by the TC parameter file
            if feature_params[self._INIT_VALUE] is not None:
                if not feature_params[self._INIT_VALUE]:
                    # The init state is OFF
                    feature_params[self._SET_OFF](*feature_params[self._SET_OFF_PARAM])
                elif self._check_feature_incompatibilities(feature_params[self._FEATURE_NAME]):
                    # The init state is ON
                    feature_params[self._SET_ON](*feature_params[self._SET_ON_PARAM])
                else:
                    # In case of incompatibility, the initial state have to be forced to "feature ignored"
                    feature_params[self._INIT_VALUE] = None
                    self._secondary_report.add_result(feature_params[self._FEATURE_NAME],
                                                      self._secondary_report.verdict.BLOCKED,
                                                      "Cannot test this feature due to incompatibilities",
                                                      self.get_name(), self.tc_order)

                time.sleep(self._wait_btwn_cmd)

    def _check_feature_incompatibilities(self, feature_name):
        """
        Checks feature incompatibilities.
        Feature incompatibilities are stored in the CWS_AirPlaneMode_Cycle_Effect.xml file

        :type feature_name: String
        :param feature_name: Name of the feature to check incompatibilities

        :rtype: boolean
        :return: True if the
        """
        incompatibilies = self._apm_cycle_effect_config[feature_name][ConfigsParser.APM_INCOMPATIBILITIES]

        for inc in incompatibilies:
            if inc in self._features_on:
                self._logger.warning("%s is incompatible with %s. It will be deactivated" % (feature_name, inc))
                return False

        self._features_on.append(feature_name)
        return True

    def _check_all_off(self):
        """
        Check all tested features are off
        This function should be used after a flight mode ON.
        This function raised an Exception in case of all feature are not OFF
        """
        global_error_msg = ""

        # For each feature
        for feature_params in self._DB:
            # Test if the feature is handled by the TC parameter file
            if feature_params[self._INIT_VALUE] is not None:

                # Test if the feature is well OFF
                if feature_params[self._GET_STATE]() != feature_params[self._EXPECTED_OFF]:
                    global_error_msg = "%s, " % feature_params[self._FEATURE_NAME]
                    msg = "Fail to turn OFF " + feature_params[self._FEATURE_NAME]
                    sub_verdict = self._secondary_report.verdict.FAIL
                else:
                    msg = "Success to turn OFF " + feature_params[self._FEATURE_NAME]
                    sub_verdict = self._secondary_report.verdict.PASS

                # Record the sub verdict
                self._secondary_report.add_result(feature_params[self._FEATURE_NAME] + "_turn_OFF", sub_verdict, msg,
                                                  self.get_name(), self.tc_order)
                time.sleep(self._wait_btwn_cmd)

        if global_error_msg:
            # Remove the last ", " characters
            global_error_msg = global_error_msg[:-2]
            global_error_msg = "Flight mode has been activated and %s is(are) still ON" % global_error_msg
            self._logger.error(global_error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, global_error_msg)

    def _check_all_features_after_apm_cycle(self):
        """
        Checks all tested features states after AirPlaneMode cycle.
        """
        error_msg = ""

        for feature_params in self._DB:
            # Test if the feature is handled by the TC parameter file
            if feature_params[self._INIT_VALUE] is not None:

                # Compute the expected state
                expected_state = feature_params[self._INIT_VALUE] and \
                    (self._apm_cycle_effect_config[feature_params[self._FEATURE_NAME]][ConfigsParser.APM_STATE_AFTER_APM_CYCLE] == "ON")
                if expected_state:
                    expected_state = "enabled"
                    expected_value = feature_params[self._EXPECTED_ON]
                else:
                    expected_state = "disabled"
                    expected_value = feature_params[self._EXPECTED_OFF]

                # Test if the feature is in the expected state
                if feature_params[self._GET_STATE]() != expected_value:
                    self._logger.debug("%s should be %s" % (feature_params[self._FEATURE_NAME], expected_state))
                    error_msg = "%s, " % feature_params[self._FEATURE_NAME]
                    msg = "WRONG state after APM cycle for " + feature_params[self._FEATURE_NAME]
                    sub_verdict = self._secondary_report.verdict.FAIL
                else:
                    msg = "Good state after APM cycle for %s (%s)" % (feature_params[self._FEATURE_NAME], expected_state)
                    sub_verdict = self._secondary_report.verdict.PASS

                # Record the sub verdict
                self._secondary_report.add_result(feature_params[self._FEATURE_NAME] + "_state_after_APM_cycle", sub_verdict, msg,
                                                  self.get_name(), self.tc_order)
                time.sleep(self._wait_btwn_cmd)

        if error_msg:
            error_msg = "%s is(are) in the WRONG state" % error_msg
            self._logger.error(error_msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)
