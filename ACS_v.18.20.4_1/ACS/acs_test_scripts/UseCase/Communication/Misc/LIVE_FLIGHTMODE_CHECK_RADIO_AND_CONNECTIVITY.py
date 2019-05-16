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

:organization: INTEL MCG PSI
:summary: This file implements different radio types activation while
    in Flight Mode.
:since: 28/05/2013
:author: asebbanx
"""
# Module name follows ACS conventions
# pylint: disable=C0103
import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.DeviceException import DeviceException


class LiveFlightModeCheckRadioAndConnectivity(UseCaseBase):

    """
    Flight Mode effect on different radio types.
    We test:
    - that the Flight Mode deactivates all radio types (we do not check the
        modem because it is done in some other Use Cases)
    - that any radio type (except modem) can then be activated while
        in Flight Mode
    """

    STATE_ON = 1
    """
    Static attribute that represents the status I{ON}.
    """

    STATE_OFF = 0
    """
    Static attribute that represents the status I{OFF}.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCaseBase initializer
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize required parameters
        self._initial_flight_mode_state = None

        # Read parameters and convert them to their final type
        self._check_bt = \
            str(self._tc_parameters.get_param_value("CHECK_BT"))
        self._check_bt = str_to_bool(self._check_bt)
        self._check_nfc = \
            str(self._tc_parameters.get_param_value("CHECK_NFC"))
        self._check_nfc = str_to_bool(self._check_nfc)
        self._check_wlan = \
            str(self._tc_parameters.get_param_value("CHECK_WLAN"))
        self._check_wlan = str_to_bool(self._check_wlan)
        self._check_gps = \
            str(self._tc_parameters.get_param_value("CHECK_GPS"))
        self._check_gps = str_to_bool(self._check_gps)

        # Arbitrary value for timeout in seconds
        self._radio_activation_timeout = 40

        # Instantiate UE Command categories
        self._networking_api = self._device.get_uecmd("Networking")
        self._connectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._location_api = self._device.get_uecmd("Location")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Test setup
        """

        # Call the inherited method
        UseCaseBase.set_up(self)

        # Recording initial state before starting the test
        self._initial_flight_mode_state = \
            self._networking_api.get_flight_mode()

        # Toogle flight mode if it was initially activated
        if self._initial_flight_mode_state:
            self._logger.info("Flight Mode was active, deactivating it now.")
            self._networking_api.set_flight_mode(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
            time.sleep(self._radio_activation_timeout)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call the inherited method
        UseCaseBase.run_test(self)

        # Enable requested radio types separatedly
        self._logger.info("First we check that the different radio types")
        self._logger.info("can be activated.")
        if self._check_bt:
            # Activate bluetooth
            self._logger.debug("Checking BT activation.")
            self._connectivity_api.set_bt_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_ON)
            self._logger.info("Sleeping %d seconds."
                              % self._radio_activation_timeout)
            time.sleep(self._radio_activation_timeout)
            # Check bluetooth activation
            self.__check_bt_activation()
        if self._check_nfc:
            # Activate NFC
            self._logger.debug("Checking NFC activation.")
            self._connectivity_api.nfc_enable()
            time.sleep(self._radio_activation_timeout)
            # Check NFC activation
            self.__check_nfc_activation()
        if self._check_wlan:
            # Activate WLAN
            self._logger.debug("Checking WLAN activation.")
            self._networking_api.set_wifi_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_ON)
            time.sleep(self._radio_activation_timeout)
            # Check WLAN activation
            self.__check_wlan_activation()
        if self._check_gps:
            # Activate GPS
            self._logger.debug("Checking GPS activation.")
            self._location_api.set_gps_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_ON)
            time.sleep(self._radio_activation_timeout)
            # CheckGPS activation
            self.__check_gps_activation()

        # Enable Flight Mode
        self._logger.info("Now enabling Flight Mode.")
        self._networking_api.set_flight_mode(
            LiveFlightModeCheckRadioAndConnectivity.STATE_ON)
        time.sleep(self._radio_activation_timeout)

        # Check that BT/WLAN/NFC is deactivated while GPS has is activated
        if self._check_bt:
            self._logger.info("Check that BT has been deactivated.")
            self.__check_bt_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
        if self._check_nfc:
            self._logger.info("Check that NFC has been deactivated.")
            self.__check_nfc_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
        if self._check_wlan:
            self._logger.info("Check that WLAN has been deactivated.")
            self.__check_wlan_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
        if self._check_gps:
            self._logger.info("Check that GPS has not been deactivated.")
            self.__check_gps_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_ON)

        # Enable BT
        if self._check_bt:
            self._logger.info("Now check that BT can be activated.")
            # Turn BT on
            self._connectivity_api.set_bt_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_ON)
            self._logger.info("Sleeping %d seconds."
                              % self._radio_activation_timeout)
            time.sleep(self._radio_activation_timeout)

            # Check BT activation
            self.__check_bt_activation()

            self._logger.info("Disable BT.")
            # Disable BT
            self._connectivity_api.set_bt_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
            time.sleep(self._radio_activation_timeout)

            # Check BT deactivation
            self.__check_bt_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)

        # Enable NFC if needed
        if self._check_nfc:
            self._logger.info("Now check that NFC can be activated.")
            # Turn NFC on
            self._connectivity_api.nfc_enable()
            time.sleep(self._radio_activation_timeout)

            # Check NFC activation
            self.__check_nfc_activation()

            self._logger.info("Disable NFC.")
            # Disable NFC
            self._connectivity_api.nfc_disable()
            time.sleep(self._radio_activation_timeout)

            # Check NFC deactivation
            self.__check_nfc_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)

        # Enable WLAN if needed
        if self._check_wlan:
            self._logger.info("Now check that WLAN can be activated.")
            # Turn wifi power on
            self._networking_api.set_wifi_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_ON)
            time.sleep(self._radio_activation_timeout)

            # Check WLAN activation
            self.__check_wlan_activation()

            self._logger.info("Disable WLAN.")
            # Disable WLAN
            self._networking_api.set_wifi_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
            time.sleep(self._radio_activation_timeout)

            # Check WLAN deactivation
            self.__check_wlan_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)

        if self._check_gps:
            self._logger.info("Now check that GPS can be deactivated.")
            # Disable GPS
            self._location_api.set_gps_power(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)
            time.sleep(self._radio_activation_timeout)

            # Check GPS deactivation
            self.__check_gps_activation(
                LiveFlightModeCheckRadioAndConnectivity.STATE_OFF)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Tear down function
        """

        # Call the inherited method
        UseCaseBase.tear_down(self)

        # Set flight mode back to initial state
        # if flight mode has been correctly retrieved in the beginning
        if self._initial_flight_mode_state not in ("", None):
            self._logger.info("Restoring Flight Mode to its initial state.")
            self._networking_api.set_flight_mode(
                self._initial_flight_mode_state)

        return Global.SUCCESS, "No errors"

    def __check_bt_activation(self, expected_state=STATE_ON):
        """
        Checks that the I{Bluetooth} is active
        and raises an exception otherwise.

        :type expected_state: int
        :param expected_state: [optional] the expected state (1:ON / 0:OFF)
            Default value is 1 (ON).

        :raise DeviceException: if the I{Bluetooth}
            is not active.
        """
        # Store the expected state as str
        expected_state_str = self.__get_state_str(expected_state)
        # Retrieve the BT power status and convert it to str
        bt_power_status = self._connectivity_api.get_bt_power_status()
        bt_states_on = (
            "STATE_ON",
            "STATE_TURNING_ON",
            "STATE_CONNECTED",
            "STATE_CONNECTING")
        if str(bt_power_status) in bt_states_on:
            bt_power_status = "ON"
        else:
            bt_power_status = "OFF"
        self._logger.info("Got BT power status: %s" % str(bt_power_status))
        # Compare the actual state with the expected one
        if bt_power_status != expected_state_str:
            # If the retrieved BT status does not match the expected one
            # build an error message.
            message = "Current BT state '%s' does not match the " \
                "expected state '%s'." % (
                    bt_power_status,
                    expected_state_str)
            # Raise an exception
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, message)
        else:
            # Otherwise simply log a message indicating that
            # everything went OK
            self._logger.debug("BT activation status is the expected one.")

    def __check_nfc_activation(self, expected_state=STATE_ON):
        """
        Checks that the I{NFC} is active
        and raises an exception otherwise.

        :type expected_state: int
        :param expected_state: [optional] the expected state (1:ON / 0:OFF)
            Default value is 1 (ON).

        :raise DeviceException: if the I{NFC}
            is not active.
        """
        # Retrieve the current NFC status
        nfc_status = self._connectivity_api.get_nfc_status()
        self._logger.info("Got NFC status: %s" % str(nfc_status))
        if nfc_status in ("ON", "TURNING_ON"):
            # To simplify, we consider that TURNING_ON
            # in equivalent to ON
            nfc_status = LiveFlightModeCheckRadioAndConnectivity.STATE_ON
        else:
            # We have to convert the retrieved NFC status to
            # an equivalent to OFF
            nfc_status = LiveFlightModeCheckRadioAndConnectivity.STATE_OFF
        # Compare the actual state with the expected one
        if nfc_status != expected_state:
            # If the retrieved NFC status does not match the expected one
            # build an error message.
            nfc_status_str = self.__get_state_str(nfc_status)
            expected_state_str = self.__get_state_str(expected_state)
            message = "Current NFC state '%s' does not match the " \
                "expected state '%s'." % (nfc_status_str, expected_state_str)
            # Raise an exception
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, message)

        else:
            # Otherwise simply log a message indicating that
            # everything went OK
            self._logger.debug("NFC activation status is the expected one.")

    def __check_wlan_activation(self, expected_state=STATE_ON):
        """
        Checks that the I{WLAN} is active
        and raises an exception otherwise.

        :type expected_state: int
        :param expected_state: [optional] the expected state (1:ON / 0:OFF)
            Default value is 1 (ON).

        :raise DeviceException: if the I{WLAN}
            is not active.
        """
        # Retrieve the current WLAN power status
        wlan_power_state = self._networking_api.get_wifi_power_status()
        # Compare the actual state with the expected one
        if wlan_power_state != expected_state:
            # If the retrieved power status does not match the expected one
            # build an error message.
            power_state_str = self.__get_state_str(wlan_power_state)
            expected_state_str = self.__get_state_str(expected_state)
            message = "Current WLAN state '%s' does not match the " \
                "expected state '%s'." % (power_state_str, expected_state_str)
            # Raise an exception
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, message)

        else:
            # Otherwise simply log a message indicating that
            # everything went OK
            self._logger.debug("WLAN power status is the expected one.")

    def __check_gps_activation(self, expected_state=STATE_ON):
        """
        Checks that the I{GPS} is active
        and raises an exception otherwise.

        :type expected_state: int
        :param expected_state: [optional] the expected state (1:ON / 0:OFF)
            Default value is 1 (ON).

        :raise DeviceException: if the I{GPS}
            is not active.
        """
        # Retrieve the current GPS status
        gps_power_status = self._location_api.get_gps_power_status()
        # Compare the actual state with the expected one
        if gps_power_status != expected_state:
            # If the retrieved GPS status does not match the expected one
            # build an error message.
            message = "Current GPS state '%s' does not match the " \
                "expected state '%s'." % (
                    gps_power_status,
                    expected_state)
            # Raise an exception
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, message)
        else:
            # Otherwise simply log a message indicating that
            # everything went OK
            self._logger.debug("GPS activation status is the expected one.")

    def __get_state_str(self, state):
        """
        Returns the str representation of the given binary state
        (ON/OFF).

        :type state: int
        :param state: the expected state (1:ON / 0:OFF)

        :rtype: str
        :return: the state as str
        """
        # Initialize the return value
        state_str = "OFF"
        # Check whether the state is ON or not
        if state == LiveFlightModeCheckRadioAndConnectivity.STATE_ON:
            # Update the return value accordingly
            state_str = "ON"
        # Return the computed value
        return state_str
