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
:summary: This file implements the Modem UEcmd for Android devices
:since: 19/04/2011
:author: dgonzalez
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Networking.Networking import Networking
from acs_test_scripts.Device.UECmd.Interface.Communication.IModem import IModem
from acs_test_scripts.Device.UECmd.UECmdTypes import BPLOG_LOC, NETWORK_TYPES
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

from UtilitiesFWK.Utilities import get_method_name, Global
import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class Modem(BaseV2, IModem):

    """
    :summary: Modem UEcommands for Android platform
    """

    """
    A C{dict} containing all possible integer representations
    of network preferences.

    :note: Values given by getPreferredNetworkType
    """
    NETWORK_EQUIVALENCES = {
        "GSM": "GPRS",
        "EDGE": "EGPRS",
        "WCDMA": "UMTS",
        "CUSTOM": "HSPAP"
    }

    @need('modem')
    def __init__(self, device):
        """
        Constructor
        """
        IModem.__init__(self, device)
        BaseV2.__init__(self, device)
        self._logger = device.get_logger()
        self._call_setup_timeout = device.get_call_setup_timeout()
        # Use device getter to ensure usage of inheritance of variables and methods
        self._phone_system = device.get_uecmd("PhoneSystem")
        self._networking = Networking(device)

        self.icategory = "intel.intents.category.MODEM"
        self.component = "com.intel.acs.agent/.Modem"
        self.__telephony_module = "acscmd.telephony.TelephonyModule"

        self._bplog_location = None

    def set_modem_power(self, mode):
        """
        Sets the modem power to off or on.

        :type mode: str or int
        :param mode: can be 'on', '1' or 1 to enable.
        'off', '0' or 0) to disable.

        :return: None
        """
        method = "setModemPower"
        mode_str = None
        cmd_arg = None
        if mode in ("on", "1", 1):
            mode_str = "ON"
            cmd_arg = "--ei mode 1"
        elif mode in ("off", "0", 0):
            mode_str = "OFF"
            cmd_arg = "--ei mode 0"
        else:
            self._logger.error("Parameter mode %s is not valid" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Parameter mode is not valid !")

        self._logger.info("Settings Modem Power to " + mode_str)
        self._internal_exec_v2(self.__telephony_module, method, cmd_arg, is_system=True)

    def get_modem_power_status(self):
        """
        Returns the modem power status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        method = "getModemPowerStatus"
        output = self._internal_exec_v2(self.__telephony_module, method, is_system=True)
        return int(output["modem_power_status"])

    def get_imsi(self, timeout):
        """
        Returns the IMSI value.
        :type timeout: int
        :param timeout: maximum time allowed (in seconds) in order to retrieve
        I{IMSI}
        :rtype: str
        :return: The IMSI number
        :raise DeviceException: If the timeout is reached.
        """
        self._check_usim_operator(timeout)
        # Retrieve IMSI
        function = "getIMSI"
        results = self._internal_exec_v2(self.__telephony_module, function, is_system=True)
        imsi = str(results["imsi"])
        self._logger.info("Retrieved IMSI is : " + str(imsi))
        return imsi

    def get_lac(self):
        """
        Returns the LAC value.

        :rtype: String List
        :return: The LAC number list
        """

        method = "getLac"
        raw_lacs = self._internal_exec_multiple_v2(self.__telephony_module, method, is_system=True)

        lacs_list = self._build_list_from_dict(raw_lacs, "lac")

        self._logger.info("MODEM: Retrieved LAC are : " + str(lacs_list))
        return lacs_list

    def register_to_network(self, name='default'):
        """
        Attempts to register to a network

        :type name: str
        :param name: name of the network , can be omitted for default network

        :return: None
        """
        method = "registerToNetwork"
        self._internal_exec_v2(self.__telephony_module, method, is_system=True)

    def deregister_all(self):
        """
        Deregister from the network

        .. warning:: Should not be implemented anymore

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                              "%s not implemented on Android" % get_method_name())

    def get_network_registration_status(self):
        """
        Returns the registration status value.

        This function uses "adb shell getprop" commands to get device properties:
        - when operator field (gsm.operator.numeric) is empty, device is unregistered
        - when SIM operator (gsm.sim.operator.numeric) is the same as camped operator
        (gsm.operator.numeric), device is registered
        - when SIM operator (gsm.sim.operator.numeric) is different from camped operator
        (gsm.operator.numeric), device is roaming (national or international)
        - in any other cases, the status will be unknown

        :rtype: str
        :return: the requested value into a list of 1 element(or an error message
        if an error occurred).
        Network registration state:
                "unregistered"  Not registered to any network
                "registered"    Registered to home network
                "roaming"       Roaming (national or international)
                "unknown"       Status is unknown
        """
        # default state: unknown
        reg_state = "unknown"

        # Check if device is unregistered
        gsm_operator = self._phone_system.get_single_sim_gsm_property_value("gsm.operator.numeric")

        if gsm_operator in ["", None]:
            reg_state = "unregistered"
        else:
            # The device is registered, let's check if it is roaming or not
            gsm_sim_operator = \
                self._phone_system.get_single_sim_gsm_property_value("gsm.sim.operator.numeric")
            if gsm_sim_operator in [None, ""]:
                reg_state = "unregistered"
            else:
                gsm_sim_operator = gsm_sim_operator[:5]

                if gsm_sim_operator == gsm_operator:
                    reg_state = "registered"
                else:
                    reg_state = "roaming"

        return reg_state

    def check_cdk_registration_bfor_timeout(self, timeout):
        """
        Check if the network registration status is registered before timeout.

        :type timout: int
        :param timeout: time allowed (in seconds) in order to reach the status I{registered}.

        :return: None (raise exception if an error occurred).
        """
        self.check_cdk_state_bfor_timeout("registered", timeout)

    def check_cdk_no_registration_bfor_timeout(self, timeout):
        """
        Check if the network registration status is not registered before timeout.

        :type timout: int
        :param timeout: time allowed (in seconds) in order to reach a status not equal to registered.

        :return: None (raise exception if an error occurred).
        """
        state_reached = False

        str_state = "registered"

        msg = "Checks DUT is not registered before %s seconds..." % str(timeout)
        self._logger.info(msg)
        start_time = time.time()
        try:
            self._phone_system.display_on()

            while (time.time() - start_time) <= timeout:
                reg_state = self.get_network_registration_status()
                if reg_state != str_state:
                    state_reached = True
                    break
        finally:
            self._phone_system.display_off()

        if state_reached:
            return_msg = "State " + reg_state + " has been reached !"
            self._logger.info(return_msg)
            return reg_state
        else:
            return_msg = "DUT is still registered after %s seconds..." % str(timeout)
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)

    def check_cdk_state_bfor_timeout(self, state, timeout):
        """
        Check the network registration state before timeout.

        :type state: str or list of several state
        :param state: The possible values are:
                "unregistered"  Not registered to any network
                "registered"    Registered to home network
                "searching"     Not registered, but searching
                "denied"        Registration has been denied
                "unknown"       Status is unknown
                "roaming"       Roaming

        :type timeout: int
        :param timeout: Time to wait attempting state

        :rtype: str
        :return: the state reached if state match or raise error
        """
        state_reached = False

        if not isinstance(state, list):
            state = [state]

        for element in state:
            if element not in ["unregistered", "registered", "searching",
                               "denied", "unknown", "roaming"]:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "unknown state:" + element)
        str_state = str(state).replace(",", " or")

        msg = "Checks %s state before %s seconds..." % \
            (str_state, str(timeout))
        self._logger.info(msg)
        start_time = time.time()
        try:
            self._phone_system.display_on()
            while (time.time() - start_time) <= timeout:
                reg_state = self.get_network_registration_status()
                if reg_state in state:
                    state_reached = True
                    break
                time.sleep(1)
        finally:
            self._phone_system.display_off()

        if state_reached:
            return_msg = "State " + reg_state + " has been reached !"
            self._logger.info(return_msg)
            return reg_state
        else:
            return_msg = "State " + str_state + \
                " has not been reached on time !"
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)

    def set_preferred_radio(self, radio_type):
        """
        Sets the preferred radio type.

        .. warning:: As we not have a status of the internal implementation on the board
                    We use a "script" which made the action pseudo-manually using the RadioInfo application
                    integrated on com.android.settings default application.

                    To avoid potential exception error from UI instability the implementation of this ue command has been commented.
                    The following info log should be noticed to the user : "set_preferred_radio is not implementable. It is only available at UI level !"

        :type radio_type: str
        :param radio_type: the radio type. Possible value are:
            - "any"
            - "umts"
            - "gsm"
            - "lte"

        :return: None
        """

        KEYCODE_DPAD_UP = "19"
        KEYCODE_DPAD_DOWN = "20"
        KEYCODE_DPAD_CENTER = "23"
        KEYCODE_BACK = "4"
        KEYCODE_PAGE_UP = "92"

        # Unlock the phone
        self._phone_system.set_phone_lock(0)

        self._logger.info("Setting preferred radio to " + radio_type)

        output = self._exec(
            "adb shell am start -n com.android.settings/.RadioInfo")
        if output.find("Error:") != -1:
            error_msg = output[output.find("Error: ") + len("Error: "):]
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_msg)

        if output.find("Warning: ") != -1:
            warning_msg = output[output.find("Warning: ") + len("Warning: "):]
            self._logger.warning(warning_msg)

        # Ensure we are at the bottom of the current view
        count = 1
        while count < 11:
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_UP, 2, True, False)
            count += 1

        self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN)

        # Enter the spinner menu
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER)

        # Ensure we are on the top of current spinner
        count = 1
        # while (count < 15):
        self._exec("adb shell input keyevent " + KEYCODE_PAGE_UP)
        self._exec("adb shell input keyevent " + KEYCODE_PAGE_UP)
        #    count += 1

        # default value
        position = 9

        # Then select the right entry according choice param
        if radio_type == "any":
            position = 1
        if radio_type == "gsm":
            position = 2
        if radio_type == "umts":
            position = 3
        if radio_type == "lte":
            position = 12
        if radio_type not in ("gsm", "umts", "lte", "any"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Wrong choice: " + radio_type + " should be gsm, umts, lte or any")

        # Go to the right entry
        count = 1
        while count < position:
            self._exec("adb shell input keyevent " + KEYCODE_DPAD_DOWN, 20, True, False)
            count += 1
        # Select the relevant entry
        self._exec("adb shell input keyevent " + KEYCODE_DPAD_CENTER, 20, True, False)
        # Exit the com.android.settings/.RadioInfo application
        self._exec("adb shell input keyevent " + KEYCODE_BACK, 20, True, False)

        # Re-allow phone locking
        self._phone_system.set_phone_lock(1)
        self._logger.warning("%s is not implementable at Application Framework level. It is only available at UI level !" % get_method_name())

    def get_emergency_numbers(self):
        """
        Get the list of emergency numbers inside the device

        :rtype: list
        :return: Emergency numbers
        """
        recup_emergency_list = self._exec("adb shell getprop ril.ecclist")
        return recup_emergency_list.split(",")

    def set_modem_online(self, mode):
        """
        Sets the modem to offline or online.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        self.set_modem_power(mode)

    def get_modem_online_status(self):
        """
        Returns the modem online status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        return self.get_modem_power_status()

    def _check_usim_operator(self, timeout=30):
        """
        Check and return USIM operator

        :type timeout: int
        :param timeout: timeout in seconds to wait for SIM card to be ready

        :rtype: int
        :return: GSM sim operator
        """
        start_time = time.time()
        msg = "Check USIM operator before %s seconds..." % str(timeout)
        self._logger.info(msg)

        gsm_sim_operator_is_digit = False
        while time.time() - start_time <= timeout:
            gsm_sim_operator = self._phone_system.get_single_sim_gsm_property_value("gsm.sim.operator.numeric")
            if str(gsm_sim_operator).isdigit():
                gsm_sim_operator_is_digit = True
                break
            time.sleep(1)

        if gsm_sim_operator_is_digit:
            usim_state = self._phone_system.get_single_sim_gsm_property_value("gsm.sim.state")
            if usim_state not in ["READY", "", None] and len(usim_state) < 2:
                return_msg = "DUT USIM state incompatible (%s)!" % usim_state
                self._logger.error(return_msg)
                raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)
            elif "READY" in usim_state:
                return gsm_sim_operator
            else:
                return gsm_sim_operator
        else:
            return_msg = "DUT could not access USIM operator on time !"
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)

    def get_sim_operator_info(self, timeout=30):
        """
        Returns the sim MNC and MCC.

        :type timeout: int
        :param timeout: timeout in seconds to wait for SIM card to be ready

        :rtype: dict
        :return: return a dictionary that contain the key (MNC,MCC)
        """

        gsm_sim_operator = self._check_usim_operator(timeout)
        sim_info = {
            "MCC": int(gsm_sim_operator[:3]),
            "MNC": int(gsm_sim_operator[3:5])
        }
        return sim_info

    def get_extended_sim_operator_info(self, timeout=30, nb_digits=2):
        """
        Returns the sim MNC and MCC as str and handles 3 digits MNC.

        :type timeout: int
        :param timeout: timeout in seconds to wait for SIM card to be ready

        :type nb_digits: int
        :param nb_digits: number of digits for MNC

        :rtype: dict
        :return: return a dictionary that contain the key (MNC,MCC)
        """

        gsm_sim_operator = self._check_usim_operator(timeout)
        sim_info = {
            "MCC": str(gsm_sim_operator[:3]),
            "MNC": str(gsm_sim_operator[3:(3 + nb_digits)])
        }
        return sim_info

    def get_cellular_operator_info(self):
        """
        Returns the cellular MNC and MCC.
        On a I{DSDS} phone the following properties are also returned: MCC_2, MNC_2
        for the second SIM.

        :rtype: dict
        :return: return a dictionary that contain the keys:
            - MNC
            - MCC
            - MNC_2 (on I{DSDS} phones)
            - MCC_2 (on I{DSDS} phones)
        """

        # Process information for default SIM (or SIM 1)
        gsm_cellular_operator = self._phone_system.get_single_sim_gsm_property_value("gsm.operator.numeric")

        if str(gsm_cellular_operator) == "":
            cellular_info = {
                "MCC": "",
                "MNC": ""
            }
        elif not str(gsm_cellular_operator).isdigit():
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "Cellular operator numeric format is not digit (%s)" % gsm_cellular_operator)
        else:
            cellular_info = {
                "MCC": int(gsm_cellular_operator[:3]),
                "MNC": int(gsm_cellular_operator[3:5])
            }
        # Process information for SIM 2 if any
        gsm_cellular_operator2 = self._phone_system.get_property_value("gsm.operator2.numeric")
        # Check whether the property is available or not
        if gsm_cellular_operator2 is not None:
            # Force str conversion
            gsm_cellular_operator2 = str(gsm_cellular_operator2)
            if gsm_cellular_operator2 == "":
                # In that case set empty values for MNC 2 / MCC 2
                cellular_info["MCC_2"] = ""
                cellular_info["MNC_2"] = ""
            # Check the property format
            elif gsm_cellular_operator2.isdigit():
                # Update MNC 2 / MCC 2 with appropriate values
                cellular_info["MCC_2"] = int(gsm_cellular_operator2[:3])
                cellular_info["MNC_2"] = int(gsm_cellular_operator2[3:5])
            # Otherwise the value is considered not valid
            else:
                raise DeviceException(DeviceException.INVALID_PARAMETER,
                                      "Cellular operator numeric format (SIM 2) is not digit (%s)"
                                      % gsm_cellular_operator2)

        return cellular_info

    def get_network_type(self):
        """
        Returns the Network type.

        :rtype: str
        :return: network type value can be :
            - UNKNOWN (GSM)
            - GPRS
            - EGPRS
            - UMTS
            - CDMA
            - EVDO_0
            - EVDO_A
            - 1xRTT
            - HSDPA
            - HSUPA
            - HSPA
            - IDEN
            - EVDO_B
            - LTE
            - EHRPD
            - HSPAP
        """
        function = "getNetworkType"
        results = self._internal_exec_v2(self.__telephony_module, function, is_system=True)
        # Retrieve the network type value
        network_type = str(results["network_type"])
        self._logger.debug("DUT RAT is %s" % network_type)

        # For camp over EGPRS, function getNetworkType() returns "EDGE"
        # so, returned network type must be EGPRS to be understood with
        # equipment network type
        if network_type == "EDGE":
            network_type = "EGPRS"

        if network_type == "UMTS":
            network_type = "WCDMA"

        return network_type

    def get_current_rat(self):
        """
        Returns the raw value of the current I{RAT} as returned
        by Android.

        This method is similar to C{get_network_type} but without
        the Agilent-related post-processing that makes the method
        unusable on a I{live} network.

        :rtype: str
        :return: network type value can be :
            - UNKNOWN (GSM)
            - GPRS
            - EGPRS
            - UMTS
            - CDMA
            - EVDO_0
            - EVDO_A
            - 1xRTT
            - HSDPA
            - HSUPA
            - HSPA
            - IDEN
            - EVDO_B
            - LTE
            - EHRPD
            - HSPAP
        """
        # Call the embedded method
        function = "getNetworkType"
        results = self._internal_exec_v2(self.__telephony_module, function, is_system=True)
        # Retrieve the network type value
        network_type = str(results["network_type"])
        self._logger.debug("DUT RAT is %s" % network_type)

        # Return the value
        return network_type

    def check_network_type_before_timeout(self,
                                          network_type,
                                          timeout=0):
        """
        Check whether or not the DUT retrieves the RAT provided by the network.
        :type network_type: str
        :param network_type: expected network type to check
        :type timeout: integer
        :param timeout: maximum authorized time for DUT registration
        :rtype: bool
        :return: True if DUT RAT match network_type, else return False.
        """
        start_time = time.time()
        dut_network_type = None
        ret_val = True

        self._logger.info("Check DUT RAT matches network RAT (%s) before %d seconds", network_type, timeout)

        # Check that equipment network type is different than "UNKNOWN" to do the RAT check
        if network_type != "UNKNOWN":
            try:
                self._phone_system.display_on()

                # Check that DUT is registered on the expected network type before timeout
                while ((time.time() - start_time) <= timeout and
                       (dut_network_type != network_type)):

                    # Read RAT value reported by the DUT
                    dut_network_type = self.get_network_type()
                    # If RAT is HSPA it is possible to have mismatch between NW and DUT
                    if (dut_network_type in ("HSDPA", "HSUPA", "HSPAP")) and (network_type == "HSPA"):
                        self._logger.info("DUT RAT matches network RAT (%s)" % network_type)
                        return ret_val
                    time.sleep(1)
            finally:
                self._phone_system.display_off()

            # Compare the service type seen by the DUT with the one of the Network simulator.
            if dut_network_type == network_type:
                # Registration success
                self._logger.info("DUT RAT matches network RAT (%s)" % dut_network_type)

            # For camp over GSM, function getNetworkType() returns "UNKNOWN"
            elif (dut_network_type == "UNKNOWN") and (network_type == "GSM"):
                self._logger.info("DUT RAT matches network RAT (%s)" % network_type)

            # For camp over WCDMA with Circuit Switched Data, function getNetworkType() returns "UNKNOWN"
            elif (dut_network_type == "UNKNOWN") and (network_type == "WCDMA"):
                self._logger.info("DUT RAT matches network RAT (%s)" % network_type)

            else:  # DUT RAT does not matches network RAT
                msg = "DUT RAT (%s) still does not match network RAT (%s) after %d seconds !" \
                    % (dut_network_type,
                       network_type,
                       timeout)
                self._logger.warning(msg)
                ret_val = False

        else:  # Equipment network type can not be retrieved
            msg = "Function to retrieve the network type is not present in the equipment driver. RAT Check is skipped."
            self._logger.warning(msg)
            ret_val = False

        return ret_val

    def get_imei(self):
        """
        Returns the IMEI value.
        :rtype: str
        :return: The IMEI number
        """
        function = "getIMEI"
        results = self._internal_exec_v2(self.__telephony_module, function, is_system=True)
        imei = str(results["imei"])
        self._logger.info("Retrieved IMEI is : " + str(imei))
        return imei

    def get_second_imei(self, sim):
        """
        Returns the IMEI value for second SIM.
        :rtype: str
        :return: The IMEI number for second SIM
        """
        function = "getIMEIbyslot"
        args = "--ei sim %d" % (sim)
        results = self._internal_exec_v2(self.__telephony_module, function, args, is_system=True)
        imei = str(results["imei"])
        self._logger.info("Retrieved second IMEI is : " + str(imei))
        return imei

    def get_imeisv(self):
        """
        Returns the IMEISV value.
        :rtype: str
        :return: The IMEISV number
        """
        function = "getIMEISV"
        results = self._internal_exec_v2(self.__telephony_module, function, is_system=True)
        imeisv = str(results["imeisv"])
        self._logger.info("Retrieved IMEISV is : " + str(imeisv))
        return imeisv

    def configure_modem_trace(self, hsi_speed="h", trace_level=2):
        """
        Configure the modem tracing

        :type hsi_speed: str
        :param hsi: HSI speed, can be:
                            - u: disable HSI
                            - d: default speed HSI, 78MHz
                            - h: high speed HSI, 156MH [default]

        :type trace_level: int
        :param trace_level: Level of trace, can be:
                            - 0: disable trace
                            - 1: first level trace (bb_sw)
                            - 2: second level trace (bb_sw & 3g_sw) [default]
                            - 3: third level trace (bb_sw & 3g_sw & digrf2)
                            - 4: fourth level trace (bb_sw & 3g_sw & digrfx2 & lte_l1_sw)
                            - 5: fifth level trace (bb_sw & 3g_sw & digrfx & lte_l1_sw & 3g_dsp & tdscdma_l1_sw & sig_mon)

        .. warning:: This UeCmd need a hardware reboot to be effective !

        :raise AcsConfigException.INVALID_PARAMETER:
                                        In case of invalid input parameter

        :raise AcsConfigException.INTERNAL_EXEC_ERROR:
                                        In case of command failure

        :return: None
        """
        self._logger.info("Configure modem trace (hsi speed %s, trace level %d)"
                          % (hsi_speed, trace_level))

        if hsi_speed not in ("u", "d", "h"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The given trace level is invalid (%s) !" % (str(trace_level)))

        if trace_level not in (0, 1, 2, 3, 4, 5):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The given trace level is invalid (%s) !" % (str(trace_level)))

        # Ensure adb is rooted
        self._exec('adb root', self._uecmd_default_timeout, False)

        # Configure modem trace
        adb = "adb shell "
        cmd = "configure_trace_modem -%s -t%s" % (hsi_speed, str(trace_level))
        output = self._exec(adb + cmd, self._uecmd_default_timeout, False)

        # Check command success
        if output.find(cmd + " SUCCESS.") != -1:
            self._logger.debug("Configuration of modem trace on level %s succeed" % trace_level)
        else:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Unable to configure modem trace (hsi speed: %s, trace level: %s) !"
                                  % (hsi_speed, str(trace_level)))

    # pylint: disable=E1101
    # Because pylint can not resolve enum, need to set here because param
    def activate_modem_trace(self, destination=BPLOG_LOC.EMMC,
                             file_size_option=2):
        """
        Activate the modem tracing

        :type destination: UeCmdTypes.BPLOG_LOC
        :param destination: The destination of the logs (emmc|sdcard)

        :type file_size_option: int
        :param file_size_option: The index representing the log file size
        e.g. on r4-dsds device 1 mean  100Mb of files (5 x 20Mb),
                and 2 mean 150Mb of files (6 x 25Mb)
        e.g. on redhookbay device 1 mean  100Mb of files (5 x 20Mb),
                and 2 mean 600Mb of files (3 x 200 Mb)

        .. note:: This last parameter mean different values for each platform.
        .. note:: As using AMTL application, this UeCmd will start mts service as
                persistent one.

        :raise AcsConfigException.INVALID_PARAMETER:
                                        In case of invalid input parameter

        :raise AcsConfigException.INTERNAL_EXEC_ERROR:
                                        In case of command failure

        :return: None
        """

        self._logger.info("Activate trace modem (loc: %s, file option %d)" % (str(destination), file_size_option))

        target_dest = None
        if destination not in BPLOG_LOC:
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR,
                                     "The given destination is invalid (%s) !" % (str(destination)))
        else:
            self._bplog_location = destination
            if destination == BPLOG_LOC.EMMC:
                target_dest = "e"
            elif destination == BPLOG_LOC.SDCARD:
                target_dest = "sd"

        if file_size_option not in (1, 2):
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR,
                                     "The given file size parameter is invalid (%s) !" % (str(file_size_option)))

        # Ensure adb is rooted
        self._exec('adb root', self._uecmd_default_timeout, False)

        # Activate modem trace
        adb = "adb shell "
        cmd = "activate_trace_modem -on"
        params = " -%s%d -p" % (target_dest, file_size_option)
        output = self._exec(adb + cmd + params, self._uecmd_default_timeout, False)

        # Check command success
        if cmd in output and "persistent enabled" in output:
            self._logger.info("Activation of modem trace on %s succeed" % (str(destination)))
        elif "mts is already running" in output:
            self._logger.info("Activation of modem trace on %s succeed" % (str(destination)))
        else:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Unable to ativate modem trace on %s !" % (str(destination)))
        # pylint: enable=E1101

    def deactivate_modem_trace(self):
        """
        Deactivate the modem tracing

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of command failure

        :return: None
        """
        # Activate modem trace with off parameters (deactivate)
        adb = "adb shell "
        cmd = "activate_trace_modem -off && echo Ok || echo NOk"
        output = self._exec(adb + cmd, self._uecmd_default_timeout, False)

        # Check command success
        if "Ok" in output:
            self._logger.debug("Deactivation of modem trace succeed")
        else:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to deactivate modem trace")

    def check_bplog_file_growing(self, time_period, target_rate=50):
        """
        Check wether the bplog file is growing up during a given time period

        :type time_period: int
        :param time_period: The period of time to do the check (in seconds)

        :type target_rate: int
        :param target_rate: The percentage of the given time period where the
                                bplog file should grow up.

        :raise DeviceException.TIMEOUT_REACHED: If the time is up and file
                                                    stop growing up
        :raise AcsConfigException.INVALID_PARAMETER:
                                            In case of invalid input parameter
        :rtype: tuple
        :return: operation status & output log in case of success
        """
        self._logger.info("Checking bplog file size growing up for %d seconds (target: %d %%)..."
                          % (time_period, target_rate))

        if 0 > target_rate > 100:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "The given target rate parameter is invalid (%s) !" % (str(target_rate)))

        current_size = self._get_bplog_file_size()
        previous_size = current_size
        time_count = 1
        stop_growing_count = 0

        while current_size >= previous_size and time_count <= time_period:

            current_size = self._get_bplog_file_size()
            self._logger.debug("Previous bplog file size is: %s bytes" % str(previous_size))
            if current_size == previous_size:
                stop_growing_count += 1
                self._logger.debug("The bplog file stop growing up since last update (%s s, count: %s) !"
                                   % (str(time_count), str(stop_growing_count)))

            # Update loop
            previous_size = current_size
            time_count += 1
            time.sleep(1)

        # If did not grow from given percentage of the time period can say test fail
        # target_rate should be an integer value from 0 to 100
        # compute how many time the file grow
        growing_count = time_period - stop_growing_count
        growing_rate = (growing_count / float(time_period)) * 100

        # compute the target rate with given values
        stop_growing_rate = 100 - growing_rate

        if growing_rate < target_rate:
            err_msg = "The bplog file has stopped growing up for %s seconds over %s expected (%s %% of total time)" \
                % (str(stop_growing_count), str(time_period), str(stop_growing_rate))
            raise DeviceException(DeviceException.TIMEOUT_REACHED, err_msg)
        else:
            output_message = "The bplog file was continuously growing during %d seconds (%s %% of total time)" \
                % (time_period, str(growing_rate))
            self._logger.info(output_message)

            return Global.SUCCESS, output_message

    def _get_bplog_file_size(self):
        """
        Gets the bplog file size on the DUT.

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of error

        :rtype: int
        :return: The bplog file size in bytes
        """
        # Check BP log location
        if self._bplog_location is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknow BPLog location set !")

        # by default logs are located on internal memory (emmc)
        target_dest = "/logs/"

        if self._bplog_location == BPLOG_LOC.SDCARD:
            target_dest = "/sdcard/logs/"
        elif self._bplog_location == BPLOG_LOC.DATA:
            target_dest = "/data/logs/"

        # adb -d shell stat -c %s <file>
        cmd = "adb shell stat -c %%s %sbplog.istp" % target_dest
        output = self._exec(cmd, self._uecmd_default_timeout, False)

        if output is not None and output.isdigit():
            self._logger.debug("Current bplog file size is: %s bytes" % output)
            return int(output)
        else:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR,
                                  "Unable to get bplog file size, bad value %s !" % (str(output)))

    def set_emergency_numbers(self, new_emergency_number, overwrite=False):
        """
        Add a new emergency number in emergency list and verify

        :type new_fake_emergency_number: str
        :param new_fake_emergency_number: New fake emergency number

        :type overwrite: boolean
        :param overwrite: True to overwrite the entire list , False to just add a new one
        """
        if not overwrite:
            number_to_set = self.get_emergency_numbers()
            for number in new_emergency_number:
                if number not in number_to_set:
                    number_to_set.append(number)
        else:
            number_to_set = new_emergency_number

        number_to_set = str(number_to_set).replace("[", "").replace("]", "").replace("'", "").replace(" ", "")
        # Run the command with an arbitrary timeout value
        self._exec("adb shell setprop ril.ecclist %s" % number_to_set)
        time.sleep(3)

    def is_rat_compatible_with_pref_network(self, rat, pref_network):
        """
        Checks if the rat passed as parameters is compatible with the prefered
        network.
        :type rat: str
        :param rat: the network type str value
        :type pref_network: str
        :param pref_network: the network preference name
        :rtype: bool
        :return: True if the rat and prefered_network are compatible, False
        otherwise.
        :raise AcsConfigException: if C{rat} does not match any value of
        C{NETWORK_PREFERENCES}, and if C{pref_network} does not match any value of
        C{NETWORK_TYPES}.
        """
        compatible = False
        if not self._networking.is_preferred_network_type_valid(pref_network):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Invalid parameter value for 'pref_network': %s" % str(pref_network))
        if rat not in NETWORK_TYPES:
            # If we have not found the value at this point,
            # check the equivalent values for this parameter
            if rat in self.NETWORK_EQUIVALENCES:
                # Replace the parameter value with its equivalent
                rat = self.NETWORK_EQUIVALENCES[rat]
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Invalid parameter value for 'rat': %s" % str(rat))
        if pref_network in PreferredNetwork.GSM_PREFERRED_NETWORKS and rat in PreferredNetwork.GSM_RAT:
            compatible = True
        elif pref_network in PreferredNetwork.CDMA_PREFERRED_NETWORKS and rat in PreferredNetwork.CDMA_RAT:
            compatible = True
        elif pref_network in PreferredNetwork.EVDO_PREFERRED_NETWORKS and rat in PreferredNetwork.EVDO_RAT:
            compatible = True
        elif pref_network in PreferredNetwork.WCDMA_PREFERRED_NETWORKS and rat in PreferredNetwork.WCDMA_RAT:
            compatible = True
        elif pref_network in PreferredNetwork.LTE_PREFERRED_NETWORKS and rat in PreferredNetwork.LTE_RAT:
            compatible = True
        return compatible

    def check_rat_with_pref_network(self, pref_network, timeout):
        """
        Waits for the DUT to camp on a network compatible with the preferred
        netword passed as parameter.

        :type pref_network: str
        :param pref_network: preferred network choice, should be in the
        NETWORK_PREFERENCES list

        :type timeout: int
        :param timeout: Time to wait for the DUT to camp.

        :raise AcsConfigException if the parameters are not in the expected range
        :raise DeviceException if the DUT does not camp on a valid network before the timeout.

        .. note:: Method useful for LIVE UC
        """
        elapsed_time = 0
        compatible = False
        pref_network = str(pref_network)
        if not isinstance(timeout, int):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "The timeout parameter should be a int.")
        if not self._networking.is_preferred_network_type_valid(pref_network):
            # If wrong network type entered raise an exception.
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown network type: %s" % pref_network)
        # Getting the preferred network set on the phone.
        start_time = time.time()

        try:
            self._phone_system.display_on()
            # Comparing current RAT to pref network during the timeout.
            while elapsed_time < timeout:
                # Wakeup screen to ensure camping
                rat = self.get_current_rat()
                compatible = self.is_rat_compatible_with_pref_network(rat, pref_network)
                if compatible:
                    break
                elapsed_time = time.time() - start_time
        finally:
            self._phone_system.display_off()

        # self._phone_system.display_off()
        if not compatible:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "The DUT is not camped on a compatible network type with "
                                  "the selected preferred network. Camped on %s, preferred network: %s"
                                  % (rat, pref_network))

    def send_at_command(self, serial_device_name, command, timeout=1):
        """
        Send a command to the modem and return its result string

        :type serial_device_name: str
        :param serial_device_name: serial device which represents the modem on the local computer
        :type command: str
        :param command: command to send to the modem
        :type timeout: int
        :param timeout: time to wait result on serial device
        :rtype: str
        :return: result string of the command
        """

        # Add a '\r' character at the end of the command (without it, command end is not detected by the modem)
        from acs_test_scripts.Utilities.UartUtilities import UartUtilities
        command += "\r"

        uart_api = UartUtilities(self._logger)
        uart_api.configure_port(port=serial_device_name, baudrate=9600, timeout=timeout)
        cmd_succeed, command_result = uart_api.uart_run_cmd(command, timeout=timeout, silent_mode=False)
        command_result = command_result.strip("\n\r")

        self._logger.info("send_at_command: result for '{0}' command: '{1}'".format(command.strip("\n\r"),
                                                                                    command_result))

        return command_result

    def set_bp_logs_location(self, location):
        """
        Sets the location for BP logs

        :type location: str
        :param location: location for BP logs

        :raise AcsConfigException: In case the parameter 'location' is invalid
        """
        if location not in BPLOG_LOC:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Invaid parameter for BP logs location")
        self._bplog_location = location
