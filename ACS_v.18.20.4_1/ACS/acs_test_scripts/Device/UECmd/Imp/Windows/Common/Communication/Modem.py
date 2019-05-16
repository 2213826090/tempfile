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
:summary: This file implements the Modem UEcmd for Windows devices
:since: 17/02/2012
:author: emarmounier
"""
from acs_test_scripts.Device.UECmd.Interface.Communication.IModem import IModem
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.UECmdTypes import NETWORK_TYPES
from acs_test_scripts.Device.UECmd.UECmdTypes import PreferredNetwork
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
import time
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class Modem(Base, IModem):

    """
    Modem UEcommands for Windows platforms.
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
        Initializes this instance.
        """
        IModem.__init__(self, device)
        Base.__init__(self, device)
        self._module_name = "Intel.Acs.TestFmk.MBConnectivity"
        self._class_name = "Intel.Acs.TestFmk.MBConnectivity.MBActivity"
        # Retrieve default timeout used for UEcmd in sec and convert it to ms
        self._modem_timeout = self._uecmd_default_timeout * 1000
        # Instantiate the PhoneSystem UE Command category
        self._phone_system = self._device.get_uecmd("PhoneSystem")
        self._wait_btwn_cmd = self._device.get_config("waitBetweenCmd", 5, float)

    def set_modem_power(self, mode):
        """
        Sets the modem power to off or on.

        [ExpectParameter("sw_radio", "on", "off", Optional = false)]

        :type mode: str
        :param mode: can be 'on' to enable
                            'off' to disable

        :return: None
        """

        if mode == 0:
            mode = "off"
        elif mode == 1:
            mode = "on"

        if mode not in ("on", "off"):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter mode is not valid !")

        self._logger.info("Setting software modem radio state to " + mode)

        # Construct the command necessary to launch the UEcmd on embedded side
        # with the method name and the necessary arguments
        function = "SetRadioState"
        args = " time_out=%s sw_radio=%s" % (self._modem_timeout, mode)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args)

    def get_modem_power_status(self):
        """
        Returns the modem power status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        self._logger.info("Getting software modem radio state")

        # Construct the command necessary to launch the UEcmd on embedded side
        # with the method name
        function = "GetRadioState"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        sw_radio = output["values"]["sw_radio"]
        hw_radio = output["values"]["hw_radio"]

        result = 0
        if sw_radio:
            result = 1

        return result

    def get_imsi(self, timeout):
        """
        Returns the IMSI value.
        :type timeout: int
        :param timeout: maximum time allowed (in seconds) in order to retrieve I{IMSI}
        :rtype: str
        :return: The IMSI number
        :raise DeviceException: If the timeout is reached.
        """
        # Retrieve IMSI
        function = "GetSubscriberId"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        imsi = output["values"]["subscriber_id"]
        self._logger.info("Retrieved IMSI is : " + str(imsi))

        return imsi

    def get_network_registration_status(self):
        """
        Returns the registration status value.

        :rtype: str
        :return: the requested value into a list of 1 element(or an error message
        if an error occurred).
        Network registration state:
                "unknown"          Device registration is unknown
                "unregistered"     Not registered to any network
                "searching"        Not registered, but searching
                "registered"       Registered to home network
                "roaming"          Roaming
                "denied"           Registration has been denied
                "partner"          The device is on a roaming partner
        """
        module_name, class_name = self._get_module_and_class_names("Connectivity")
        output = self._internal_uecmd_exec(
            module_name,
            class_name,
            "GetNetworkRegistrationStatus")

        register_state = output["values"]["register_state"]
        reg_list = ["none", "unregistered", "searching", "registered", "roaming", "partner", "denied"]
        if register_state < 0 or register_state >= len(reg_list):
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, "unknown state '%s'" % (register_state))

        return str(reg_list[register_state])

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
        while (time.time() - start_time) <= timeout:
            self._phone_system.wake_screen()
            reg_state = self.get_network_registration_status()
            if reg_state != str_state:
                state_reached = True
                break

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
                "unknown"          device registration is unknown
                "unregistered"  Not registered to any network
                "searching"     Not registered, but searching
                "registered"    Registered to home network
                "roaming"       Roaming
                "denied"        Registration has been denied
                "partner"       The device is on a roaming partner

        :type timeout: int
        :param timeout: Time to wait attempting state

        :rtype: str
        :return: the state reached if state match or raise error
        """
        state_reached = False

        if not isinstance(state, list):
            state = [state]

        # Check state is a list, all test cases use "registered" as "home", so that replace "home" with "registered" in the list below
        for element in state:
            if element not in ["none", "unregistered", "searching", "registered", "roaming", "partner", "denied"]:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "unknown state:" + element)
        str_state = str(state).replace(",", " or")

        msg = "Checks %s state before %s seconds..." % (str_state, str(timeout))
        self._logger.info(msg)
        start_time = time.time()
        while (time.time() - start_time) <= timeout:
            reg_state = self.get_network_registration_status()
            if reg_state in state:
                state_reached = True
                break

        if state_reached:
            return_msg = "State " + reg_state + " has been reached !"
            self._logger.info(return_msg)
            return reg_state
        else:
            return_msg = "State " + str_state + \
                " has not been reached on time !"
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)

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
        Returns the modem status (online/offline).

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        return self.get_modem_power_status()

    def get_sim_operator_info(self, timeout=30):
        """
        Returns the sim I{MNC} and I{MCC}.

        :type timeout: int
        :param timeout: timeout in seconds to wait for SIM card to be ready

        :rtype: dict
        :return: return a dictionary that contain the key (MNC,MCC)
        """
        # We get the necessary info from imsi
        imsi = self.get_imsi(1000)

        mcc = int(imsi[:3])
        mnc = int(imsi[3:5])

        sim_info = {
            "MCC": mcc,
            "MNC": mnc
        }

        return sim_info

    def get_network_type(self):
        """
        Returns the Network type.

        :rtype: str
        :return: network type value can be :
            0x00000000: "DATA_CLASS_NONE",               # 0
            0x00000001: "DATA_CLASS_GPRS",               # 1
            0x00000002: "DATA_CLASS_EDGE",               # 2
            0x00000004: "DATA_CLASS_UMTS",               # 4
            0x00000008: "DATA_CLASS_HSDPA",              # 8
            0x00000010: "DATA_CLASS_HSUPA",              # 16
            0x00000018: "DATA_CLASS_HSPA",               # 24
            0x00000020: "DATA_CLASS_LTE",                # 32
            0x00000003: "DATA_CLASS_EDGE_GPRS",          # 3
            0x00000007: "DATA_CLASS_UMTS_EDGE_GPRS",     # 7
            0x0000001F: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS",   # 31
            0x0000003F: "DATA_CLASS_LTE_HSUPA_HSDPA_UMTS_EDGE_GPRS",    # 63
            0x00010000: "DATA_CLASS_1XRTT",
            0x00020000: "DATA_CLASS_1XEVDO",
            0x00040000: "DATA_CLASS_1XEVDO_REVA",
            0x00080000: "DATA_CLASS_1XEVDV",
            0x00100000: "DATA_CLASS_3XRTT",
            0x00200000: "DATA_CLASS_1XEVDO_REVB",
            0x00400000: "DATA_CLASS_UMB",
            0x80000000: "DATA_CLASS_CUSTOM"
        """
        function = "GetNetworkType"
        args = "time_out=%s" % (self._modem_timeout)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)

        network_type = output["values"]["networkType"]
        self._logger.info("DUT RAT is : " + str(network_type))

        # For camp over EGPRS, function getNetworkType() returns "EDGE"
        # so, returned network type must be EGPRS to be understood with
        # equipment network type
        if network_type == "EDGE":
            network_type = "EGPRS"

        if network_type == "UMTS":
            network_type = "WCDMA"

        return network_type

    def check_network_type_before_timeout(self, network_type, timeout=0):
        """
        Check whether or not the DUT retrieves the RAT provided by the network.

        :type network_type: str
        :param network_type: expected network type to check
        :type timeout: int
        :param timeout: maximum authorized time for DUT registration
        """
        start_time = time.time()
        dut_network_type = None

        self._logger.info("Check DUT RAT matches network RAT (%s) before %d seconds", network_type, timeout)
        time.sleep(self._wait_btwn_cmd)

        # Check that equipment network type is different than "UNKNOWN" to do the RAT check
        if network_type != "UNKNOWN":

            # Check that DUT is registered on the expected network type before timeout
            dut_network_type = self.get_network_type()

            # Compare the service type seen by the DUT with the one of the Network simulator.
            if (dut_network_type == network_type or dut_network_type == "CUSTOM"):
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

        else:  # Equipment network type can not be retrieved
            msg = "Function to retrieve the network type is not present in the equipment driver. RAT Check is skipped."
            self._logger.warning(msg)
        return dut_network_type

    def get_imei(self):
        """
        Returns the IMEI value.

        :rtype: str
        :return: The IMEI number
        """
        self._logger.info("Get IMEI string ")

        function = "GetDeviceID"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        imei = output["values"]["deviceID"]
        self._logger.info("Retrieved IMEI is : " + str(imei))

        return imei

    def get_imeisv(self):
        """
        Returns the IMEISV value.
        :rtype: str
        :return: The IMEISV number
        """
        # windows MBIM doesn't support IMEISV
        # imeiv is predefined in .xml file, it can be 16, 12 or 0 digit. Because windows MBIM doesn't support IMEISV, imeisv in .xml file should be empty,
        # this imeisv function should not be called, and the info message will be displayed  "The imeisv number will not be checked "
        return ""

    def detect_modem_connection(self):
        """
        Returns True if modem is connected and driver installed successful, False otherwise

        :rtype: bool
        :return: modem connection status
        """
        self._logger.info("Dectect Installed Modem ")

        # Construct the command necessary to launch the UEcmd on embedded side
        # with the method name
        function = "GetInterfaces2"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        result = True
        interfaces = output["values"]["interfaces"]
        interface = interfaces[0]
        if interface == "":
            result = False

        return result

    def get_current_network_name(self):
        """
        Returns the current registered network operator name

        :rtype: str
        :return: network name
        """

        # after The device is registered, get the network operator name which the dut has registered to
        # Construct the command necessary to launch the UEcmd on embedded side with the method name
        function = "GetProviderName"

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function)

        provider_name = output["values"]["provider_name"]
        return provider_name

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
        # Initiate networking module only in needed function to avoid loop
        self._networking = self._device.get_uecmd("Networking")
        if not isinstance(timeout, int):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "The timeout parameter should be a int.")
        if not self._networking.is_preferred_network_type_valid(pref_network):
            # If wrong network type entered raise an exception.
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown network type: %s" % pref_network)
        # Getting the preferred network set on the phone.
        start_time = time.time()
        # Comparing current RAT to pref network during the timeout.
        while elapsed_time < timeout:
            # Wakeup screen to ensure camping
            self._phone_system.wake_screen()
            rat = self.get_current_rat()
            compatible = self.is_rat_compatible_with_pref_network(rat, pref_network)
            if compatible:
                break
            elapsed_time = time.time() - start_time
        if not compatible:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "The DUT is not camped on a compatible network type with "
                                  "the selected preferred network. Camped on %s, preferred network: %s"
                                  % (rat, pref_network))

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
        network_type = self.get_network_type()

        # Return the value
        return network_type

    def get_cellular_operator_info(self):
        """
        Returns the cellular MNC and MCC.
        """
        """
        On a I{DSDS} phone the following properties are not returned for Windows: MCC_2, MNC_2
        for the second SIM.
        Window MBIM limitations.

        :rtype: dict
        :return: return a dictionary that contain the keys:
            - MNC
            - MCC
            - MNC_2 (on I{DSDS} phones)
            - MCC_2 (on I{DSDS} phones)
        """

        cellular_info = self.get_sim_operator_info()

        return cellular_info

    def set_preferred_radio(self, radio_type):  # pylint: disable=W0613
        """
        Sets the preferred radio type.

        :type radio_type: str
        :param radio_type: the radio type. Possible value are:
            - "any"
            - "umts"
            - "gsm"
            - "lte"

        :return: None
        """

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()
        """
        #we get provider id first
        function = "GetProviderId"
        #get current provider id from dut
        output   = self._internal_uecmd_exec(module_name, class_name, function, "")
        provider_id = output["values"]["providerID"]
        """

        bflag = False
        if self.get_modem_power_status() == 1:
            self.set_modem_power("off")
            time.sleep(self._wait_btwn_cmd)
            bflag = True

        function = "SetPreferredDataClass"
        args = "provider_id=none data_class=%s time_out=%s" % (radio_type, self._modem_timeout)
        # Launch the UEcmd on the embedded side
        self._internal_uecmd_exec(module_name, class_name, function, args)
        time.sleep(self._wait_btwn_cmd)

        if bflag:
            self.set_modem_power("on")
            time.sleep(self._wait_btwn_cmd)

    def get_preferred_radio(self):
        """
        Returns the preferred radio type (any, umts, gsm, lte)

        :note: Preferred Network Mode correspondance:
            0: "DATA_CLASS_NONE",
            1: "DATA_CLASS_GPRS",
            2: "DATA_CLASS_EDGE",
            4: "DATA_CLASS_UMTS",
            8: "DATA_CLASS_HSDPA",
            16: "DATA_CLASS_HSUPA",
            24: "DATA_CLASS_HSPA",
            32: "DATA_CLASS_LTE",
            3: "DATA_CLASS_EDGE_GPRS",
            7: "DATA_CLASS_UMTS_EDGE_GPRS",
            31: "DATA_CLASS_HSPA_HSUPA_HSDPA_UMTS_EDGE_GPRS",
            63: "DATA_CLASS_LTE_HSUPA_HSDPA_UMTS_EDGE_GPRS",
            65536: "DATA_CLASS_1XRTT",
            131072: "DATA_CLASS_1XEVDO",
            262144: "DATA_CLASS_1XEVDO_REVA",
            524288: "DATA_CLASS_1XEVDV",
            1048576: "DATA_CLASS_3XRTT",
            2097152: "DATA_CLASS_1XEVDO_REVB",
            4194304: "DATA_CLASS_UMB",
            2147483648: "DATA_CLASS_CUSTOM"

        :rtype: str
        :return: (any, umts, gsm, lte)
        """
        function = "GetPreferredDataClass"
        args = "provider_id=none time_out=%s" % (self._modem_timeout)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)
        preferred = output["values"]["preferred_data_class"]

        return preferred

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
        # Initiate networking module only in needed function to avoid loop
        self._networking = self._device.get_uecmd("Networking")
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

    def get_cell_power(self):
        """
        Gets the serving cell power in dBm

        :return : the registered cell power in dBm.
        :rtype: int
        """
        self._logger.info("Get cell power")
        function = "GetSignalStrength"
        args = "time_out=%s" % (self._modem_timeout)

        # Get the method and class name of the UEcommand on the embedded side
        module_name, class_name = self._get_module_and_class_names()

        # Launch the UEcmd on the embedded side
        output = self._internal_uecmd_exec(module_name, class_name, function, args)
        rx_lev = int(output["values"]["signal_strength"])
        self._logger.info("Get cell power: {0}".format(rx_lev))
        # Windows  reports signal strength received by the Mobile Broadband device. For GSM based devices it reports signal strength as signal strength received in a coded value.
        # The method to get rx_lev from rssi is rx_lev = int((113-rssi)/2).
        if rx_lev == 99:
            # if level is equal to 99 it means windows cannot measure signal strength on device
            raise AcsConfigException(DeviceException.INVALID_DEVICE_STATE,
                                     "Signal strength cannot be measured")
        else:
            rssi_dBm = 2 * rx_lev - 113
        return rssi_dBm
