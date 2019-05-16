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
:summary: This script implements the interface for unitary actions
for modem features
:since: 09/08/2010
:author: vgombert
"""
from ErrorHandling.DeviceException import DeviceException


# pylint: disable=W0613


class IModem():

    """
    Abstract class that defines the interface to be implemented
    by modem handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{DeviceException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

    def set_modem_power(self, mode):
        """
        Sets the modem power to off or on.

        :type mode: str or int
        :param mode: can be ('1', 1) to enable
                            ('0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_modem_power_status(self):
        """
        Returns the modem power status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_lac(self):
        """
        Returns the LAC value.

        :rtype: str List
        :return: The LAC number list
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def register_to_network(self, name='default'):
        """
        Attempts to register to a network

        :type name: str
        :param name: name of the network , can be omitted for default network

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def deregister_all(self):
        """
        Deregister from the network

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_network_registration_status(self):
        """
        Returns the registration status value.

        :rtype: str
        :return: Network registration state:
                "unregistered"  Not registered to any network
                "registered"    Registered to home network
                "roaming"       Roaming (national or international)
                "unknown"       Status is unknown
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_cdk_registration_bfor_timeout(self, timeout):
        """
        Check if the network registration status is registred before timeout.

        :type timout: int
        :param timeout: time allowed (in seconds) in order to reach the status I{registered}.

        :return: None (raise exception if an error occurred).
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_cdk_no_registration_bfor_timeout(self, timeout):
        """
        Check if the network registration status is not registered before timeout.

        :type timout: int
        :param timeout: time allowed (in seconds) in order to reach a status not equal to registered.

        :return: None (raise exception if an error occurred).
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_preferred_radio(self, radio_type):
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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_emergency_numbers(self):
        """
        Returns the list of emergency numbers inside the device

        :rtype: list
        :return: Emergency numbers
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_modem_online(self, mode):
        """
        Sets the modem to offline or online.

        :type mode: str or int
        :param mode: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_modem_online_status(self):
        """
        Returns the modem online status.

        :rtype: int
        :return: 0 for OFF or 1 for ON
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_sim_operator_info(self, timeout=30):
        """
        Returns the sim MNC and MCC.

        :type timeout: int
        :param timeout: timeout in seconds to wait for SIM card to be ready

        :rtype: dict
        :return: return a dictionary that contain the key (MNC,MCC)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_current_rat(self):
        """
        Returns the raw value of the current I{RAT} as returned
        by Android.

        This method is similar to C{get_network_type} but without
        any post-processing.

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_network_type_before_timeout(self,
                                          network_type,
                                          timeout=0):
        """
        Check whether or not the DUT retrieves the RAT provided by the network.
        :type network_type: str
        :param network_type: expected network type to check
        :type timeout: integer
        :param timeout: maximum authorized time for DUT registration
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_imei(self):
        """
        Returns the IMEI value.
        :rtype: str
        :return: The IMEI number
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_second_imei(self):
        """
        Returns the IMEI value for second SIM.
        :rtype: str
        :return: The IMEI number for second SIM
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_imeisv(self):
        """
        Returns the IMEISV value.
        :rtype: str
        :return: The IMEISV number
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_data_sim(self, sim_index):
        """
        Set given sim index as default DATA sim on device

        :type sim_index: int
        :param sim_index: Sim index to be set as data sim (1/2)

        .. warning:: The UeCommand is available only for DUAL sim device
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_data_state(self):
        """
        Retrieve the data state of the device

        .. warning:: This UeCmd is actually implemented for DSDS only, but can
                    be implemented for usual devices. In that case please follow
                    this host/embedded convention.

        :rtype: UeCmd.DATA_STATE
        :return: Return the data state of the device for the current target sim
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_data_state(self, expected_state):
        """
        Checks whether the state of a data of given sim matches the given one.

        .. warning:: This UeCmd is actually implemented for DSDS only, but can
                    be implemented for usual devices. In that case please follow
                    this host/embedded convention.

        :type state: UECmd.DATA_STATE
        :param state: expected state (see UECmd.DATA_STATE)

        :raise DeviceException.INVALID_DEVICE_STATE: If not match expected data state

        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_data_state(self, state, timeout):
        """
        Waits to reach a data state until a timeout.

        .. warning:: This UeCmd is actually implemented for DSDS only, but can
                    be implemented for usual devices. In that case please follow
                    this host/embedded convention.

        :type state: UECmd.DATA_STATE
        :param state: expected state (see UECmd.DATA_STATE)

        :type timeout: int
        :param timeout: maximum time to wait in seconds

        :raise DeviceException.TIMEOUT_REACHED: If the time is out before reach expected state

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_target_sim_role(self, expected_role):
        """
        Checks whether the target sim role correspond to the given one

        .. warning:: The UeCommand is available only for DUAL sim device

        :type expected_role: bool
        :param expected_role: expected role (see UECmdTypes.SIM_ROLE)

        :raise DeviceException.INVALID_DEVICE_STATE: If not match expected state.
        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def configure_modem_trace(self, hsi_speed, trace_level):
        """
        Configure the modem tracing

        :type hsi_speed: str
        :param hsi: HSI speed, can be:
                            - u: disable HSI
                            - d: default speed HSI, 78MHz [default]
                            - h: high speed HSI, 156MH

        :type trace_level: int
        :param trace_level: Level of trace, can be:
                            - 0: disable trace
                            - 1: first level trace (bb_sw)
                            - 2: second level trace (bb_sw & 3g_sw) [default]
                            - 3: third level trace (bb_sw & 3g_sw & digrf)

        .. warning:: This UeCmd need a hardware reboot to be effective !

        :raise DeviceException.INVALID_PARAMETER: In case of invalid input parameter

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of command failure

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def activate_modem_trace(self, destination, file_size_option):
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

        :raise DeviceException.INVALID_PARAMETER: In case of invalid input parameter

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of command failure

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def deactivate_modem_trace(self):
        """
        Deactivate the modem tracing

        :raise DeviceException.INTERNAL_EXEC_ERROR: In case of command failure

        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_bplog_file_growing(self, time_period, target_rate):
        """
        Check wether the bplog file is growing up during a given time period

        :type time_period: int
        :param time_period: The period of time to do the check (in seconds)

        :type target_rate: int
        :param target_rate: The percentage of the given time period where the
                                bplog file should grow up.

        :raise DeviceException.TIMEOUT_REACHED: If the time is up and file stop growing up
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_rat_with_pref_network(self, pref_network, timeout):
        """
        Waits for the DUT to camp on a network compatible with the preferred
        netword passed as parameter.
        :type pref_network: str
        :param pref_network: preferred network choice, should be in the
        NETWORK_PREFERENCES dictionary
        :type timeout: int
        :param timeout: Time to wait for the DUT to camp.
        :raise DeviceException if the parameters are not in the expected
        range. If the DUT does not camp on a valid network before the
        timeout.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def detect_modem_connection(self):
        """
        Returns True if modem is connected and driver installed successful, False otherwise

        :rtype: bool
        :return: modem connection status
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_current_network_name(self):
        """
        Returns the current registered network operator name

        :rtype: str
        :return: network name
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_lte_coex_manager_messages(self, mode="VERBOSE"):
        """
        Enables or disables the LTE coex manager messages

        @param mode : VERBOSE to enable the logs, empty to disable them.
        @type mode: string
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_lte_coex_manager_messages_state(self):
        """
        Gets the LTE coex manager state.

        @return : TestConst.STR_ON if enabled, TestConst.STR_OFF else.
        @rtype: string
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_cell_power(self):
        """
        Gets the serving cell power in dBm

        @return : the registered cell power in dBm.
        @rtype: int
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_bp_logs_location(self, location):
        """
        Sets the location for BP logs

        :type location: str
        :param location: location for BP logs

        :raise AcsConfigException: In case the parameter 'location' is invalid
        """

        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def activate_modem_trace_amtl(self, option, timeout=30):
        """
        Activates the modem trace using AMTL application

        :param option : Option which must be activated
        :type option: string
        :param timeout : Timeout for activation
        :type timeout: int

        :raise DeviceException.TIMEOUT_REACHED if timeout has been reached
        :raise DeviceException.OPERATION_FAILED if activation procedure has failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def deactivate_modem_trace_amtl(self, option, timeout=30):
        """
        Deactivates the modem trace using AMTL application

        :param option : Option which must be deactivated
        :type option: string
        :param timeout : Timeout for deactivation
        :type timeout: int

        :raise DeviceException.TIMEOUT_REACHED if timeout has been reached
        :raise DeviceException.OPERATION_FAILED if de-activation procedure has failed
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_modem_trace_configuration_amtl(self, timeout=60):
        """
        Retrieve the modem trace configurations from AMLT

        :rtype: list
        :return: The list with the modem trace configuration

        :raise DeviceException.TIMEOUT_REACHED if timeout has been reached
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def close_amtl_app(self):
        """
        Close the AMTL application

        :rtype: None
        :return: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_ims_modem_configuration(self, ims_param_keys, params_values):
        """
        Configure the modem with IMS parameters
        :param ims_param_keys: Contains the keys which needs to be set
        :type ims_param_keys: tuple
        :param param_values: Values for the parameters
        :type param_values: tuple
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def set_single_ims_param(self, ims_param_key, param_value):
        """
        Configure the modem with a single IMS parameter

        :param ims_param_key: Contains the key which needs to be set in modem side
        :type ims_param_key: int
        :param param_value: Value for the parameter
        :type param_value: str or int
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def turn_ims(self, mode):
        """
        Turn ON or OFF the IMS services
        :param mode: Mode to set the IMS services
        :type mode: str or int

        :return: None
        :raise: AcsConfigException if wrong value for input parameter
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
