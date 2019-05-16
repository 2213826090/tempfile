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
:summary: Implements Classes and functions used in Communication usecases
:since: 15/04/2013
:author: dgonza4x
"""
import os
import re
import string
from xml.dom import minidom
from lxml import etree

import serial
import time
import threading
import xpath
from Core.PathManager import Paths
from UtilitiesFWK.Utilities import Global, format_exception_info
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies
from math import ceil
from acs_test_scripts.Utilities.LocalConnectivityUtilities import get_bt_channel_from_freq
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure, DuplexThroughputMeasure


class IDataAnalyser:

    """
    Interface defining how a data should be analyzed
    """

    def analyse_line(self, line):
        """
        Analyze a line

        :param line: data provided by the C{SerialHandler}, to be analyzed
        :type line: str

        :rtype: tuple
        :return: Tuple containing 3 values:
        - [0] : boolean meaning if the result has been found
        - [1] : verdict (Global.FAILURE of Global.SUCCESS)
        - [2] : analysis message as str, giving more details about verdict
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)


class SerialHandler:

    """
    Class handling serial connection, communication and analysis
    """
    DEFAULT_TIMEOUT = 5

    def __init__(self):
        """
        Constructor.
        """
        self.__analyzer = None
        self.__serial = None
        self.__port = None
        self.__is_connected = False
        self.__timeout = self.DEFAULT_TIMEOUT

    def set_data_analyser(self, data_analyzer):
        """
        Set the data analyzer, object implementing C{IDataAnalyser}

        :param data_analyzer: data analyzer
        :type data_analyzer: object

        :return: None
        """
        self.__analyzer = data_analyzer

    def set_default_timeout(self, timeout):
        """
        Set the maximum time to wait for analysis.

        :param timeout: maximum among of time to wait, in second,
        used when analyzing command response

        :return: None
        """
        self.__timeout = timeout

    def connect(self, tty_port,
                baud_rate=115200,
                byte_size=serial.EIGHTBITS,
                parity_checking=serial.PARITY_NONE,
                stop_bits=serial.STOPBITS_ONE,
                read_timeout=1):
        """
        Set and establish a serial connection
        to a specified port

        :param tty_port: port name
        :type tty_port: str

        :param baud_rate: used baud rate
        :type baud_rate:  int

        :param byte_size: number of databits
        :type byte_size: int

        :param parity_checking: enable parity checking
        :type parity_checking: str

        :param stop_bits: number of stopbits
        :type stop_bits: int

        :param read_timeout: set a timeout value used when reading a line
        :type read_timeout: int

        :return: None
        """
        self.__port = tty_port
        if self.__is_connected:
            self.disconnect()
        try:
            # setup and open serial connection
            self.__serial = serial.Serial(
                port=self.__port,
                baudrate=baud_rate,
                bytesize=byte_size,
                parity=parity_checking,
                stopbits=stop_bits,
                timeout=read_timeout)
            # flush previous data
            if self.__serial.isOpen():
                self.__serial.flushInput()
                self.__serial.flushOutput()
            self.__is_connected = True
        except Exception as excp:
            raise TestEquipmentException(
                TestEquipmentException.OPERATION_FAILED, str(excp))

    def get_port(self):
        """
        Getter returning port name

        :rtype: object
        :return: port name if a connection has been established, None otherwise
        """
        return self.__port

    def is_connected(self):
        """
        Returns connection state.

        :rtype: bool
        :return: True is a connection is established, False otherwise
        """
        return self.__is_connected

    def disconnect(self):
        """
        Disconnect is necessary a connection previously made.

        :return: None
        """
        if self.__serial is not None:
            self.__serial.close()
        self.__is_connected = False
        self.__port = None
        self.__serial = None

    def write(self, data):
        """
        Write a message to a port, thanks to a connection previously made

        :param data: message to write
        :type data: str

        :return: None
        """
        if not self.__is_connected:
            raise TestEquipmentException(
                TestEquipmentException.CONNECTION_ERROR,
                "No connection established")
        elif not self.__serial.isOpen():
            raise TestEquipmentException(
                TestEquipmentException.CONNECTION_LOST,
                "Connection has been released by another entity")
        self.__serial.write("%s\r\n" % data)

    def analyse(self):
        """
        Analyze data received from a port, thanks to a connection already
        established, and returns verdict and analysis

        :rtype: list
        :return: List containing the verdict and a message explaining it.
        """
        end_timer = time.time() + self.__timeout
        is_found = False
        verdict = Global.FAILURE
        msg = ""

        if not self.__is_connected or not self.__serial.isOpen():
            return (Global.FAILURE,
                    "Connection hasn't been established or has been released")
        try:
            while not is_found and \
                    time.time() <= end_timer:
                # Read a line
                answer_line = self.__serial.readline()
                # Remove unwanted characters
                answer_line = answer_line.translate(None, "\r\n")
                # Ignore the line if empty or None
                if answer_line is None or answer_line == "":
                    continue
                else:
                    # Analysis answer
                    is_found, verdict, msg = self.__analyzer.analyse_line(answer_line)
            if is_found:
                return verdict, msg
        except Exception as excp:
            return (Global.FAILURE,
                    "Connection problem has occurred during analysis : " + str(excp))

        return Global.FAILURE, "No answer detected"

    def write_and_analyse(self, text):
        """
        From a given connection made, write a message and analyze the answer.

        :param text: message to write
        :type text: str

        :rtype: list
        :return: List containing the verdict and a message explaining it.
        """
        self.write(text)
        return self.analyse()

    def get_serial(self):
        """
        Returns the instantiated serial that handle connection.

        :rtype: serial.Serial
        :return: the serial used if the handler is connected to a port,
        None otherwise
        """
        return self.__serial

    def send_at_command_and_get_result(self, at_command, serial_command_timeout):
        """
        Returns the response to AT command.

        @param at_command: at command to be sent
        @type at_command: str

        @param serial_command_timeout: at command to be sent
        @type serial_command_timeout: int

        @rtype: list
        @return: List containing the verdict and a message explaining it.

        Rise DeviceException in case verdict is failed.
        """
        self.set_default_timeout(serial_command_timeout)
        verdict, msg = self.write_and_analyse(at_command)
        if verdict is Global.SUCCESS:
            return verdict, msg
        else:
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR,
                                         "AT command result is: %s, %s " % (verdict, msg))


class ATCommandAnalyser(IDataAnalyser):

    """
    Class analyzing AT command result received by an instance of
    C{SerialHandler}
    """

    def __init__(self, expected_result):
        """
        Constructor
        """
        self.__expected_result = expected_result
        self.__analyse_msg = ""

    def analyse_line(self, line):
        """
        Analyze a line

        :param line: data provided by the C{SerialHandler}, to be analyzed
        :type line: str

        :rtype: tuple
        :return: Tuple containing 3 values:
        - [0] : boolean meaning if the result has been found
        - [1] : verdict (I{Global.FAILURE} of I{Global.SUCCESS})
        - [2] : analysis message as str, giving more details about verdict
        """
        is_result_found = False
        verdict = Global.FAILURE

        msg = self.__analyse_msg + line
        # Check answer AT command status
        if "OK" in msg or "ERROR" in msg:
            is_result_found = True
            # reset analyse_msg for future analysis
            self.__analyse_msg = ""
        else:
            # concatenate line to analyze with analysis message
            self.__analyse_msg = msg
        # Check AT command answer with expected result
        if self.__expected_result in msg:
            verdict = Global.SUCCESS

        return is_result_found, verdict, msg

class MeasureConnectionTargets:

    """
    Structure that represent connection measure objects

    .. attention:: Default units value is s
    """

    (SEC_UNIT, MIN_UNIT, HOUR_UNIT) = (['s', 'sec', "S"],
                                        ['min', "Min", "MIN"],
                                        ['h', 'H'])

    def __init__(self):
        self.connection_target_value = float(0)
        self.connection_failure_value = float(0)
        self.connection_unit = MeasureConnectionTargets.SEC_UNIT[0]


class MeasureThroughputTargets:

    """
    Structure that represent throughput measure objects

    .. attention:: Default units value is Kbps
    """
    def __init__(self):
        self.ul_target = ThroughputMeasure(0, ThroughputMeasure.KBPS_UNIT)
        self.ul_failure = ThroughputMeasure(0, ThroughputMeasure.KBPS_UNIT)
        self.dl_target = ThroughputMeasure(0, ThroughputMeasure.KBPS_UNIT)
        self.dl_failure = ThroughputMeasure(0, ThroughputMeasure.KBPS_UNIT)
        self.bler = 0.

    def add_secondary_carrier_throughput_targets(self, other):
        """
        Add Secondary Component Carrier throughput targets to current throughput

        :type other: MeasureThroughputTargets
        :param other: Secondary Component Carrier throughput targets to add
        """
        self.dl_target += other.dl_target
        self.dl_failure += other.dl_failure

    def set_failure_throughput_from_config(self, dut_config, failure_targets=None,
                                           kpi_test=False, tc_name=None):
        """
        Set failure througput from config

        :type dut_config: lxml.etree
        :param dut_config: DUT config

        :type failure_targets: str
        :param failure_targets

        :type kpi_test: bool
        :param kpi_test: True if it's a KPI test, False either

        :type tc_name: str
        :param tc_name: Test Case name
        """
        # Default throughput ration in pourcentage
        DEFAULT_NFT_THROUGHPUT_RATIO = float(92.)
        DEFAULT_FUTE_THROUGHPUT_RATIO = float(70.)

        if failure_targets is not None and failure_targets != "":
            if failure_targets == "FUTE":
                failure_ratio = float(dut_config.get("futeThroughputRatio",
                                                     DEFAULT_FUTE_THROUGHPUT_RATIO)) / 100.0
            elif failure_targets == "NFT":
                failure_ratio = float(dut_config.get("nftThroughputRatio",
                                                     DEFAULT_NFT_THROUGHPUT_RATIO)) / 100.0
            else:
                try:
                    (self.ul_failure, self.dl_failure) = \
                        self._parse_uldl_throughputs(failure_targets)
                    failure_ratio = None
                except:  # FAILURE_TARGETS not well formed
                    raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR,
                                             "FAILURE_TARGETS field not in the good format: (FUTE|NFT|UL:xx,DL:xx)")
        else:  # FAILURE_TARGET doesn't exist
            if kpi_test:
                if "kpiThroughputTargetsFile" in dut_config and dut_config.get("kpiThroughputTargetsFile") != "":
                    kpi_targets_file_name = os.path.join("Throughput_Targets", dut_config.get("kpiThroughputTargetsFile"))
                else:
                    raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR, "Attribute kpiThroughputTargetsFile doesn't exist or is empty in Device Model\n")

                kpi_targets_file = ConfigsParser(kpi_targets_file_name)

                tc_name = os.path.basename(tc_name)
                milestone = dut_config.get("milestone", "PV")
                target_expression = "/TestCases/TestCase[@Name='%s']/Target[@Milestone='%s']/attribute::Value" % (tc_name, milestone)
                target_results = kpi_targets_file._etree_document.xpath(target_expression)
                if target_results:
                    targets = target_results[0]
                    try:
                        (self.ul_failure, self.dl_failure) = \
                            self._parse_uldl_throughputs(targets)
                        failure_ratio = None
                    except:
                        raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR,
                                                 "Target value field not in the good format: (UL:xx,DL:xx)")

                else:
                    raise AcsConfigException(AcsConfigException.READ_PARAMETER_ERROR,
                                             "Parameter not found in KPI Throughput Target parameter value: TestCase='%s', Milestone='%s'"
                                             % (tc_name, milestone))


            else:  # Not KPI test
                failure_ratio = float(dut_config.get("nftThroughputRatio",
                                                     DEFAULT_NFT_THROUGHPUT_RATIO)) / 100.0

        if failure_ratio:
            self.ul_failure = self.ul_target * failure_ratio
            self.dl_failure = self.dl_target * failure_ratio

    @staticmethod
    def _parse_uldl_throughputs(uldl_throughputs):
        """
        Parse string of format as (UL:XX.X,DL:XX.X)
        on a tuple of two floats (ul_throughput, dl_throughput)

        :type uldl_throughputs: str
        :param uldl_throughputs: UL/DL in string format

        :rtype: (ThroughputMeasure, ThroughputMeasure)
        :return UL/DL Tuple or None if parameter has not a good format
        """
        uldl_dict = {}
        for throughput in uldl_throughputs.replace(" ", "").split(","):
            (key, value) = throughput.split(":")
            uldl_dict[key] = float(value) * 1000.

        if "UL" in uldl_dict and "DL" in uldl_dict:
            return (ThroughputMeasure(uldl_dict["UL"], ThroughputMeasure.KBPS_UNIT),
                    ThroughputMeasure(uldl_dict["DL"], ThroughputMeasure.KBPS_UNIT))
        elif "UL" in uldl_dict:
            return (ThroughputMeasure(uldl_dict["UL"], ThroughputMeasure.KBPS_UNIT),
                    ThroughputMeasure(0.0, ThroughputMeasure.KBPS_UNIT))
        elif "DL" in uldl_dict:
            return (ThroughputMeasure(0.0, ThroughputMeasure.KBPS_UNIT),
                    ThroughputMeasure(uldl_dict["DL"], ThroughputMeasure.KBPS_UNIT))
        else:
            return None

    def convert_downlink_to(self, unit):
        """
        Converts target and failure for downlink values.

        :type unit: Unit
        :param unit: unit wanted.
        """
        self.dl_target.convert_to(unit)
        self.dl_failure.convert_to(unit)

    def convert_uplink_to(self, unit):
        """
        Converts target and failure for uplink values.

        :type unit: Unit
        :param unit: unit wanted.
        """
        self.ul_target.convert_to(unit)
        self.ul_failure.convert_to(unit)


class LteParameters:
    """
    Structure that represent LTE parameters objects
    """

    def __init__(self):
        self.antennas_number = ""
        self.bandwidth = ""
        self.transmission_mode = ""
        self.dl_nb_rb = 0
        self.dl_i_mcs = 0
        self.ul_nb_rb = 0
        self.ul_i_mcs = 0


class ConfigsParser:
    """
        This class implements parser for each type of files in _Configs.
        It has been created to allow parsing of xml file in Use Case scripts,
        which cannot be simply instantiated with current implementation of FileParsingManager.
        FileParsingManager is used in CampaignEngine and
        need to have a list of instance arguments
        (bench_config, Device_Catalog..) that we do not necessary.
    """
    # Constant parameter masks used for Wifi Sleep Policy input parameters
    SP_WIFI_CONNECTED_MASK = int('10000', 2)
    SP_DISPLAY_STATE_MASK = int('01000', 2)
    SP_KEEP_WIFI_ON_DURING_SLEEP_MASK = int('00100', 2)
    SP_USB_PLUGGEDIN_MASK = int('00010', 2)
    SP_REMEMBERED_SSID_LIST_EMPTY_MASK = int('00001', 2)

    # Constant ignore parameters mask list used to ignore some
    # Wifi Sleep Policy parameters
    SP_MASK_LIST = [int('00000', 2),
                    int('00001', 2),
                    int('00010', 2),
                    int('00100', 2),
                    int('01000', 2),
                    int('10000', 2),
                    int('00011', 2),
                    int('00101', 2),
                    int('01001', 2),
                    int('10001', 2),
                    int('00110', 2),
                    int('01010', 2),
                    int('10010', 2),
                    int('01100', 2),
                    int('10100', 2),
                    int('11000', 2),
                    int('00111', 2),
                    int('01011', 2),
                    int('10011', 2),
                    int('01101', 2),
                    int('10101', 2),
                    int('11001', 2),
                    int('01110', 2),
                    int('10110', 2),
                    int('11010', 2),
                    int('11100', 2), ]

    APM_INCOMPATIBILITIES = "incompatibilities"
    APM_STATE_AFTER_APM_CYCLE = "state_after_apm_cycle"

    def __init__(self, file_name, config_folder=Paths.CONFIGS):
        """
            Init function when instantiate object
            param : file_name     : xml file to parse
                    config_folder: use in case configs folder is not in current directory
        """

        # Add xml extension if not the case
        if not file_name.endswith(".xml"):
            file_name += ".xml"
        self.dl_failure_value = 0
        self.ul_failure_value = 0
        # Normalize path of folder and concatenate the file name
        config_folder = os.path.normpath(config_folder)
        file_name = os.path.join(config_folder, file_name)

        # Check if file exists
        if not os.path.exists(file_name):
            msg = "File %s not found !" % str(file_name)
            raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND, msg)

        # Parse current configuration file
        try:
            self._document = minidom.parse(file_name)
            self._etree_document = etree.parse(file_name)

        except Exception as e:
            error_msg = "Failed to parse file: %s" % file_name
            LOGGER_TEST_SCRIPT.error(error_msg)
            raise e

    def _get_attribute_value(self,
                             node,
                             attribute_name):
        """
        Returns the value of the given attribute for the given I{DOM} C{Node}, using LXML parser

        :type node: Node
        :param node: the I{DOM} C{Node} among which the attribute's value
            will be looked for.

        :type attribute_name: string
        :param attribute_name: the name of the requested attribute

        :rtype: string
        :return:
            - the requested attribute value if it exists
            - C{None} otherwise
        """
        return node.attrib.get(attribute_name)

    def _get_minidom_attribute_value(self,
                                     dom_node,
                                     attribute_name):
        """
        Returns the value of the given attribute for the given I{DOM} C{Node}, using MiniDOM parser
        .. deprecated:: The usage of MiniDOM is deprecated. Please use LXML instead.

        :type dom_node: Node
        :param dom_node: the I{DOM} C{Node} among which the attribute's value
            will be looked for.

        :type attribute_name: string
        :param attribute_name: the name of the requested attribute

        :rtype: string
        :return:
            - the requested attribute value if it exists
            - C{None} otherwise
        """
        value = None
        attributes = dom_node.attributes
        for index in range(attributes.length):
            if attributes.item(index).name == attribute_name:
                value = attributes.item(index).value
                break
        return value

    def __search_wifi_policy_generic_node(self, device_model_node,
                                          wifi_connected_ori,
                                          display_state_ori,
                                          keep_wifi_on_during_sleep_ori,
                                          usb_pluggedin_ori,
                                          remembered_ssid_list_empty_ori,
                                          mask_id=0):
        """
        Search for a node. If nothing found, ignores more and more parameters
        to try to find a more generic node.

        :type device_model: String
        :param device_model: Name of the device model under test
        :type wifi_connected: 'true'/'false' string
        :param wifi_connected: Is DUT wifi connected to an Access Point?
        :type display_state: string
        :param display_state: Is the display "off",
                        or on outside the WiFi menu "on_out_wifi_menu",
                        or on inside the WiFi menu "on_in_wifi_menu"?
        :type keep_wifi_on_during_sleep: string
        :param keep_wifi_on_during_sleep: User wifi sleep policiy set.
                    possible values: "always", "only_when_plugged_in", "never"
        :type usb_pluggedin: 'true'/'false' string
        :param usb_pluggedin: Is USB plugged in DUT?
        :type remembered_ssid_list_empty: 'true'/'false' string
        :param remembered_ssid_list_empty: When not connected,
                        is there any known networks?
        :type mask_id: int
        :param mask_id: Id in the SP_MASK_LIST of the ignore_mask to use
                        for this node research

        :rtype: DOM node
        :return: None if no node has been found or the DOM node that
                corresponds to the 5 input parameters
        """
        if mask_id >= len(self.SP_MASK_LIST):
            return []

        # Unset parameters to ignore
        ignore_mask = self.SP_MASK_LIST[mask_id]
        if ignore_mask & self.SP_WIFI_CONNECTED_MASK != 0:
            wifi_connected = ""
        else:
            wifi_connected = wifi_connected_ori
        if ignore_mask & self.SP_DISPLAY_STATE_MASK != 0:
            display_state = ""
        else:
            display_state = display_state_ori
        if ignore_mask & self.SP_KEEP_WIFI_ON_DURING_SLEEP_MASK != 0:
            keep_wifi_on_during_sleep = ""
        else:
            keep_wifi_on_during_sleep = keep_wifi_on_during_sleep_ori
        if ignore_mask & self.SP_USB_PLUGGEDIN_MASK != 0:
            usb_pluggedin = ""
        else:
            usb_pluggedin = usb_pluggedin_ori
        if ignore_mask & self.SP_REMEMBERED_SSID_LIST_EMPTY_MASK != 0:
            remembered_ssid_list_empty = ""
        else:
            remembered_ssid_list_empty = remembered_ssid_list_empty_ori

        expression = "WifiSleepPolicies/WifiSleepPolicy[@WifiConnected='%s' and " % wifi_connected
        expression += "@Display='%s' and " % display_state
        expression += "@KeepWifiOnDuringSleep='%s' and " % keep_wifi_on_during_sleep
        expression += "@UsbPluggedIn='%s' and " % usb_pluggedin
        expression += "@RememberedSsidListEmpty='%s']/Behavior" % remembered_ssid_list_empty

        behavior_node_found = device_model_node.xpath(expression)

        if behavior_node_found is None or len(behavior_node_found) == 0:
            return self.__search_wifi_policy_generic_node(device_model_node,
                                                          wifi_connected_ori,
                                                          display_state_ori,
                                                          keep_wifi_on_during_sleep_ori,
                                                          usb_pluggedin_ori,
                                                          remembered_ssid_list_empty_ori,
                                                          mask_id + 1)
        else:
            return behavior_node_found

    def parse_amplitude_offset_table(self):
        """
            This function parses the Amplitude offset Table
            to get list frequency (in MHZ) and corresponding correction offset (in dB)
        """

        amp_offset_table = {}

        try:
            # Retrieve all parameters
            amp_offset_table["FrequencyList"] = xpath.findvalues(
                '/AmplitudeOffsetTable/FrequencyList/Value',
                self._document)[0]

            amp_offset_table["OffsetList"] = xpath.findvalues(
                '/AmplitudeOffsetTable/OffsetList/Value',
                self._document)[0]
            try:
                amp_offset_table["SecondAntennaOffsetList"] = xpath.findvalues(
                    '/AmplitudeOffsetTable/SecondAntennaOffsetList/Value',
                    self._document)[0]
            except IndexError:
                pass

            # Return all parameter's values
            return amp_offset_table

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_audio_quality_target(self, meas_type, codec_type, call_type):
        """
        This function parses the Audio_Quality_Targets.xml file
        to retrieve target for selected measurement

        :type meas_type: str
        :param meas_type: Type of audio quality measurement
        :type codec_type: str
        :param codec_type: Type of audio codec (NB/WB/SWB)
        :type call_type: str
        :param call_type: Type of voice call (CSV/VoIP/VoLTE)
        """
        meas_codec = meas_type.upper() + "_" + codec_type
        try:
            target_expression = "/Sections/Section[@Name=\"%s\"]" % call_type + \
                "/%ss/%s" % (meas_type, meas_type) + "[@Name=$TEST]"
            target_scheme = xpath.find(
                target_expression,
                self._document,
                TEST=meas_codec)
            target_scheme = target_scheme[0]
            target = self._get_minidom_attribute_value(target_scheme, "Value")
            return target
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_energy_management_targets(self, ucase_name,
                                        input_dict, input_platform="default"):
        """
        Parses the Energy_Management configuration XML file
        for the given I{use case name} into a dictionary and returns it.

        :type ucase_name: String
        :param ucase_name: The use case name

        :type input_dict: dict
        :param input_dict: the spefic key:value for this capabilities

        :type input_platform: String
        :param input_platform: platform for this capabilities

        :rtype: dict
        :return : all data read from the Energy Management configuration file.
        """

        class __DictionaryList(list):

            """
            this is an object that act both like a list and a dict.
            - Allow you to sort your dict element.
            - Allow you to call list element with str as key
            """

            def __getitem__(self, *args, **kwargs):
                """
                Return a dict if args is an str and this dict contain at least a key "name"
                otherwhise act like list getter
                """
                # act like a dictionary if key is str
                if type(args[0]) in [str, unicode]:
                    for element in self:
                        if isinstance(element, dict) and\
                                "name" in element and\
                                element["name"] == args[0]:
                            return element
                    return None
                # else act like a list by default
                return list.__getitem__(self, *args, **kwargs)

            def __contains__(self, key):
                """
                operator surcharge for in
                """
                result = False
                for element in self:
                    if isinstance(element, dict) and\
                            "name" in element:
                        if element["name"] == key:
                            result = True
                            break

                return result

            def keys(self):
                """
                Act like dict.keys() function
                """
                result = []
                for element in self:
                    if isinstance(element, dict) and\
                            "name" in element:
                        result.append(element["name"])
                return result

        def __parse_node(node, em_list, tag=""):
            """
            This private function parse a node from EM config parsing.

            :type node: object
            :param node: node of the xml document parsed by minidom.

            :param em_list: DictionaryList
            :param em_list: parameter used for comparison

            :param tag: str
            :param tag: tag to associate to this node.

            :rtype: tuple
            :return: Data stocked into a dictionnary and DictionaryList
            """
            dico = {}
            result = (dico, em_list)

            node_list = xpath.find('./*', node)
            if not node_list:
                name = xpath.find('attribute::name', node)
                if name:
                    attr_list = xpath.find('./attribute::*', node)
                    for attr in attr_list:
                        if attr.name == "name":
                            dico[str(attr.name)] = tag
                        else:
                            dico[str(attr.name)] = str(attr.value)

                    em_list.append(dico)
                    result = (dico, em_list)
            else:
                for node_item in node_list:
                    name = xpath.find('attribute::name', node_item)
                    if name:
                        if tag != "":
                            modified_tag = tag + "." + str(name[0].value)
                        else:
                            modified_tag = str(name[0].value)

                        result = __parse_node(node_item, em_list, modified_tag)

            return result

        def __parse_platform(platform_name, node_list, target_dict):
            """
            search for the right target in given platform and input node

            :type platform_name: String
            :param platform_name: platform name

            :type node_list: String
            :param node_list: The use case name

            :type target_dict: dict
            :param target_dict: the spefic key:value for this capabilities

            :rtype: minidom node object
            :return : right node witch match with targets, None otherwise
            """
            platform_name = str(platform_name).lower()
            for node in node_list:
                # check platform first
                xml_plaform = xpath.find('attribute::platform', node)
                platform_list = str(xml_plaform[0].value).split("|")
                platform_found = False
                # check if the platform name is known
                if xml_plaform:
                    for target_platform in platform_list:
                        target_platform = target_platform.lower().strip()
                        # if '*' is used as target name try to match the target name with the given platform name
                        if '*' in target_platform:
                            # use regular expression to evaluate the expression
                            target_platform = target_platform.replace("*", "(.*)") + "$"
                            match = re.match(target_platform, platform_name, re.IGNORECASE)
                            # double security , also compare result with platform name
                            if match and match.group() == platform_name:
                                platform_found = True
                                break
                        # else check that the name are strictly equal
                        elif platform_name == target_platform:
                            platform_found = True
                            break

                if platform_found:
                    input_targets = xpath.find('./Target', node)
                    target_validate = 0
                    expected_pass = len(input_targets)
                    message = ""
                    for target in input_targets:
                        target_name = xpath.find('attribute::input_name', target)
                        if target_name:
                            target_name = str(target_name[0].value)
                        else:
                            break

                        if target_name in target_dict:
                            # check fix value as str
                            input_value = xpath.find('attribute::equal', target)
                            usecase_target = str(target_dict[target_name]).strip()
                            # try to find 'starts_with' key
                            start_with_value = xpath.find('attribute::starts_with', target)

                            # If equal key is found
                            if input_value:
                                input_value = str(input_value[0].value).split("|")

                                for key in input_value:
                                    # ignore empty key
                                    key = key.strip()
                                    if key == "":
                                        continue
                                    # compare numeric value here
                                    if self.__is_numeric(key):
                                        key = float(key)
                                        if self.__is_numeric(usecase_target) and float(usecase_target) == key:
                                            target_validate += 1
                                            message += "\ntarget '%s' = '%s'(numeric) found in EM targets that match with your testcase configuration" % (
                                                target_name, key)
                                    # else compare str
                                    elif usecase_target.upper() == key.upper():
                                        target_validate += 1
                                        message += "\ntarget '%s' = '%s'(str) found in EM targets that match with your testcase configuration" % (
                                            target_name, key)
                            # If start_with key is found
                            elif start_with_value:
                                start_with_value = str(start_with_value[0].value).split("|")

                                for key in start_with_value:
                                    # ignore empty key
                                    key = key.strip()
                                    if key == "":
                                        continue
                                    # else compare str
                                    elif usecase_target.upper().startswith(key.upper()):
                                        target_validate += 1
                                        message += "\ntarget '%s' starts with '%s'(str) found in EM targets that match with your testcase configuration" % (
                                            target_name, key)
                            elif self.__is_numeric(usecase_target):
                                # else check rang value
                                usecase_target = float(usecase_target)

                                # else check rang value
                                inclusive_hi_lim = xpath.find('attribute::inclusive_hi_lim', target)
                                exclusive_hi_lim = xpath.find('attribute::exclusive_hi_lim', target)
                                inclusive_lo_lim = xpath.find('attribute::inclusive_lo_lim', target)
                                exclusive_lo_lim = xpath.find('attribute::exclusive_lo_lim', target)
                                high_lim_value = ""
                                low_lim_value = ""
                                lo_ok = None
                                hi_ok = None
                                # check value is <= inclusive_hi_lim
                                if inclusive_hi_lim:
                                    hi_ok = False
                                    inclusive_hi_lim = str(inclusive_hi_lim[0].value)
                                    if self.__is_numeric(inclusive_hi_lim):
                                        if usecase_target <= float(inclusive_hi_lim):
                                            hi_ok = True
                                            high_lim_value = inclusive_hi_lim
                                # else value is strictly < exclusive_hi_lim
                                elif exclusive_hi_lim:
                                    hi_ok = False
                                    exclusive_hi_lim = str(exclusive_hi_lim[0].value)
                                    if self.__is_numeric(exclusive_hi_lim):
                                        if usecase_target < float(exclusive_hi_lim):
                                            hi_ok = True
                                            high_lim_value = exclusive_hi_lim

                                # check value is >= inclusive_lo_lim
                                if inclusive_lo_lim:
                                    lo_ok = False
                                    inclusive_lo_lim = str(inclusive_lo_lim[0].value)
                                    if self.__is_numeric(inclusive_lo_lim):
                                        if usecase_target >= float(inclusive_lo_lim):
                                            lo_ok = True
                                            low_lim_value = inclusive_lo_lim
                                # check value is strictly > exclusive_lo_lim
                                elif exclusive_lo_lim:
                                    lo_ok = False
                                    exclusive_lo_lim = str(exclusive_lo_lim[0].value)
                                    if self.__is_numeric(exclusive_lo_lim):
                                        if usecase_target > float(exclusive_lo_lim):
                                            lo_ok = True
                                            low_lim_value = exclusive_lo_lim

                                if lo_ok and hi_ok:
                                    message += "\ntarget '%s': %s in range (%s, %s) found in EM targets that match with your testcase configuration" % (target_name,
                                                                                                                                                        usecase_target,
                                                                                                                                                        low_lim_value,
                                                                                                                                                        high_lim_value)
                                    target_validate += 1
                                elif lo_ok and hi_ok is None:
                                    message += "\ntarget '%s': %s >= to %s found in EM targets that match with your testcase configuration" % (target_name,
                                                                                                                                               usecase_target,
                                                                                                                                               low_lim_value)
                                    target_validate += 1
                                elif lo_ok is None and hi_ok:
                                    message += "\ntarget '%s': %s <= to %s found in EM targets that match with your testcase configuration" % (target_name,
                                                                                                                                               usecase_target,
                                                                                                                                               high_lim_value)
                                    target_validate += 1

                    if 0 < expected_pass == target_validate:
                        LOGGER_TEST_SCRIPT.debug(message)
                        return node
                    else:
                        message = ""

        def __search_default(platform_name, node_list):  # pylint: disable=W0621
            """
            private function, search for default target
            looking for input which does not contain any <Target> tag as
            long as we are in the good platform

            :type platform_name: String
            :param platform_name: platform name

            :type node_list: String
            :param node_list: The use case name

            :rtype: minidom node object
            :return : right node witch match with targets, None otherwise
            """
            for node in node_list:
                # check platform first
                xml_plaform = xpath.find('attribute::platform', node)
                platform_list = str(xml_plaform[0].value).split("|")
                platform_found = False
                # check if the platform name is known
                if xml_plaform:
                    for target_platform in platform_list:
                        target_platform = target_platform.lower().strip()
                        # if '*' is used as target name try to match the target name with the given platform name
                        if '*' in target_platform:
                            # use regular expression to evaluate the expression
                            target_platform = target_platform.replace("*", "(.*)") + "$"
                            match = re.match(target_platform, platform_name, re.IGNORECASE)
                            # double security , also compare result with platform name
                            if match and match.group() == platform_name:
                                platform_found = True
                                break
                        # else check that the name are strictly equal
                        elif platform_name == target_platform:
                            platform_found = True
                            break

                if platform_found:
                    input_targets = xpath.find('./Target', node)
                    if not input_targets:
                        LOGGER_TEST_SCRIPT.debug("default target declaration found")
                        return node

        em_list_paramters = __DictionaryList()
        try:
            LOGGER_TEST_SCRIPT.info("Parsing Energy management file to find appropriate target")
            # Check if scenario exists in DG09 configuration
            uc_node = xpath.find('/UseCases/UseCase[@id = $UCname]',
                                 self._document,
                                 UCname=ucase_name)

            error_msg = None
            if not uc_node:
                error_msg = "UseCase %s does not exist" % ucase_name
            elif len(uc_node) > 1:
                error_msg = "duplicate usecase name '%s' found in your energy management target file " % ucase_name
            elif not isinstance(input_dict, dict):
                error_msg = "passed target must be a non empty dict"
            if error_msg is not None:
                LOGGER_TEST_SCRIPT.error(error_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

            # Get parameter for given input properties
            node_list = xpath.find('./Inputs/Input', uc_node[0])
            default_platform = "default"
            # search for the value and platform that match with setting
            platform_node = None
            last_platform = ""
            for platform_name in [input_platform, default_platform]:  # pylint: disable=W0621
                platform_node = __parse_platform(platform_name,
                                                 node_list, input_dict)
                if platform_node is not None:
                    last_platform = platform_name
                    break
            # if failed to find a specific target, search default one
            if platform_node is None:
                for platform_name in [input_platform, default_platform]:
                    platform_node = __search_default(platform_name,
                                                     node_list)
                    if platform_node is not None:
                        last_platform = platform_name
                        break

            if platform_node is not None:
                # Get parameter for given input properties
                node_list = xpath.find('./Parameters/*', platform_node)

                for node in node_list:
                    name = xpath.find('attribute::name', node)
                    if name:
                        name = str(name[0].value)
                        em_list_paramters = __parse_node(node, em_list_paramters, name)[1]

                        # In case MSIC register
                        temp_dict = {}
                        option = xpath.find('attribute::scheduled_time', node)
                        if option:
                            option = str(option[0].value)
                            temp_dict.update({"scheduled_time": str(option)})
                        # In case temperature
                        option = xpath.find('attribute::temperature', node)
                        if option:
                            option = str(option[0].value)
                            temp_dict.update({"temperature": str(option)})

                        if temp_dict != {}:
                            temp_dict.update({"name": name})
                            em_list_paramters.append(temp_dict)

                LOGGER_TEST_SCRIPT.info("loading Specific testcase capability definitions for %s platform" % last_platform)
            else:
                # error message if no node found
                msg = "Specific testcase capability definitions for platform %s not found" % input_platform
                LOGGER_TEST_SCRIPT.warning(msg)

        except Exception as exception:  # pylint: disable=W0703
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, str(exception))

        if len(em_list_paramters.keys()) == 0:
            msg = "parse energy management targets failed, "\
                "check if %s with specific targets: %s for '%s' or '%s' platform exist in your EM configuration file" % (
                    ucase_name, str(input_dict), input_platform, default_platform)
            LOGGER_TEST_SCRIPT.error(msg)
            LOGGER_TEST_SCRIPT.warning("Test will be run without target")

        # Return all parameter's values
        return em_list_paramters

    # sub function to avoid long line code
    def __is_numeric(self, text):
        """

        :type text: object
        :param text: a text that maybe a numeric in str format

        :rtype: boolean
        :return : true if it is numeric, else otherwise

        """
        try:
            float(str(text))
            return True
        except ValueError:
            return False

    def parse_em_ocv_targets(self, platform_name, temperature):  # pylint: disable=W0621
        """
        Get OCV limit parameter in order to compute the limit from the SOC.
        The limits are define for a specific target and at a specific temperature
        Note that temperature can be describe as a range of temperature depending of how you write it.

        :type platform_name: String
        :param platform_name: platform for this target

        :type temperature: int
        :param temperature: temperature for this target, if temperature is None ,
                            room_temperature configuration will be used

        :rtype: dict
        :return : all data read from the Energy_Management_OCV_limit.xml configuration file.
        """
        # if temperature set to None, use default target value
        if temperature is None:
            temperature = "room_temperature"
        # Check if platform exist in file Energy_Management_OCV_limit.xml
        temp_tag_found = False
        good_node = None
        platform_node = xpath.find('/PlatformList/Platform',
                                   self._document)

        for p_node in platform_node:
            # get platform name declared in xml
            raw_platform = xpath.find('attribute::id', p_node)
            if raw_platform:
                raw_platform = str(raw_platform[0].value)
            else:
                continue

            target = raw_platform.lower().split("|")
            target = map(string.strip, target)

            # go to next iteration if your platform is not in xml target
            if platform_name.lower() not in target:
                continue
            # get temperature node for this platform
            temperature_node = xpath.find("./Target", p_node)
            # Check if temperature exist in file Energy_Management_OCV_limit.xml

            for node in temperature_node:
                # check fixed temperature first
                temp_value = xpath.find('attribute::temperature', node)
                if temp_value:
                    # temp value have high priority compare with range
                    temp_value = str(temp_value[0].value)
                    if (isinstance(temperature, str) and temperature == temp_value) or \
                            self.__is_numeric(temp_value) and temperature == float(temp_value):
                        temp_tag_found = True
                        good_node = node
                        break

                if not isinstance(temperature, str):
                    # then check if range is used
                    hi_temp = xpath.find('attribute::high_temperature', node)
                    lo_temp = xpath.find('attribute::low_temperature', node)

                    if hi_temp != [] or lo_temp != []:
                        # get high lim
                        if hi_temp:
                            hi_temp = str(hi_temp[0].value)
                            hi_temp = float(hi_temp)
                        else:
                            # put a very high value
                            hi_temp = 100000
                        # get low lim
                        if lo_temp:
                            lo_temp = str(lo_temp[0].value)
                            lo_temp = float(lo_temp)
                        else:
                            # put a very low value
                            lo_temp = -100000

                        if lo_temp <= temperature <= hi_temp:
                            temp_tag_found = True
                            good_node = node
                            break

        # get targets
        dico = {}
        if temp_tag_found:
            # Get parameter for given input properties
            node_list = xpath.find('./Parameters/*', good_node)
            for node in node_list:
                # get soc limits
                lo_soc = xpath.find('attribute::lo_soc', node)
                hi_soc = xpath.find('attribute::hi_soc', node)
                if [] in [lo_soc, hi_soc]:
                    # if one of these parameter is missing go to next node
                    continue
                lo_soc = int(lo_soc[0].value)
                hi_soc = int(hi_soc[0].value)
                # get sub key
                sub_dico = {}
                for tag in ["lo_slope", "lo_intercept",
                            "hi_slope", "hi_intercept"]:
                    value = None
                    # if tag exist, get is value
                    raw_value = xpath.find('attribute::' + tag, node)
                    if raw_value:
                        value = float(raw_value[0].value)
                    sub_dico.update({tag: value})

                # init dico
                dico.update({(lo_soc, hi_soc): sub_dico})
            msg = "OCV target limits for platform %s at %s Degree Celsius loaded" % (platform_name, temperature)
            LOGGER_TEST_SCRIPT.info(msg)
        else:
            # warn user  that target is not found
            msg = "Specific OCV target limits for platform %s at %s Degree Celsius not found" % (
                platform_name, temperature)
            LOGGER_TEST_SCRIPT.warning(msg)

        return dico

    def parse_wifi_direct_targets(self, device_model, protocole="TCP", wifi_bandwidth="20", aliases=None):
        """
        This function parses the Throughput Wifi Direct Targets
        to get WiFi Direct uplink and downlink failure and target values in Kbps

        :type device_model: String
        :param device_model: Name of the device model under test
        :type protocole: String
        :param protocole: The protocole used for the test "TCP" or "UDP"
        :type wifi_bandwidth: String
        :param wifi_bandwidth: The bandwidth used for the test "20", "40" or "80"

        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: MeasureThroughputTargets
        :return : Wifi Targets with Target and Failure data.
        """
        protocole = protocole.upper()
        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # Compute wifi bandwidth
        if wifi_bandwidth not in ["20", "40", "80"]:
            cmd = "Bandwidth %s not supported in throughput targets" % wifi_bandwidth
            LOGGER_TEST_SCRIPT.error(cmd)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, cmd)

        # look for the device in the wifi_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in wifi_targets" \
                  % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # no alias, find this device
        try:
            target_wifi_direct_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            target_wifi_direct_expression += "/ThroughputTypes/ThroughputType[@Name='TARGET']"
            target_wifi_direct_expression += "/Categories/Category[@Bandwidth='%s' and " % wifi_bandwidth
            target_wifi_direct_expression += "@Direction='%s' and "
            target_wifi_direct_expression += "@Protocole='%s']" % protocole

            failure_wifi_direct_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            failure_wifi_direct_expression += "/ThroughputTypes/ThroughputType[@Name='FAILURE']"
            failure_wifi_direct_expression += "/Categories/Category[@Bandwidth='%s' and " % wifi_bandwidth
            failure_wifi_direct_expression += "@Direction='%s' and "
            failure_wifi_direct_expression += "@Protocole='%s']" % protocole

            failure_dl_wifi = self._etree_document.xpath(failure_wifi_direct_expression % "DL")
            failure_dl_wifi = failure_dl_wifi[0]

            failure_ul_wifi = self._etree_document.xpath(failure_wifi_direct_expression % "UL")
            failure_ul_wifi = failure_ul_wifi[0]

            target_dl_wifi = self._etree_document.xpath(target_wifi_direct_expression % "DL")
            target_dl_wifi = target_dl_wifi[0]

            target_ul_wifi = self._etree_document.xpath(target_wifi_direct_expression % "UL")
            target_ul_wifi = target_ul_wifi[0]

            wifi_direct_targets = MeasureThroughputTargets()
            wifi_direct_targets.dl_failure_value = float(
                self._get_attribute_value(failure_dl_wifi, "Throughput"))
            wifi_direct_targets.dl_target_value = float(
                self._get_attribute_value(target_dl_wifi, "Throughput"))
            wifi_direct_targets.ul_failure_value = float(
                self._get_attribute_value(failure_ul_wifi, "Throughput"))
            wifi_direct_targets.ul_target_value = float(
                self._get_attribute_value(target_ul_wifi, "Throughput"))

            wifi_direct_targets.ul_units = "Kbps"
            wifi_direct_targets.dl_units = "Kbps"

            return wifi_direct_targets
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # look for an alias
            alias_expression = "/DeviceModels/DeviceModel[@Name='%s']/Alias" % device_model
            result = self._etree_document.xpath(alias_expression)
            if len(result) != 0:
                # alias found, add device model in already seen devices
                aliases.append(device_model)
                base_model = self._get_attribute_value(result[0], "Name")
                if base_model in aliases:
                    # error: malformed xml causing infinite recursion
                    msg = "DeviceModel %s is malformed in wifi_direct_targets" \
                          % device_model
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                # retry with this device
                return self.parse_wifi_direct_targets(base_model, protocole, wifi_bandwidth, aliases)
            else:
                LOGGER_TEST_SCRIPT.error(format_exception_info())
                raise

    def parse_wifi_mr_targets(self, device_model, standard_frequency, bandwidth, sta_direction, sta_protocole,
                              p2p_direction, p2p_protocole, aliases=None):
        """
        This function parses the Throughput Wifi Multi Role Targets
        to get WiFi MR uplink and downlink failure and target values in Kbps

        :type device_model: String
        :param device_model: Name of the device model under test
        :type standard_frequency: String
        :param standard_frequency: WiFi standard used. 2.4G or 5G
        :type bandwidth: String
        :param bandwidth: The bandwidth used for the test "20", "40" or "80"
        :type sta_direction: String
        :param sta_direction: Direction of transfer for STA : "up" or "down"
        :type sta_protocole: String
        :param sta_protocole: The protocole used with STA transfer for the test "TCP" or "UDP"
        :type p2p_direction: String
        :param p2p_direction: Direction of transfer for P2P : "up" or "down"
        :type p2p_protocole: String
        :param p2p_protocole: The protocole used with P2P transfer for the test "TCP" or "UDP"

        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: MeasureThroughputTargets, MeasureThroughputTargets
        :return : Wifi Targets with Target and Failure data for STA, Wifi Targets with Target and Failure data for P2P
        """
        sta_protocole = sta_protocole.upper()
        p2p_protocole = p2p_protocole.upper()
        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # Compute directions
        if sta_direction.upper() in ["DOWN", "DL"]:
            sta_direction = "DL"
        elif sta_direction.upper() in ["UP", "UL"]:
            sta_direction = "UL"
        else:
            msg = "Unsupported STA direction %s" % sta_direction
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if p2p_direction.lower() in ["down", "dl"]:
            p2p_direction = "DL"
        elif p2p_direction.lower() in ["up", "ul"]:
            p2p_direction = "UL"
        else:
            msg = "Unsupported P2P direction %s" % p2p_direction
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Compute standard frequency
        if standard_frequency not in ["2.4G", "5G"]:
            msg = "Standard frequency %s is not supported in throughput targets" % standard_frequency
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Compute wifi bandwidth
        if bandwidth not in ["20", "40", "80"]:
            cmd = "Bandwidth %s not supported in throughput targets" % bandwidth
            LOGGER_TEST_SCRIPT.error(cmd)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, cmd)

        # look for the device in the wifi_mr_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in wifi_mr_targets" \
                  % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # no alias, find this device
        try:
            target_wifi_mr_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            target_wifi_mr_expression += "/ThroughputTypes/ThroughputType[@Name='TARGET']"
            target_wifi_mr_expression += "/Categories/Category[@StandardFrequency='%s' and " % standard_frequency
            target_wifi_mr_expression += "@Bandwidth='%s' and " % bandwidth
            target_wifi_mr_expression += "@STADirection='%s' and " % sta_direction
            target_wifi_mr_expression += "@STAProtocole='%s' and " % sta_protocole
            target_wifi_mr_expression += "@P2PDirection='%s' and " % p2p_direction
            target_wifi_mr_expression += "@P2PProtocole='%s']" % p2p_protocole

            failure_wifi_mr_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            failure_wifi_mr_expression += "/ThroughputTypes/ThroughputType[@Name='FAILURE']"
            failure_wifi_mr_expression += "/Categories/Category[@StandardFrequency='%s' and " % standard_frequency
            failure_wifi_mr_expression += "@Bandwidth='%s' and " % bandwidth
            failure_wifi_mr_expression += "@STADirection='%s' and " % sta_direction
            failure_wifi_mr_expression += "@STAProtocole='%s' and " % sta_protocole
            failure_wifi_mr_expression += "@P2PDirection='%s' and " % p2p_direction
            failure_wifi_mr_expression += "@P2PProtocole='%s']" % p2p_protocole

            target_mr_wifi = self._etree_document.xpath(target_wifi_mr_expression)
            target_mr_wifi = target_mr_wifi[0]

            failure_mr_wifi = self._etree_document.xpath(failure_wifi_mr_expression)
            failure_mr_wifi = failure_mr_wifi[0]

            wifi_sta_targets = MeasureThroughputTargets()
            if sta_direction == "UL":
                wifi_sta_targets.ul_failure.set(float(self._get_attribute_value(failure_mr_wifi, "STAThroughput")),
                                                ThroughputMeasure.KPS_UNIT)
                wifi_sta_targets.ul_target.set(float(self._get_attribute_value(target_mr_wifi, "STAThroughput")),
                                               ThroughputMeasure.KPS_UNIT)
            else:
                wifi_sta_targets.dl_failure.set(float(self._get_attribute_value(failure_mr_wifi, "STAThroughput")),
                                                ThroughputMeasure.KPS_UNIT)
                wifi_sta_targets.dl_target.set(float(self._get_attribute_value(target_mr_wifi, "STAThroughput")),
                                               ThroughputMeasure.KPS_UNIT)
            wifi_p2p_targets = MeasureThroughputTargets()
            if p2p_direction == "UL":
                wifi_p2p_targets.ul_failure.set(float(self._get_attribute_value(failure_mr_wifi, "P2PThroughput")),
                                                ThroughputMeasure.KPS_UNIT)
                wifi_p2p_targets.ul_target.set(float(self._get_attribute_value(target_mr_wifi, "P2PThroughput")),
                                               ThroughputMeasure.KPS_UNIT)
            else:
                wifi_p2p_targets.dl_failure.set(float(self._get_attribute_value(failure_mr_wifi, "P2PThroughput")),
                                                ThroughputMeasure.KPS_UNIT)
                wifi_p2p_targets.dl_target.set(float(self._get_attribute_value(target_mr_wifi, "P2PThroughput")),
                                               ThroughputMeasure.KPS_UNIT)

            return wifi_sta_targets, wifi_p2p_targets
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # look for an alias
            alias_expression = "/DeviceModels/DeviceModel[@Name='%s']/Alias" % device_model
            result = self._etree_document.xpath(alias_expression)
            if len(result) != 0:
                # alias found, add device model in already seen devices
                aliases.append(device_model)
                base_model = self._get_attribute_value(result[0], "Name")
                if base_model in aliases:
                    # error: malformed xml causing infinite recursion
                    msg = "DeviceModel %s is malformed in wifi_mr_targets" \
                          % device_model
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                # retry with this device
                return self.parse_wifi_mr_targets(base_model, standard_frequency, bandwidth, sta_direction,
                                                  sta_protocole, p2p_direction, p2p_protocole, aliases)
            else:
                LOGGER_TEST_SCRIPT.error(format_exception_info())
                raise

    def parse_wifi_targets(self,
                           device_model,
                           wifi_standard,
                           wifi_security,
                           protocole="TCP",
                           wifi_bandwidth="20",
                           aliases=None):
        """
        This function parses the Throughput Wifi Targets
        to get WIFI uplink and downlink failure and target values
        in Kbps

        :type device_model: String
        :param device_model: Name of the device model under test
        :type wifi_standard: String
        :param wifi_standard: the wifi standard used by the AP.
                The possible value are:
                "a", "b", "g", "an", "bg", "gn", "bgn", "n2.4G", "n5G", "ac"
        :type wifi_security: String
        :param wifi_security: The wifi security used for the test "OPEN",
                                "WEP64","WEP128","WPA-PSK-TKIP",
                                "WPA2-PSK-AES", "EAP-WPA" or "EAP-WPA2"
        :type protocole: String
        :param protocole: The protocole used for the test "TCP" or "UDP"
        :type wifi_bandwidth: String
        :param wifi_bandwidth: The bandwidth used for the test "20", "40" or "80"

        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: MeasureThroughputTargets
        :return : Wifi Targets with Target and Failure data.
        """
        protocole = protocole.upper()
        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # Compute wifi standard:
        wifi_standard = str(wifi_standard).lower()
        if wifi_standard in ["bg", "gb", "g"]:
            wifi_standard = "bg"
        elif wifi_standard in ["bgn", "gbn", "bng", "gnb",
                               "nbg", "ngb", "gn", "ng"]:
            wifi_standard = "bgn"
        elif wifi_standard in ["n", "n2.4g"]:
            wifi_standard = "n2.4G"
        elif wifi_standard in ["an", "na"]:
            wifi_standard = "an"
        elif wifi_standard == "n5g":
            wifi_standard = "n5G"
        elif wifi_standard in ["a", "b", "ac"]:
            pass
        else:
            cmd = "Invalid WiFi standard parameter: " + wifi_standard
            LOGGER_TEST_SCRIPT.error(cmd)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, cmd)

        # Compute wifi security
        wifi_security = str(wifi_security).upper()
        if wifi_security in ["OPEN", "NONE"]:
            wifi_security = "OPEN"
        elif "WEP" in wifi_security:
            wifi_security = "WEP"
        else:
            # Compute WPA security Personal or Enterprise
            secu = None
            if "WPA2" in wifi_security:
                secu = "WPA2"
            elif "WPA" in wifi_security:
                secu = "WPA"
            if secu is None:
                cmd = "invalid wifi security parameter: " + wifi_security
                LOGGER_TEST_SCRIPT.error(cmd)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, cmd)
            # Compute WPA ciphering mode
            if "TKIP" in wifi_security:
                secu += "-TKIP"
            elif "AES" in wifi_security:
                secu += "-AES"
            else:
                # Default WPA ciphering mode
                if secu == "WPA":
                    secu = "WPA-TKIP"
                else:
                    secu = "WPA2-AES"
            if "WPA-WPA2" in wifi_security:
                secu = "WPA-WPA2"
            wifi_security = secu

        # Compute wifi bandwidth
        if wifi_bandwidth not in ["20", "40", "80"]:
            cmd = "Bandwidth %s not supported in throughput targets" % wifi_bandwidth
            LOGGER_TEST_SCRIPT.error(cmd)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, cmd)

        # look for the device in the wifi_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in wifi_targets" \
                % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # no alias, find this device
        try:
            target_wifi_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            target_wifi_expression += "/ThroughputTypes/ThroughputType[@Name='TARGET']"
            target_wifi_expression += "/Categories/Category[@Standard='%s' and " % wifi_standard
            target_wifi_expression += "@Bandwidth='%s' and " % wifi_bandwidth
            target_wifi_expression += "@Direction='%s' and "
            target_wifi_expression += "@Protocole='%s']" % protocole

            failure_wifi_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            failure_wifi_expression += "/ThroughputTypes/ThroughputType[@Name='FAILURE']"
            failure_wifi_expression += "/SecuritiyTypes/Security[@Name='%s']" % wifi_security
            failure_wifi_expression += "/Categories/Category[@Standard='%s' and " % wifi_standard
            failure_wifi_expression += "@Bandwidth='%s' and " % wifi_bandwidth
            failure_wifi_expression += "@Direction='%s' and "
            failure_wifi_expression += "@Protocole='%s']" % protocole

            failure_dl_wifi = self._etree_document.xpath(failure_wifi_expression % "DL")
            failure_dl_wifi = failure_dl_wifi[0]

            failure_ul_wifi = self._etree_document.xpath(failure_wifi_expression % "UL")
            failure_ul_wifi = failure_ul_wifi[0]

            target_dl_wifi = self._etree_document.xpath(target_wifi_expression % "DL")
            target_dl_wifi = target_dl_wifi[0]

            target_ul_wifi = self._etree_document.xpath(target_wifi_expression % "UL")
            target_ul_wifi = target_ul_wifi[0]

            wifi_targets = MeasureThroughputTargets()
            wifi_targets.dl_failure.set(float(self._get_attribute_value(failure_dl_wifi, "Throughput")),
                                        ThroughputMeasure.KBPS_UNIT)
            wifi_targets.dl_target.set(float(self._get_attribute_value(target_dl_wifi, "Throughput")),
                                       ThroughputMeasure.KBPS_UNIT)
            wifi_targets.ul_failure.set(float(self._get_attribute_value(failure_ul_wifi, "Throughput")),
                                        ThroughputMeasure.KBPS_UNIT)
            wifi_targets.ul_target.set(float(self._get_attribute_value(target_ul_wifi, "Throughput")),
                                       ThroughputMeasure.KBPS_UNIT)
            return wifi_targets
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # look for an alias
            alias_expression = "/DeviceModels/DeviceModel[@Name='%s']/Alias" % device_model
            result = self._etree_document.xpath(alias_expression)
            if len(result) != 0:
                # alias found, add device model in already seen devices
                aliases.append(device_model)
                base_model = self._get_attribute_value(result[0], "Name")
                if base_model in aliases:
                    # error: malformed xml causing infinite recursion
                    msg = "DeviceModel %s is malformed in wifi_targets" \
                        % device_model
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                # retry with this device
                return self.parse_wifi_targets(base_model,
                                               wifi_standard,
                                               wifi_security,
                                               protocole,
                                               wifi_bandwidth,
                                               aliases)
            else:
                LOGGER_TEST_SCRIPT.error(format_exception_info())
                raise

    def parse_wifi_connection_targets(self, device_model, aliases=None):
        """
        This function parses the Connection Wifi Targets to get WIFI connection time failure and target values in second

        :type device_model: String
        :param device_model: Name of the device model under test

        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: MeasureConnectionTargets
        :return : Wifi Connection Targets with Target and Failure data.
        """
        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # look for the device in the wifi_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in wifi_connection_targets" % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # no alias, find this device
        try:
            target_wifi_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            target_wifi_expression += "/ConnectionTimes/ConnectionTime[@Name='TARGET']"

            failure_wifi_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            failure_wifi_expression += "/ConnectionTimes/ConnectionTime[@Name='FAILURE']"

            failure_connection_wifi = self._etree_document.xpath(failure_wifi_expression)
            failure_connection_wifi = failure_connection_wifi[0]

            target_connection_wifi = self._etree_document.xpath(target_wifi_expression)
            target_connection_wifi = target_connection_wifi[0]

            wifi_connection_targets = MeasureConnectionTargets()
            wifi_connection_targets.connection_failure_value = float(self._get_attribute_value(failure_connection_wifi, "Value"))
            wifi_connection_targets.connection_target_value = float(self._get_attribute_value(target_connection_wifi, "Value"))
            wifi_connection_targets.connection_unit = MeasureConnectionTargets.SEC_UNIT[0]

            return wifi_connection_targets
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # look for an alias
            alias_expression = "/DeviceModels/DeviceModel[@Name='%s']/Alias" % device_model
            result = self._etree_document.xpath(alias_expression)
            if len(result) != 0:
                # alias found, add device model in already seen devices
                aliases.append(device_model)
                base_model = self._get_attribute_value(result[0], "Name")
                if base_model in aliases:
                    # error: malformed xml causing infinite recursion
                    msg = "DeviceModel %s is malformed in wifi_connection_targets" \
                        % device_model
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                # retry with this device
                return self.parse_wifi_connection_targets(base_model, aliases)
            else:
                LOGGER_TEST_SCRIPT.error(format_exception_info())
                raise

    def parse_bt_targets(self, device_model, protocol="OPP", aliases=None):
        """
        This function parses the Throughput BT Targets
        to get BT uplink and downlink target values in KBps

        :type device_model: String
        :param device_model: Name of the device model under test
        :type protocol: String
        :param protocol: The protocol used for the test "OPP"

        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: MeasureThroughputTargets
        :return : BT Targets with Target data.
        """
        protocol = protocol.upper()
        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # look for the device in in wifi_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in BT_Throughput_Targets.xml" % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # look for the device in the wifi_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in bt_targets" % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # no alias, find this device
        try:
            target_bt_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            target_bt_expression += "/ThroughputTypes/ThroughputType[@Name='TARGET']"
            target_bt_expression += "/Categories/Category[@Direction='%s' and "
            target_bt_expression += "@Protocol='%s']" % protocol

            target_dl_bt = self._etree_document.xpath(target_bt_expression % "DL")
            target_dl_bt = target_dl_bt[0]

            target_ul_bt = self._etree_document.xpath(target_bt_expression % "UL")
            target_ul_bt = target_ul_bt[0]

            failure_bt_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            failure_bt_expression += "/ThroughputTypes/ThroughputType[@Name='FAILURE']"
            failure_bt_expression += "/Categories/Category[@Direction='%s' and "
            failure_bt_expression += "@Protocol='%s']" % protocol

            failure_dl_bt = self._etree_document.xpath(failure_bt_expression % "DL")
            failure_dl_bt = failure_dl_bt[0]

            failure_ul_bt = self._etree_document.xpath(failure_bt_expression % "UL")
            failure_ul_bt = failure_ul_bt[0]

            bt_targets = MeasureThroughputTargets()
            bt_targets.dl_target.set(float(self._get_attribute_value(target_dl_bt, "Throughput")),
                                     ThroughputMeasure.KBPS_UNIT)
            bt_targets.ul_target.set(float(self._get_attribute_value(target_ul_bt, "Throughput")),
                                     ThroughputMeasure.KBPS_UNIT)
            bt_targets.dl_failure.set(float(self._get_attribute_value(failure_dl_bt, "Throughput")),
                                     ThroughputMeasure.KBPS_UNIT)
            bt_targets.ul_failure.set(float(self._get_attribute_value(failure_ul_bt, "Throughput")),
                                     ThroughputMeasure.KBPS_UNIT)
            return bt_targets
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            # look for an alias
            alias_expression = "/DeviceModels/DeviceModel[@Name='%s']/Alias" % device_model
            result = self._etree_document.xpath(alias_expression)
            if len(result) != 0:
                # alias found, add device model in already seen devices
                aliases.append(device_model)
                base_model = self._get_attribute_value(result[0], "Name")
                if base_model in aliases:
                    # error: malformed xml causing infinite recursion
                    msg = "DeviceModel %s is malformed in bt_targets" \
                          % device_model
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
                # retry with this device
                return self.parse_bt_targets(base_model, protocol, aliases)
            else:
                LOGGER_TEST_SCRIPT.error(format_exception_info())
                raise

    def parse_wifi_sleep_policies(self,
                                  device_model,
                                  wifi_connected,
                                  display_state,
                                  keep_wifi_on_during_sleep,
                                  usb_pluggedin,
                                  remembered_ssid_list_empty,
                                  aliases=None):
        """
        This function parses the Wifi_Sleep_Policies.xml file. This file is
        used to know the DUT WiFi sleep behavior depending on input parameters.
        If there is no XML TAG that correspond to the input paramters, it will
        find a more generic initial state in the XML file.

        :type device_model: String
        :param device_model: Name of the device model under test
        :type wifi_connected: Boolean or 'true'/'false' string
        :param wifi_connected: Is DUT wifi connected to an Access Point?
        :type display_state: string
        :param display_state: Is the display "off",
                        or on outside the WiFi menu "on_out_wifi_menu",
                        or on inside the WiFi menu "on_in_wifi_menu"?
        :type keep_wifi_on_during_sleep: string
        :param keep_wifi_on_during_sleep: User wifi sleep policiy set.
                    possible values: "always", "only_when_plugged_in", "never"
        :type usb_pluggedin: boolean or 'true'/'false' string
        :param usb_pluggedin: Is USB plugged in DUT?
        :type remembered_ssid_list_empty: boolean or 'true'/'false' string
        :param remembered_ssid_list_empty: When not connected,
                        is there any known networks?
        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: list of 4 int and 1 string
        :return: information about the behavior the DUT should have:
            - Scan period in seconds
                (0 means no scan period, -1 means periodic scan)
            - Number of minutes to turn OFF wifi (0: WiFi will never turn off)
            - Recommanded test duration in seconds (0: no recommanded value)
            - A short description of the test case or the test case ID itself
        """
        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # look for the device in in Wifi_Sleep_Policies
        dm_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        dm_node = self._etree_document.xpath(dm_expression)
        if dm_node is None or len(dm_node) == 0:
            msg = "DeviceModel %s does not exist in Wifi_Sleep_Policies" \
                % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # look for an alias
        alias_node = dm_node[0].xpath("Alias")
        if len(alias_node) != 0:
            # alias found, add device model in already seen devices
            aliases.append(device_model)
            base_model = self._get_attribute_value(alias_node[0], "Name")
            if base_model in aliases:
                # error: malformed xml causing infinite recursion
                msg = "DeviceModel %s is malformed in Wifi_Sleep_Policies" \
                    % device_model
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            # retry with this device
            return self.parse_wifi_sleep_policies(base_model,
                                                  wifi_connected,
                                                  display_state,
                                                  keep_wifi_on_during_sleep,
                                                  usb_pluggedin,
                                                  remembered_ssid_list_empty,
                                                  aliases)

        # check input parameters
        if str(wifi_connected).lower() not in ["true", "false"]:
            msg = "wifi_connected is not a boolean: %s" % str(wifi_connected)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        wifi_connected = str(wifi_connected).lower()
        if str(usb_pluggedin).lower() not in ["true", "false"]:
            msg = "usb_pluggedin is not a boolean: %s" % str(usb_pluggedin)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        usb_pluggedin = str(usb_pluggedin).lower()
        if str(remembered_ssid_list_empty).lower() not in ["true", "false"]:
            msg = "remembered_ssid_list_empty is not a boolean %s" % \
                str(remembered_ssid_list_empty)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        remembered_ssid_list_empty = str(remembered_ssid_list_empty).lower()
        if str(display_state).lower() not in ["off",
                                              "on_out_wifi_menu",
                                              "on_in_wifi_menu"]:
            msg = "display_state value isn't expected: %s" % str(display_state)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        display_state = str(display_state).lower()
        if str(keep_wifi_on_during_sleep).lower() not in ["always",
                                                          "only_when_plugged_in",
                                                          "never"]:
            msg = "keep_wifi_on_during_sleep value isn't expected: %s" % \
                str(keep_wifi_on_during_sleep)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        keep_wifi_on_during_sleep = str(keep_wifi_on_during_sleep).lower()

        # Initialize the output value
        scan = 0
        wifioff = 0
        test_duration = 0
        description = ""

        # no alias, find this device
        try:
            behavior_node_found = \
                self.__search_wifi_policy_generic_node(dm_node[0],
                                                       wifi_connected,
                                                       display_state,
                                                       keep_wifi_on_during_sleep,
                                                       usb_pluggedin,
                                                       remembered_ssid_list_empty)

            if behavior_node_found is None or len(behavior_node_found) == 0:
                msg = "No Sleep policy found in Wifi_Sleep_Policies.xml file"
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

            behavior_node_found = behavior_node_found[0]

            # Search for Scan output value
            scan_node = behavior_node_found.xpath("Scan")
            if scan_node is not None and len(scan_node) > 0:
                scan_read = scan_node[0].text
                if str(scan_read).upper() == "PERIODIC":
                    scan = -1
                elif str(scan_read).isdigit():
                    scan = int(scan_read)

            # Search for Wifi OFF output value
            wifioff_node = behavior_node_found.xpath("WifiOff")
            if wifioff_node is not None and len(wifioff_node) > 0:
                wifioff_read = wifioff_node[0].text
                if str(wifioff_read).isdigit():
                    wifioff = int(wifioff_read)

            # Search for Test Duration output value
            duration_node = behavior_node_found.xpath("TestDuration")
            if duration_node is not None and len(duration_node) > 0:
                test_duration_read = duration_node[0].text
                if str(test_duration_read).isdigit():
                    test_duration = int(test_duration_read)

            # Search for Description value
            description_node = behavior_node_found.xpath("Description")
            if description_node is not None and len(description_node) > 0:
                description = description_node[0].text

        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

        return [scan, wifioff, test_duration, description]

    def parse_cws_airplanemode_cycle_effect(self):
        """
        Parses the CWS_AirPlaneMode_Cycle_Effect.xml file.

        returns a dictionary like:
        {"WIFI": {"incompatibilities": ("HOTSPOT_WIFI"),
                  "state_after_apm_cycle": "ON"}

        :rtype: dict of dicts
        :return: a dictionnary of properties for each cws feature.
            each property is a dictionnary composed of 2 elements:
                - incompatibilities, which is a list of strings
                - state_after_apm_cycle: which is a String ON of OFF
        """
        output_db = dict()

        expression = "/features/feature"
        fs_node = self._etree_document.xpath(expression)

        for node in fs_node:
            feature = self._get_attribute_value(node, "name")
            if feature and feature not in output_db:
                i_node = node.xpath(self.APM_INCOMPATIBILITIES)
                s_node = node.xpath(self.APM_STATE_AFTER_APM_CYCLE)

                if i_node[0].text:
                    incompatibilities = i_node[0].text.split(',')
                else:
                    incompatibilities = []
                state_after_apm_cycle = str(s_node[0].text).upper()

                output_db[feature] = {self.APM_INCOMPATIBILITIES: incompatibilities,
                                      self.APM_STATE_AFTER_APM_CYCLE: state_after_apm_cycle}

        return output_db


class TelephonyConfigsParser(ConfigsParser):
    """
    This class implements parser for telephony throughput files in _Configs.
    It has been created to allow parsing of xml file in Use Case scripts,
    which cannot be simply instantiated with current implementation of FileParsingManager.
    FileParsingManager is used in CampaignEngine and
    need to have a list of instance arguments
    (bench_config, Device_Catalog..) that we do not necessary.
    """

#-------------------- Parsing throughput targets -----------------------------------------
    def _get_multislot_values(self, multislot):
        """
        Parses the multislot str to extract the Downlink and Uplink
        multislot value.

        :type multislot: str
        :param multislot: Multislot str to parse.

        :rtype: list
        :return: the downlink and uplink values: ("DL"<DL_Value, "UL"<UL_Value>)
        """
        multislot_values = {}
        multislot_values["DL"] = multislot[1]
        multislot_values["UL"] = multislot[3]

        return multislot_values

    def parse_gprs_egprs_theoretical_targets(self,
                                             section,
                                             multislot,
                                             dl_coding_scheme,
                                             ul_coding_scheme=None):
        """
            This function parses the theoretical throughput targets
            to get uplink and downlink target values
        """

        try:
            if section not in ["GPRS", "EGPRS"]:
                msg = "Invalid parameter value: '%s'" % section
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            target_expression = "/Sections/Section[@Name='%s']/CodingSchemes" \
                "/CodingScheme[@Name='%s']/attribute::Throughput"

            # Get DL coding scheme target throughput
            target_dl_expression = target_expression % (section, dl_coding_scheme)
            target_dl_results = self._etree_document.xpath(target_dl_expression)
            if target_dl_results:
                dl_target = target_dl_results[0]
            else:
                msg = "Invalid parameter value: CS='%s'" % dl_coding_scheme
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Get UL coding scheme target throughput
            if ul_coding_scheme is None:
                ul_target = dl_target
            else:
                target_ul_expression = target_expression % (section, ul_coding_scheme)
                target_ul_results = self._etree_document.xpath(target_ul_expression)
                if target_ul_results:
                    ul_target = target_ul_results[0]
                else:
                    msg = "Invalid parameter value: CS='%s'" % ul_coding_scheme
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            multislot_values = self._get_multislot_values(multislot)

            target_values = MeasureThroughputTargets()
            target_values.dl_target.set(float(dl_target) * float(multislot_values["DL"]),
                                        ThroughputMeasure.KBPS_UNIT)
            target_values.ul_target.set(float(ul_target) * float(multislot_values["UL"]),
                                        ThroughputMeasure.KBPS_UNIT)
            return target_values
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_wcdma_theoretical_targets(self,
                                        dl_rab,
                                        ul_rab=None):
        """
            This function parses the theoretical throughput targets
            to get WCDMA uplink and downlink target values

        """
        try:
            target_expression = "/Sections/Section[@Name='WCDMA']/RABs/" \
                "RAB[@Name='%s']/attribute::Throughput"

            # Get DL RAB target throughput
            target_dl_expression = target_expression % dl_rab
            target_dl_results = self._etree_document.xpath(target_dl_expression)
            if target_dl_results:
                dl_target = target_dl_results[0]
            else:
                msg = "Invalid parameter value: DL_RAB='%s'" % dl_rab
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Get UL RAB target throughput
            if ul_rab is None:
                ul_target = dl_target
            else:
                target_ul_expression = target_expression % ul_rab
                target_ul_results = self._etree_document.xpath(target_ul_expression)
                if target_ul_results:
                    ul_target = target_ul_results[0]
                else:
                    msg = "Invalid parameter value: UL_RAB='%s'" % ul_rab
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            target_values = MeasureThroughputTargets()
            target_values.dl_target.set(float(dl_target), ThroughputMeasure.KBPS_UNIT)
            target_values.ul_target.set(float(ul_target), ThroughputMeasure.KBPS_UNIT)

            return target_values
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_hspa_theoretical_targets(self,
                                       hsdpa_cat,
                                       hsupa_cat=None,
                                       ul_rab=None):
        """
            This function parses the theoretical throughput targets
            to get HSDPA and (HSUPA or WCDMA) target values.
            hsupa_cat OR ul_rab param must be equal to None.

            :type hsdpa_cat: String
            :param hsdpa_cat: (1|2|3|4|5|6|7|8|11|12)

            :type hsupa_cat: String
            :param hsupa_cat: (1|2|3|4|5|6|7|8|9) or None

            :type ul_rab: String
            :param ul_rab: None or (64k|128k|384k)

            :rtype: Tuple ( Integer, Integer, MeasureThroughputTargets instance)
            :return: Tuple containing:
                    - CQI value
                    - TTI value if hsupa_cat not equals to None
                    - targets values

        """
        try:
            target_expression = "/Sections/Section[@Name='%s']" \
                "/Categories/Category[@Name='%s']"

            # Get HSDPA target throughput
            target_dl_expression = target_expression % ("HSDPA", hsdpa_cat)
            target_dl_results = self._etree_document.xpath(target_dl_expression)
            if target_dl_results:
                dl_target = target_dl_results[0].attrib["Throughput"]
                cqi_value = int(target_dl_results[0].attrib["CQI"])
            else:
                msg = "Invalid parameter value: HSDPA_CAT='%s'" % hsdpa_cat
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Get HSUPA/UL_RAB target throughput
            if hsupa_cat is not None:
                target_ul_expression = target_expression % ("HSUPA", hsupa_cat)
                target_ul_results = self._etree_document.xpath(target_ul_expression)
                if target_ul_results:
                    ul_target = target_ul_results[0].attrib["Throughput"]
                    tti_value = int(target_ul_results[0].attrib["TTI"])
                else:
                    msg = "Invalid parameter value: HSUPA_CAT='%s'" % hsupa_cat
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            elif ul_rab is not None:
                target_ul_rab_expression = "/Sections/Section[@Name='WCDMA']/RABs/" \
                                           "RAB[@Name='%s']/attribute::Throughput" % ul_rab
                target_ul_rab_results = self._etree_document.xpath(target_ul_rab_expression)
                if target_ul_rab_results:
                    ul_target = target_ul_rab_results[0]
                    tti_value = None
                else:
                    msg = "Invalid parameter value: UL_RAB='%s'" % ul_rab
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            else:
                raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR,
                                         "Failed to parse hspa throughput targets "
                                         "(missing hsupa category or ul rab value) !")

            target_values = MeasureThroughputTargets()
            target_values.dl_target.set(float(dl_target), ThroughputMeasure.KBPS_UNIT)
            target_values.ul_target.set(float(ul_target), ThroughputMeasure.KBPS_UNIT)

            return target_values, cqi_value, tti_value
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_tdscdma_theoretical_targets(self, direction):
        """
            This function parses the theoretical throughput targets
            to get TD-SCDMA target values.
        """
        try:
            target_expression = "/Sections/Section[@Name='TD-SCDMA']" \
                                "/Categories/Category[@Direction='%s']" % direction

            target_results = self._etree_document.xpath(target_expression)
            tdscdma_target = target_results[0].attrib

            target_values = MeasureThroughputTargets()
            if direction.upper() in ["DL", "BOTH"]:
                target_values.dl_target.set(float(tdscdma_target["DL_Throughput"]), ThroughputMeasure.KBPS_UNIT)
            if direction.upper() in ["UL", "BOTH"]:
                target_values.ul_target.set(float(tdscdma_target["UL_Throughput"]), ThroughputMeasure.KBPS_UNIT)

            return target_values
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_live_wcdma_targets(self):
        """
        This function parses the Throughput Live Wcdma Targets
        to get WCDMA live unlink and downlink failure and target values
        in Kbps

        :rtype: MeasureThroughputTargets
        :return: Live Wcdma Targets with Target data.
        """
        try:
            target_wifi_expression = "/Sections/Section[@Name=\"LIVE_WCDMA\"]\
                /ThroughputTypes/ThroughputType[@Name=\"TARGET\"]/Directions\
                /Direction[@Name=$Name]"

            failure_wifi_expression = "/Sections/Section[@Name=\"LIVE_WCDMA\"]\
                /ThroughputTypes/ThroughputType[@Name=\"FAILURE\"]/Directions\
                /Direction[@Name=$Name]"

            failure_dl_live_wcdma = self._etree_document.xpath.find(
                failure_wifi_expression,
                self._document,
                Name="DL")
            failure_dl_live_wcdma = failure_dl_live_wcdma[0]

            failure_ul_live_wcdma = self._etree_document.xpath.find(
                failure_wifi_expression,
                self._document,
                Name="UL")
            failure_ul_live_wcdma = failure_ul_live_wcdma[0]

            target_dl_live_wcdma = self._etree_document.xpath.find(
                target_wifi_expression,
                self._document,
                Name="DL")
            target_dl_live_wcdma = target_dl_live_wcdma[0]

            target_ul_live_wcdma = self._etree_document.xpath.find(
                target_wifi_expression,
                self._document,
                Name="UL")
            target_ul_live_wcdma = target_ul_live_wcdma[0]

            live_wcdma_targets = MeasureThroughputTargets()
            live_wcdma_targets.dl_failure.set(float(self._get_minidom_attribute_value(failure_dl_live_wcdma, "Throughput")),
                                              ThroughputMeasure.KBPS_UNIT)
            live_wcdma_targets.dl_target.set(float(self._get_minidom_attribute_value(target_dl_live_wcdma, "Throughput")),
                                             ThroughputMeasure.KBPS_UNIT)
            live_wcdma_targets.ul_failure.set(float(self._get_minidom_attribute_value(failure_ul_live_wcdma, "Throughput")),
                                              ThroughputMeasure.KBPS_UNIT)
            live_wcdma_targets.ul_target.set(float(self._get_minidom_attribute_value(target_ul_live_wcdma, "Throughput")),
                                             ThroughputMeasure.KBPS_UNIT)
            return live_wcdma_targets
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_lte_theoretical_targets(self, lte_cat, lte_bandwidth=None, lte_ant=None):
        """
        This function parses the theoretical throughput targets
        to get lte uplink and downlink target values
        :type lte_cat: str
        :param lte_cat: (1|2|3|4)
        :type lte_bandwidth: str
        :param lte_bandwidth: (1.4|3|5|10|15|20 MHz)
        :type lte_ant: str
        :param lte_ant: (1|2) number of antennas (2 for MIMO)
        :return: a tuple of 3 objects:
                 target_values (a MeasureThroughputTargets object,
                                containing the target throughput),
                 lte_parameters (an LteParameters object, containing dl_nb_rb, dl_i_mcs,
                                 transmission mode, ul_nb_rb and ul_i_mcs)
        """
        # If lte_bandwidth and lte_ant parameter are not set, init them for max throughput
        if lte_bandwidth is None:
            if int(lte_cat) > 2:
                lte_bandwidth = "20"
            else:
                lte_bandwidth = "10"
        if lte_ant is None:
            if int(lte_cat) > 1:
                lte_ant = "2"
            else:
                lte_ant = "1"

        try:
            # Get throughput targets and optimal parameters for LTE connection
            target_expression = "/Sections/Section[@Name='LTE']/Categories" \
                                "/Category[@Name='%s'and@Bandwidth='%s'and@Antenna='%s']" \
                                % (lte_cat, lte_bandwidth, lte_ant)

            target_results = self._etree_document.xpath(target_expression)
            if target_results:
                lte_target = target_results[0].attrib
            else:
                msg = "Invalid parameter value: LTE_CAT='%s', BW='%s' or NB_ANT='%s'" \
                      % (lte_cat, lte_bandwidth, lte_ant)

                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            target_values = MeasureThroughputTargets()
            target_values.dl_target.set(float(lte_target["DL_Throughput"]), ThroughputMeasure.KBPS_UNIT)
            target_values.ul_target.set(float(lte_target["UL_Throughput"]), ThroughputMeasure.KBPS_UNIT)
            target_values.bler = float(lte_target["max_BLER"])

            # Parse LTE parameters
            lte_parameters = LteParameters()
            lte_parameters.dl_i_mcs = int(lte_target["dl_i_mcs"])
            lte_parameters.dl_nb_rb = int(lte_target["dl_nb_rb"])
            lte_parameters.ul_i_mcs = int(lte_target["ul_i_mcs"])
            lte_parameters.ul_nb_rb = int(lte_target["ul_nb_rb"])
            lte_parameters.transmission_mode = str(lte_target["tm_mode"])

            return target_values, lte_parameters

        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise

    def parse_lte_tdd_targets(self, lte_cat, lte_direction='DL'):
        """
        This function parses the Throughput targets
        to get lte uplink and downlink failure and target values
        :type lte_cat: str
        :param lte_cat: (1|2|3|4)
        :type lte_direction: str
        :param lte_direction: (UL|DL|BOTH)
        :return: a tuple of 3 objects:
                 target_values (a MeasureThroughputTargets object,
                                containing the target throughput),
                 lte_parameters_dl (an LteParameters object, containing dl_nb_rb, dl_i_mcs,
                                    transmission mode, ul_nb_rb and ul_i_mcs),
                 lte_parameters_ul (the same as lte_parameters_dl)
        """
        try:
            # Get Throughput targets and optimal parameters for LTE connection
            target_expression = "/Sections/Section[@Name='LTE_TDD']/Categories" \
                                "/Category[@Name='%s'and@Direction='%s']" % \
                                (lte_cat, lte_direction)

            target_results = self._etree_document.xpath(target_expression)
            if target_results:
                lte_target = target_results[0].attrib
            else:
                msg = "Invalid parameter value: '%s or %s'" \
                      % (lte_cat, lte_direction)

                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            target_values = MeasureThroughputTargets()
            if lte_direction.upper() in("DL", "BOTH"):
                target_values.dl_target.set(float(lte_target["DL_Throughput"]), ThroughputMeasure.KBPS_UNIT)
            if lte_direction.upper() in("UL", "BOTH"):
                target_values.ul_target.set(float(lte_target["UL_Throughput"]), ThroughputMeasure.KBPS_UNIT)
            target_values.bler = float(lte_target["max_BLER"])

            return target_values, lte_target["TDD_Configuration"]

        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise



    def parse_lte_expected_safe_ranges(self,
                         device_model,
                         channel="",
                         aliases=None):
        """
        This function parses the LTE expected safe ranges

        :type device_model: String
        :param device_model: Name of the device model under test
        :type channel: String
        :param channel: The channel for which you want to get the safe range.
        :type aliases: List
        :param aliases: List of parent devices to avoid infinite
                        recursion. Do not use this parameter: it is
                        reserved for internal use.

        :rtype: LTEExpectedSafeRanges
        :return : LTE expected safe ranges data.
        """
        channel = int(channel)

        if not isinstance(aliases, list):
            # Set to default value if not list
            aliases = []

        # look for the device in in wifi_targets
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) == 0:
            msg = "DeviceModel %s does not exist in LTE_Expected_Safe_Ranges.xml" \
                % device_model
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # look for an alias
        alias_expression = "/DeviceModels/DeviceModel[@Name='%s']/Alias" % device_model
        result = self._etree_document.xpath(alias_expression)
        if len(result) != 0:
            # alias found, add device model in already seen devices
            aliases.append(device_model)
            base_model = self._get_attribute_value(result[0], "Name")
            if base_model in aliases:
                # error: malformed xml causing infinite recursion
                msg = "DeviceModel %s is malformed in LTE_Expected_Safe_Ranges.xml" \
                    % device_model
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            # retry with this device
            return self.parse_lte_expected_safe_ranges(base_model,
                                         channel,
                                         aliases)

        # no alias, find this device
        try:
            target_expression = "/DeviceModels/DeviceModel[@Name='%s']" % device_model
            target_expression += "/LTEExpectedSafeRanges/LTExpectedSafeRange[@Channel='%s']" % channel

            lte_safe_range_data = self._etree_document.xpath(target_expression)

            if lte_safe_range_data is None:
                msg = "Can't find data for channel %s in LTE_Expected_Safe_Ranges.xml" % channel
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            # Get data from XML
            lte_safe_range_data = lte_safe_range_data[0]

            wifi_min_freq_rx = int(self._get_attribute_value(lte_safe_range_data, "wifi_min_freq_rx"))
            wifi_max_freq_rx = int(self._get_attribute_value(lte_safe_range_data, "wifi_max_freq_rx"))
            bt_ble_min_freq_rx = int(self._get_attribute_value(lte_safe_range_data, "bt_ble_min_freq_rx"))
            bt_ble_max_freq_rx = int(self._get_attribute_value(lte_safe_range_data, "bt_ble_max_freq_rx"))
            wifi_min_freq_tx = int(self._get_attribute_value(lte_safe_range_data, "wifi_min_freq_tx"))
            wifi_max_freq_tx = int(self._get_attribute_value(lte_safe_range_data, "wifi_max_freq_tx"))
            bt_ble_min_freq_tx = int(self._get_attribute_value(lte_safe_range_data, "bt_ble_min_freq_tx"))
            bt_ble_max_freq_tx = int(self._get_attribute_value(lte_safe_range_data, "bt_ble_max_freq_tx"))

            # Fill output data
            lte_safe_ranges = LteSafeRanges()
            lte_safe_ranges.wifi_min_freq = max(wifi_min_freq_rx, wifi_min_freq_tx)
            if lte_safe_ranges.wifi_min_freq in [2400, 2401]:
                lte_safe_ranges.wifi_min_freq = 2402
            lte_safe_ranges.wifi_max_freq = min (wifi_max_freq_rx, wifi_max_freq_tx)
            if lte_safe_ranges.wifi_max_freq in range(2495, 2500):
                lte_safe_ranges.wifi_max_freq = 2495

            # Check special case for band 40, return full range.
            if lte_safe_ranges.wifi_min_freq > lte_safe_ranges.wifi_max_freq:
                lte_safe_ranges.wifi_min_channel = 12
                lte_safe_ranges.wifi_max_channel = 14
                # wifi_min_freq is unchanged
                lte_safe_ranges.wifi_max_freq = 2495
            else:
                # Normal case
                # Find first mathing channel browsing the frequencies upward.

                lte_safe_ranges.wifi_min_channel = lte_safe_ranges.get_safe_wifi_channel_from_freq(lte_safe_ranges.wifi_min_freq, "min")
                lte_safe_ranges.wifi_max_channel = lte_safe_ranges.get_safe_wifi_channel_from_freq(lte_safe_ranges.wifi_max_freq, "max")

            # reduce BT max freq to the max value
            if bt_ble_max_freq_tx > 2484:
                bt_ble_max_freq_tx = 2484

            lte_safe_ranges.bt_min_freq = max(bt_ble_min_freq_rx, bt_ble_min_freq_tx)
            if lte_safe_ranges.bt_min_freq in [2400, 2401]:
                lte_safe_ranges.bt_min_freq = 2402
            lte_safe_ranges.bt_max_freq = min(bt_ble_max_freq_rx, bt_ble_max_freq_tx)
            if lte_safe_ranges.bt_max_freq in range(2481, 2485):
                lte_safe_ranges.bt_max_freq = 2480
            lte_safe_ranges.ble_min_freq = lte_safe_ranges.bt_min_freq
            lte_safe_ranges.ble_max_freq = lte_safe_ranges.bt_max_freq

            # Check special case for band 40, return full range.
            if lte_safe_ranges.bt_min_freq > lte_safe_ranges.bt_max_freq:
                lte_safe_ranges.bt_min_channel = 0
                lte_safe_ranges.bt_max_channel = 78
                lte_safe_ranges.ble_min_channel = 0
                lte_safe_ranges.ble_max_channel = 39
                lte_safe_ranges.bt_min_freq = 2402
                lte_safe_ranges.ble_min_freq = 2402
                lte_safe_ranges.bt_max_freq = 2480
                lte_safe_ranges.ble_max_freq = 2480
            else:
                # Normal case
                lte_safe_ranges.bt_min_channel = get_bt_channel_from_freq(lte_safe_ranges.bt_min_freq)
                lte_safe_ranges.bt_max_channel = get_bt_channel_from_freq(lte_safe_ranges.bt_max_freq)

                lte_safe_ranges.ble_min_channel = lte_safe_ranges.get_safe_ble_channel_from_freq(lte_safe_ranges.bt_min_freq, "min")
                lte_safe_ranges.ble_max_channel = lte_safe_ranges.get_safe_ble_channel_from_freq(lte_safe_ranges.bt_max_freq, "max")


            return lte_safe_ranges
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            LOGGER_TEST_SCRIPT.error(format_exception_info())
            raise


def throughput_targets_string(throughput_targets):
    """
    Compute and return Throughput target and failure for DL and UL in a str.

    :type throughput_targets: MeasureThroughputTargets
    :param throughput_targets: Structure that represent throughput measure objects (in Kbps)

    :rtype: str
    :return: msg, the str with Throughput target and failure for DL and UL
    """
    msg = "Throughputs: "
    if throughput_targets is not None:
        # Target
        dl_target = throughput_targets.dl_target.to_best_unit()
        ul_target = throughput_targets.ul_target.to_best_unit()

        # Failure
        dl_failure = throughput_targets.dl_failure.to(dl_target.unit)
        ul_failure = throughput_targets.ul_failure.to(ul_target.unit)

        # Log throughputs:
        msg += "Target: DL:%s UL:%s / Failure: DL:%s UL:%s - BLER:%s%%" \
               % (str(dl_target), str(ul_target),
                  str(dl_failure), str(ul_failure), throughput_targets.bler)
    return msg


class BlerMeasurements(threading.Thread):
    """
    Class that perform BLER (Block Error Rate) measurement on CMW
    It can be run conccurently of another task
    """
    def __init__(self, duration, offset, direction, equipment_api):
        threading.Thread.__init__(self)
        self._ul_bler = 0
        self._dl_bler = 0
        self._duration = duration
        self._offset = offset
        self._direction = direction
        self._equipment = equipment_api
        self._bler = 0.0
        self._status = False

    @property
    def bler_measure(self):
        """
        Return the measured BLER
        :rtype: float
        :return: measured BLER
        """
        return self._bler

    @property
    def bler_status(self):
        """
        Return the BLER measurement status
        :rtype: bool
        :return: BLER measurement status
        """
        return self._status

    def run(self):
        """
        Perform BLER measurement
        First wait a period of time (offset)
        Start BLER measurement over another period of time (duration)
        Get BLER measurement
        """
        time.sleep(self._offset)
        try:
            self._bler = self._equipment.get_ext_bler_measurement(self._direction, self._duration)
            self._status = True
        except TestEquipmentException as exception:
            LOGGER_TEST_SCRIPT.error("Error during BLER measurement: %s" % exception.get_error_message())
            self._status = False


class KPIUtil():

    """
    Structure that represent KPI objects,
    it is needed to store all the measured throughput during a KPI test:
    A KPI iperf test is 5 iteration long
    and test verdict is done on median throughput of these 5 iterations
    """
    def __init__(self):
        self._kpi_ul_throughputs = []
        self._kpi_dl_throughputs = []
        self._kpi_bler = []

    def append(self, meas_tput, bler=0.0):
        """
        Store UL and DL throughput in KPI internal storage.

        :type meas_tput: DuplexMeasureThroughput
        :param meas_tput: UL and DL througput values to append to kpi results
        """
        if meas_tput is not None:
            if meas_tput.ul_throughput.value != 0.0:
                self._kpi_ul_throughputs.append(meas_tput.ul_throughput)
            # convert and store DL throughput unit to the first measured throughput unit
            if meas_tput.dl_throughput.value != 0.0:
                self._kpi_dl_throughputs.append(meas_tput.dl_throughput)
            self._kpi_bler.append(bler)

    def get_median_throughput(self):
        """
        Compute and return median throughput value of a kpi iperf test.
        A KPI iperf test is 5 iterations long
        and test verdict is done on median throughput of these 5 iterations

        :rtype: DuplexThroughputMeasure
        :return: througput, the median throughput of the KPI test
        """
        throughput = DuplexThroughputMeasure()
        if len(self._kpi_ul_throughputs) > 0:
            # Compute UL median throughput
            median = sorted(self._kpi_ul_throughputs)[len(self._kpi_ul_throughputs) // 2]
            throughput.ul_throughput = median.to_best_unit()
        if len(self._kpi_dl_throughputs) > 0:
            # Compute DL median throughput
            median = sorted(self._kpi_dl_throughputs)[len(self._kpi_dl_throughputs) // 2]
            throughput.dl_throughput = median.to_best_unit()

        return throughput

    def get_median_bler(self):
        """
        Compute and return median bler value of a kpi iperf test.
        A KPI iperf test is 5 iterations long
        and test verdict is done on median throughput of these 5 iterations

        :rtype: float
        :return: bler, the median bler of the KPI test
        """
        bler = 0.0
        if len(self._kpi_bler) > 0:
            # Compute median BLER
            bler = sorted(self._kpi_bler)[len(self._kpi_bler) // 2]

        return bler

class LteSafeRanges():
    def __init__(self):
        self.wifi_min_channel = 0
        self.wifi_max_channel = 0
        self.wifi_min_freq = 0
        self.wifi_max_freq = 0
        self.ble_min_channel = 0
        self.ble_max_channel = 0
        self.ble_min_freq = 0
        self.ble_max_freq = 0
        self.bt_min_channel = 0
        self.bt_max_channel = 0
        self.bt_min_freq = 0
        self.bt_max_freq = 0

    def get_safe_ble_channel_from_freq(self, freq, min_max):
        """
        Get the 1st safe BLE RF channel ID from its frequency

        :type freq: int or str
        :param freq: The channel frequency
        :type min_max: str (min/max)
        :param min_max: Tell the method to return the minimun matching channel (parse the channel list from bottom)
                        or other way around.

        :rtype: int
        :return: The channel ID
        """
        freq = int(freq)
        if freq < 2402:
            freq = 2402
        if freq > 2480:
            freq = 2480
        if min_max.lower() == "min":
            result = int(ceil((float(freq - 2402)) / 2))
        else:
            result = ((freq - 2402) / 2)
        return result

    def get_safe_wifi_channel_from_freq(self, freq, min_max, bandwidth=20):
        """
        Get the 1st safe WiFi RF channel ID from its frequency

        :type freq: int
        :param freq: The channel frequency
        :type min_max: str (min/max)
        :param min_max: Tell the method to return the minimun matching channel (parse the channel list from bottom)
                        or other way around.
        :type bandwidth: int
        :param bandwidth: The channel bandwidth

        :rtype: int
        :return: The channel ID
        """
        if min_max.lower() == "min":
            # We start the parsing at the center of the channel
            min_freq = freq + (bandwidth / 2)
            channel_min = "0"
            while channel_min == "0":
                channel_min = AcsWifiFrequencies.get_wifi_channel_from_frequency(str(min_freq))
                min_freq = min_freq + 1
            result = channel_min
        else:
            # Find first matching channel browsing the frequencies downward.
            max_freq = freq - (bandwidth / 2)
            channel_max = "0"
            while channel_max == "0":
                channel_max = AcsWifiFrequencies.get_wifi_channel_from_frequency(str(max_freq))
                max_freq = max_freq - 1
            result = channel_max
        return result
