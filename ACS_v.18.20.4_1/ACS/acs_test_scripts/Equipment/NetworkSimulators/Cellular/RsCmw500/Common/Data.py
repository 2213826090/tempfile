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
:summary: Common Data (2G, 3G & 4G) implementation for CMW500
:since: 18/04/2013
:author: hbian
"""

import time
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.IPerfUtilities import Iperf
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure, DuplexThroughputMeasure
from ErrorHandling.AcsBaseException import AcsBaseException


class Data(Iperf, VisaObject):

    """
    Common Data (2G, 3G & 4G) implementation for CMW500
    """

    """
    Possible values for Reliability indicator returned by CMW after a measure
    It indicates the most severe error that has occurred during the measurement.
    """
    _reliability_indicator = {0: "OK",
                              1: "Measurement Timeout",
                              2: "Capture Buffer Overflow",
                              3: "Overdriven",
                              4: "Underdriven",
                              6: "Trigger Timeout",
                              7: "Acquisition Error",
                              8: "Sync Error",
                              9: "Uncal",
                              15: "Reference Frequency Error",
                              16: "RF Not Available",
                              17: "RF Level not Settled / 18 RF Frequency not Settled",
                              19: "Call not Established",
                              20: "Call Type not Usable",
                              21: "Call Lost",
                              23: "Missing Option",
                              26: "Resource Conflict",
                              27: "No Sensor Connected",
                              30: "File not Found",
                              40: "ARB File CRC Error",
                              42: "ARB Header Tag Invalid",
                              43: "ARB Segment Overflow",
                              44: "ARB File not Found",
                              45: "ARB Memory Overflow",
                              50: "Startup Error",
                              51: "No Reply",
                              52: "Connection Error",
                              53: "Configuration Error",
                              54: "Filesystem Error",
                              101: "Firmware Error",
                              102: "Unidentified Error",
                              103: "Parameter Error"}

    def __init__(self, visa):  # pylint: disable=W0231
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)
        Iperf.__init__(self)
        self._visa = visa
        self.su_id = ""
        self.window_size_enabled = True

    def disable_window_size_setting(self):
        """
        Disables the setting of a window size on CMW500

        """
        self.window_size_enabled = False

    # DAU configuration functions

    def start_dau(self, timeout):
        """
        Start the Data Application unit on CMW500

        :rtype: None
        :raise TestEquipmentException: If unable to query equipment
        """
        # Enable DAU unit  application to run on the CMW500
        initial_dau_state = self._visa.query_command("SOUR:DATA:CONT:STAT?")

        if "ON" in initial_dau_state:
            self.get_logger().info("DAU unit already switched on !")
        elif "PEND" in initial_dau_state:
            # DAU unit already switched on, wait for ON
            self.get_logger().info("DAU unit is currently switching on ...")
        elif "OFF" in initial_dau_state:
            # Switch ON DAU unit, then wait
            self.get_logger().info("Switching on the CMW500 DAU unit...")
            self._visa.send_command("SOUR:DATA:CONT:STAT ON")
        else:
            msg = "Unable to query proper DAU unit state"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

        self.check_dau_state_before_timeout(timeout, "ON")

    def check_dau_state_before_timeout(self, timeout, state, period=5):
        """
        Wait for the given DAU state before timeout

        :type timeout: int
        :param timeout: The time during the global check is performed

        :type period: int
        :param period: The period when the queries are performed

        :type state: str
        :param state: epc (DAU) state targeted before timeout

        :rtype: None

        :raise AcsConfigException: If given state parameter is incorrect
        :raise TestEquipmentException: If unable to reach <state> on time
        """
        timeout = int(timeout)
        period = int(period)
        state_reached = False
        if state not in ["ON", "OFF", "PEND"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Given state parameter (%s) is not valid for CMW500" % str(state))

        while timeout > 0:
            if self._visa.query_command("SOUR:DATA:CONT:STAT?") == state:
                state_reached = True
                break
            time.sleep(period)
            timeout -= period

        if not state_reached:
            err_msg = "Error while attempting to reach %s state on DAU unit !" % state
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
        else:
            msg = "State %s reached on time for DAU unit" % state
            self.get_logger().info(msg)

    def get_su_id(self, su_list):
        """
        Set signaling unit wanted from a list of signalling unit
        :type su_list: str list
        :param su_list: list of available signalling units
        """
        pass

    def get_su_state(self):
        """
        Returns the cell state
        :rtype: str
        :return: cell state (ON|OFF)
        """
        return "OFF"

    def set_epc_on(self):
        """
        Set the Data Application Unit on.

        It is necessary to perform end-to-end functional tests.
        Otherwise, all uplink IP data will be discarded.
        :raise TestEquipmentException: if signaling unit is still enabled when this function is called
        """
        # Start DAU with 300s timeout
        self.start_dau(300)

        # Configuring Signaling Unit
        self.get_logger().info(
            "Getting the list of installed LTE SUs in the CMW500... will configure DAU to use the first in the list...")
        su_list = self._visa.query_command("CONF:DATA:MEAS:RAN:CAT?").split(",")
        self.get_su_id(su_list)

        if self.su_id != "":
            self.get_logger().info('Found SU, checking its status...')
            su_status = self.get_su_state()
            if su_status == "OFF":
                self.get_logger().info('Configure DAU application for operation using %s application of the CMW500...' % self.su_id)
                self._visa.send_command("CONF:DATA:MEAS:RAN %s" % self.su_id)
            else:
                msg = "%s SU unit MUST be OFF before configuring DAU to use it!!!" % self.su_id
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        else:
            msg_no_lte = "No valid Signaling Unit found in this CMW500!!!"
            self.get_logger().error(msg_no_lte)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg_no_lte)

    def set_epc_off(self):
        """
        Set the Data Application Unit (equivalent of Evolved Packet Core on PXT (EPC)) off
        """
        self.get_logger().info("Switching off the CMW500 DAU unit...")
        self._visa.send_command("SOUR:DATA:CONT:STAT OFF")

    # FTP functions #

    def get_data_throughput(self, duration, type_list):
        """
        Get the data throughput during the duration time

        :type duration: int
        :param duration: measurement duration

        :type type_list: String list
        :param type_list: The type of throughput to measure: uplink_ip, downlink_ip

        :rtype: dict
        :return: the dictionary of test result:
        Current data throughput(bps) ,Min data throughput(bps),
        Max data throughput(bps), Average data transfer(bytes)
        """

        data_throughput_dict = {}

        # Init data measurement
        start_measure_cmd = "INIT:DATA:MEAS:THR"
        self._visa.send_command(start_measure_cmd)
        self.get_logger().info("Start data throughput measurement")

        time.sleep(duration)

        # Init data measurement
        stop_measure_cmd = "STOP:DATA:MEAS:THR"
        self._visa.send_command(stop_measure_cmd)
        self.get_logger().info("Stop data throughput measurement")

        for meas_type in type_list:
            if meas_type == "UL":
                measure_cmd_type = "ULIN"
            elif meas_type == "DL":
                measure_cmd_type = "DLIN"
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Wrong formating of the throughput dict.")
            get_throughput_cmd = "FETC:DATA:MEAS:THR:OVER:%s?" % measure_cmd_type
            # The result is in a comma separated str with value in the Following order
            # Current data throughput(bps) ,Min data throughput(bps),
            # Max data throughput(bps), Average data transfer(bytes)
            results = self._visa.query_command(get_throughput_cmd)
            self.get_logger().debug(results)
            # we want just to retrieve the average data throughput
            average_result = results.split(',')[3]
            # Convert data throughput result with the good units: bps/kbps/Mbps
            # If the throughput is Mbps
            if float(average_result) / 1000 >= 1000:
                result = str(round(float(average_result) / (1000 * 1000), 2)) + "Mbps"
            # If the throughput is kbps
            elif float(average_result) / 1000 >= 1:
                result = str(round(float(average_result) / 1000, 2)) + "Kbps"
            else:
                result = str(round(float(average_result), 2)) + "bps"

            data_throughput_dict.update({meas_type: result})

        return data_throughput_dict

    def start_ftp_service(self):
        """
        Start FTP server service in DAU.
        """
        set_ftp_server_service = "CONF:DATA:CONT:FTP:STYP SERV"
        start_ftp_service = "SOUR:DATA:CONT:FTP:STAT ON"

        self._visa.send_command(set_ftp_server_service)
        self._visa.send_command(start_ftp_service)

    def stop_ftp_service(self):
        """
        Stop the FTP server in the DAU.
        """
        stop_ftp_service = "SOUR:DATA:CONT:FTP:STAT OFF"
        self._visa.send_command(stop_ftp_service)

# Iperf functions #

    def start_iperf_service(self):
        """
        Starts the IPERF measurement.
        """
        if self.get_iperf_measurement_state() != "RUN":
            start_iperf_service = "INIT:DATA:MEAS:IPER"
            self._visa.send_command(start_iperf_service)

    def stop_iperf_service(self):
        """
        Stop the IPERF measurement.
        """
        stop_iperf_service = "STOP:DATA:MEAS:IPER"
        self._visa.send_command(stop_iperf_service)

    def abort_iperf_service(self):
        """
        Abort the IPERF measurement.
        """
        stop_iperf_service = "ABORt:DATA:MEAS:IPER"
        self._visa.send_command(stop_iperf_service)

    def disable_iperf_server(self):
        """
        Disables the IPERF server.
        """
        disable_iperf_server = "CONF:DATA:MEAS:IPER:SERV:ENAB OFF"
        self._visa.send_command(disable_iperf_server)

    def disable_iperf_client(self):
        """
        Disables the IPERF client
        """
        disable_iperf_client = "CONF:DATA:MEAS:IPER:CLI:ENAB OFF"
        self._visa.send_command(disable_iperf_client)

    def configure_iperf_server(self, settings):
        """
        Configure the IPERF server according to the settings dictionary.
        :type settings: dict
        :param settings: dictionary containing all the setting of the
        IPERF test to be launched.
        """
        protocol = settings.get('protocol')
        port_number = settings.get('port_number')
        duration = settings.get('duration')
        parallel_thread = settings.get('parallel_thread')

        # CMW can be used as an iperf server only for UL transfer
        w_size = settings.get('ul_server_window_size')

        # Enabling the equipment IPERF server.
        self._enable_iperf_server()

        # Setting the IPERF test duration if it is bigger than default duration (1000s)
        if duration > 950:
            self._set_iperf_test_duration(duration + 15)

        # Set nb of server for test
        if parallel_thread not in (None, ""):
            if parallel_thread > 0:
                parallel_thread = int(parallel_thread)
            else:
                parallel_thread = 1
        else:
            parallel_thread = 1

        # configure the number of desired server
        for i in range(parallel_thread):
            server_str = "SERVer" + str(i + 1)
            # Checking the port_number parameter is in the expected scope.
            if port_number not in (None, ""):
                # Setting the listening port number of the internal IPERF server.
                self.get_logger().info("Setting the listening port number of "
                                       "internal IPERF server to %s"
                                       % str(port_number))
                self._set_iperf_port_number(port_number, server_str)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Need to specify a int port number.")

            # Setting the protocol of the internal IPERF server.
            if protocol not in (None, ""):
                # Setting the protocol of the internal IPERF server.
                self._set_iperf_protocol(protocol, server_str)
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Need to specify a protocol (udp or tcp).")

            if "tcp" in protocol.lower():
                if w_size not in (None, ""):
                    # Removing the unit from the window size parameter.
                    if "K" in str(w_size[-1]).upper():
                        w_size = str(int(float(w_size[:-1])))
                    elif "M" in str(w_size[-1].upper()):
                        w_size = str(int(float(w_size[:-1]) * 1024))
                    # Checking the window size parameter is in the expected scope.
                    if w_size.isdigit():
                        # Setting the window size for the internal IPERF server.
                        w_size = int(w_size)
                        self._set_iperf_window_size(w_size, server_str)
                    else:
                        raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                                     "Should specify a int window size while tcp protocol.")

    def _set_iperf_parallel_conn_number(self, parallel_conn):
        """
        Specifies the number of parallel connections for an IPerf client
        instance. Only applicable for protocol type TCP.
        :param parallel_conn: Number of parallel connection. Range 1 to 4
        :type parallel_conn: int
        """
        gpib_command = "CONF:DATA:MEAS:IPER:CLI:PCON"
        if isinstance(parallel_conn, int) and parallel_conn in range(1, 5):
            self.get_logger().info("Setting the number of parallel connections"
                                   " to %s" % str(parallel_conn))
            self._visa.send_command(gpib_command + " " + str(parallel_conn))
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Need to specify a int number of parallel"
                                         " connections between 1 and 4.")

    def _enable_iperf_client(self):
        """
        Activates an IPerf client instance.
        """
        # Enables the equipment IPERF client.
        enable_iperf_client = "CONF:DATA:MEAS:IPER:CLI:ENAB ON"
        self.get_logger().info("Enabling the internal IPERF client.")
        self._visa.send_command(enable_iperf_client)

    def _enable_iperf_server(self):
        """
        Activates an IPerf server instance.
        """
        enable_iperf_server = "CONF:DATA:MEAS:IPER:SERV:ENAB ON"
        self.get_logger().info("Enabling the internal IPERF server.")
        self._visa.send_command(enable_iperf_server)

    def configure_iperf_client(self, settings):
        """
        Configure the IPERF client according to the settings dictionary.
        :type settings: dict
        :param settings: dictionary containing all the setting of the
        IPERF test to be launched. The dictionary should contain the following
        keys: 'duration', 'protocol', 'port_number', 'window_size', 'bandwidth'
        'parallel_thread', 'dut_ip_address'
        """
        duration = settings.get('duration')
        protocol = settings.get('protocol')
        port_number = settings.get('port_number')
        w_size = settings.get('dl_client_window_size')
        bandwidth = settings.get('bandwidth')
        if protocol == "udp" and "dl_bandwidth" in settings.keys():
            bandwidth = settings["dl_bandwidth"]
        parallel_thread = settings.get('dl_parallel_thread')
        dut_address_ip = settings.get('dut_ip_address')

        # Enabling the equipment IPERF client.
        self._enable_iperf_client()

        # Setting the IPERF test duration
        self._set_iperf_test_duration(duration)

        # Checking the parallel_connection is not none or empty.
        if parallel_thread not in (None, "", 0, 1, "0", "1"):
            self._set_iperf_parallel_conn_number(parallel_thread)
        else:
            self._set_iperf_parallel_conn_number(1)

        # Checking the port_number parameter is in the expected scope.
        if port_number not in (None, ""):
            # Setting the port number of the internal IPERF client.
            self._set_iperf_port_number(port_number, "CLIENT")
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Need to specify a int port number.")

        # Setting the protocol of the internal IPERF client.
        if protocol not in (None, ""):
            self._set_iperf_protocol(protocol, "CLIENT")
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Need to specify a protocol (udp or tcp).")

        if "tcp" in protocol.lower():
            if w_size not in (None, ""):
                # Removing the unit from the window size parameter.
                if "K" in str(w_size[-1]).upper():
                    w_size = str(int(float(w_size[:-1])))
                elif "M" in str(w_size[-1].upper()):
                    w_size = str(int(float(w_size[:-1]) * 1024))

                # Checking the window size parameter is in the expected scope.
                if w_size.isdigit():
                    # Setting the window size for the internal IPERF client.
                    w_size = int(w_size)
                    self._set_iperf_window_size(w_size, "CLIENT")
                else:
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                                 "Should specify a int window size while tcp protocol.")
        elif "udp" in protocol.lower():
            if bandwidth not in (None, ""):
                # Removing the unit from the bandwidth parameter.
                if "K" in str(bandwidth[-1]).upper():
                    bandwidth = str(int(float(bandwidth[:-1]) * 1024))
                elif "M" in str(bandwidth[-1]).upper():
                    bandwidth = str(int(float(bandwidth[:-1]) * 1024 * 1024))
                # Checking the bandwidth parameter is in the expected scope.
                if bandwidth.isdigit():
                    # Setting the bit rate for the internal IPERF client.
                    bandwidth = int(bandwidth)
                    self._set_iperf_bit_rate(bandwidth)
                else:
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                                 "Should specify a int bandwidth while udp protocol.")

        if dut_address_ip not in (None, ""):
            self._set_iperf_client_ip_address(dut_address_ip)
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Should specify a non empty DUT IP address.")

    def _set_iperf_test_duration(self, duration):
        """
        Setting the test duration on the CMW500 internal IPERF server.
        :type duration: int
        :param duration: time in seconds the IPERF test will last.
        """
        gpib_command = "CONF:DATA:MEAS:IPER:TDUR "
        if isinstance(duration, int):
            self.get_logger().debug("Setting the IPERF test duration to %s"
                                    % duration)
            self._visa.send_command(gpib_command + str(duration))
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Need to specify a int duration.")

    def _set_iperf_port_number(self, port_nb, target):
        """
        Setting the port where the IPERF server will be listening.
        :type port_number: int
        :param port_number: port on which the server will be listening.
        Should be between 0 and 65535.
        :type target: str
        :param target: defines if the client or server port number will be
        changed. Should be SERVER or CLIENT
        :raise TestEquipmentException: If one of the parameters is not in its range.
        """
        # Checking the target parameter is in its range.
        if target.upper() == "SERVER":
            gpib_target_param = "SERV"
        # When we want to enable several iperf servers on CMW
        # We configure these servers by calling them SERVER1..SERVERn in the GPIB command
        elif "SERVER" in target.upper():
            gpib_target_param = target
        elif target.upper() == "CLIENT":
            gpib_target_param = "CLI"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "The target should be either SERVER or CLIENT."
                                         " %s is not supported." % target.upper())
        # Checking the port number parameter is in its range.
        if isinstance(port_nb, int) and 0 <= port_nb <= 65535:
            self.get_logger().info("Setting the port number of "
                                   "internal IPERF %s to %s"
                                   % (target.lower(), str(port_nb)))
            gpib_command = "CONF:DATA:MEAS:IPER:%s:PORT " % gpib_target_param
            # Sending the command to the equipment.
            self._visa.send_command(gpib_command + str(port_nb))
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Port number should be between 0 and 65535."
                                         " Is %s" % str(port_nb))

    def _set_iperf_protocol(self, protocol, target):
        """
        Sets the protocol of the IPERF server in the DAU.
        :type protocol: str
        :param protocol: protocol to set, should be: tcp or udp.
        :type target: str
        :param target: defines if the client or server port number will be
        modified
        :raise TestEquipmentException: If one of the parameters is not in its range.
        """
        # Checking the target parameter is in its range.
        if target.upper() == "SERVER":
            gpib_target_param = "SERV"
        # When we want to enable several iperf servers on CMW
        # We configure these servers by calling them SERVER1..SERVERn in the GPIB command
        elif "SERVER" in target.upper():
            gpib_target_param = target
        elif target.upper() == "CLIENT":
            gpib_target_param = "CLI"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "The target should be either SERVER or CLIENT."
                                         " %s is not supported." % target.upper())
        # Checking if the protocol parameter is in its range.
        if str(protocol.upper()) in ("UDP", "TCP"):
            self.get_logger().info("Setting the protocol of internal IPERF"
                                   " server to %s" % protocol)
            gpib_param = protocol.upper()
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Wrong protocol: %s, should be UDP or"
                                         " TCP." % protocol)
        # Building the GPIB command.
        gpib_command = "CONF:DATA:MEAS:IPER:%s:PROT " % gpib_target_param
        # Sending the command to the equipment.
        self._visa.send_command(gpib_command + gpib_param)

    def _set_iperf_window_size(self, w_size, target):
        """
        Sets the window size for the IPERF server in the DAU.
        :type w_size: int
        :param w_size: window size should be between 0 and 64000 Kbytes.
        :type target: str
        :param target: defines if the client or server port number will be
        modified
        :raise TestEquipmentException: If one of the parameters is not in its range.
        """
        # Checking the target parameter is in its range.
        if self.window_size_enabled == True:
            if target.upper() == "SERVER":
                gpib_target_param = "SERV"
            # When we want to enable several iperf servers on CMW
            # We configure these servers by calling them SERVER1..SERVERn in the GPIB command
            elif "SERVER" in target.upper():
                gpib_target_param = target
            elif target.upper() == "CLIENT":
                gpib_target_param = "CLI"
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "The target should be either SERVER or CLIENT."
                                             " %s is not supported." % target.upper())
            gpib_command = "CONF:DATA:MEAS:IPER:%s:WSIZ " % gpib_target_param
            if isinstance(w_size, int) and 0 <= w_size <= 64000:
                self.get_logger().info("Setting the window size of the internal "
                                       " IPERF %s to : %s" % (target.lower(),
                                                              str(w_size)))
                self._visa.send_command(gpib_command + str(w_size))
            else:
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                             "Window size should be an int between 0 and "
                                             "64000. Is %s" % str(w_size))

    def _set_iperf_bit_rate(self, bit_rate):
        """
        Setting the maximum bit rate for an IPerf Client instance. Only applicable for protocol type UDP.
        :type bit_rate: int
        :param bit_rate: Maximum bit rate to be transferred (bit/s), in range [0;1E+9].
        """
        gpib_command = "CONF:DATA:MEAS:IPER:CLI:BITR "

        if isinstance(bit_rate, int) and 0 <= bit_rate <= 1000000000:
            self.get_logger().info("Setting the bit rate of the internal "
                                   " IPERF Client to : %s" % str(bit_rate))
            self._visa.send_command(gpib_command + str(bit_rate))
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Need to specify a int bit rate between 0 and "
                                         "1E+9. %s is not supported" % str(bit_rate))

    def _set_iperf_client_ip_address(self, ip_address):
        """
        Specifies the IP address of the mobile for an IPerf client instance.
        :type ip_address: str
        :param ip_address: String containing the IP address ex: 172.22.1.100
        """
        gpib_cmd = "CONF:DATA:MEAS:IPERf:CLI:IPAD "
        self.get_logger().debug("Setting the client IP address to %s"
                                % ip_address)
        self._visa.send_command(gpib_cmd + "'" + ip_address + "'")

    def get_iperf_result(self, target):
        """
        :type target: str
        :param target: defines if the client or server result will be returned
        :rtype: tuple
        :return: the throughput measured during the test.
        :raise TestEquipmentException: If one of the parameters is not in its range.
        """
        elapsed_time = 0
        timeout = 60
        # Checking the target parameter is in its range.
        if target.upper() == "SERVER":
            gpib_target_param = "SERV"
        elif target.upper() == "CLIENT":
            gpib_target_param = "CLI"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "The target should be either SERVER or CLIENT."
                                         " %s is not supported." % target.upper())
        state = self.get_iperf_measurement_state()
        # Waiting for the measurement on the equipment to be ready.
        while state != "RDY" and elapsed_time < timeout:
            self.get_logger().debug("Waiting for the IPERF measurement to be ready. Is %s" % state)
            time.sleep(1)
            elapsed_time += 1
            state = str(self.get_iperf_measurement_state())
        # Fetch the result from the equipment after arbitrary waiting time.
        time.sleep(3)
        gpib_command = "FETC:DATA:MEAS:IPER:%s?" % gpib_target_param
        result = self._visa.query_command(gpib_command)
        self.get_logger().debug("IPERF measurement results: %s" % result)
        split_result = result.split(",")
        meas_throughput = DuplexThroughputMeasure()
        # Getting the UL and DL throughput values and converting them in kbps.
        # They are returned by the CMW500 in bps. Result is comma separated
        # list of 8 results (server instance 1 to 8). We only activated the
        # first client or server so only the first result is relevant.
        if split_result[1].upper() not in ("NAV", "INV"):
            (unit, multiplier) = split_result[1].split("E")
            multiplier = multiplier[-1:]
            # Throughput value in bps
            value = float(unit) * 10 ** int(multiplier)
            if target.upper() == "SERVER":
                self.get_logger().debug("Adding %s bps as uplink throughput" % value)
                meas_throughput.ul_throughput.set(float(value) / 1000, ThroughputMeasure.KBPS_UNIT)
            else:
                self.get_logger().debug("Adding %s bps as downlink throughput" % value)
                meas_throughput.dl_throughput.set(float(value) / 1000, ThroughputMeasure.KBPS_UNIT)
        else:
            raise TestEquipmentException(AcsBaseException.OPERATION_FAILED, "Invalid throughput return by equipment.")
        return meas_throughput

    def get_iperf_measurement_state(self):
        """
        Queries the main measurement state. Use FETCh:...:STATe:ALL? to query
        the measurement state.
        :rtype: str
        :return: OFF | RUN | RDY
            OFF: measurement switched off, no resources allocated, no
                results available (when entered after ABORt...)
            RUN: measurement running (after INITiate..., READ...),
                synchronization pending or adjusted, resources active or queued
            RDY: measurement has been terminated, valid results may be
                available
        """
        gpib_command = "FETC:DATA:MEAS:IPER:STAT?"
        result = self._visa.query_command(gpib_command)
        self.get_logger().debug("Measurement state: %s" % result)
        return result

    def start_iperf_server(self, settings):
        return self.start_iperf_service()

    def stop_iperf_server(self):
        return self.abort_iperf_service()

    def start_iperf_client(self, settings):
        self.start_iperf_service()
        while self.get_iperf_measurement_state() == "RUN":
            time.sleep(10)
        return None

    def stop_iperf_client(self):
        return self.abort_iperf_service()

    # IP address and APN functions

    def set_apn(self, apn):
        """
        Set APN on Cellular network.

        :type apn: str
        :param apn: The Access Point Name to be configured on equipment

        :rtype: None
        """
        self.get_logger().info("Set APN on Cellular network is stubbed on CMW500")

    def dau_set_ip4_lan_address(self, ip_lan):
        """
        Sets IP LAN address in case IP attribution mode is static
        :param ip_lan:
        :return:
        """
        self.get_logger().info("Configuring IPV4 LAN address (STATIC mode): %s ..." % ip_lan)
        cmd = "CONF:DATA:CONT:IPVF:STAT:IPAD '%s'" % ip_lan
        self.start_dau(300)
        self._visa.send_command(cmd)

    def dau_set_ip4_default_gateway(self, gateway):
        """
        Sets default IPv4 gateway address
        :type gateway: str
        :param gateway: the gateway to set. 15 characters formatted
        as follows: A.B.C.D where A = 0 to 223 B,C,D = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4DefaultGateway driver function
        """
        self.get_logger().info("Configuring IPV4 gateway %s..." % gateway)
        cmd = "CONF:DATA:CONT:IPVF:STAT:GIP '%s'" % gateway
        self.start_dau(300)
        self._visa.send_command(cmd)

    def dau_set_ip4_subnet_mask(self, mask):
        """
        Sets IPv4 subnet mask
        :type mask: str
        :param mask: the subnet mask to set. 15 characters formatted
        as follows: A.B.C.D where A,B,C,D are between = 0 to 255
        (no embedded spaces).
        :raise TestEquipmentException: failed to call SetIp4SubnetMask driver function
        """
        self.get_logger().info("Configuring IPV4 subnet MASK %s..." % mask)
        cmd = "CONF:DATA:CONT:IPVF:STAT:SMAS '%s'" % mask
        self.start_dau(300)
        self._visa.send_command(cmd)

    def set_ip_addr_configuration_mode(self, mode, ip_type="ALL"):
        """
        Selects the method to be used for IP address configuration

        :type mode: str
        :param mode: STATIC - ip take from what has been manually defined,
                     AUTOMATIC - ip assignment from a default configuration
                     DYNAMIC - dynamic mode , DHCP for ipv4 or autoconfiguration for ipv6

        :type ip_type: str
        :param ip_type: the ip type, IPV4, IPV6 or dont fill it to force all type.
        """
        mode = mode.upper()
        ip_type = ip_type.upper()
        possible_mode = ["AUTO", "STATIC", "DYNAMIC"]
        possible_ip_type = ["IPV4", "IPV6", "ALL"]
        error_msg = ""
        if mode not in possible_mode:
            error_msg = "Unsupported mode %s for set_ip_mode_configuration, mode can only be in %s." % (mode, str(possible_mode))
        if ip_type not in possible_ip_type:
            error_msg += "Unsupported ip_type %s for set_ip_mode_configuration, ip_type can only be in %s." % (mode, str(possible_ip_type))

        if error_msg != "":
            self.get_logger().error(error_msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, error_msg)

        # need to check if DAU need to be start or not to set this
        self.start_dau(300)
        if ip_type in ["IPV4" , "ALL"]:
            possible_mode = {"AUTO": "AUT", "STATIC": "STAT", "DYNAMIC": "DHCP"}
            self.get_logger().info("Configuring IPV4 address mode in %s" % mode)
            self._visa.send_command("CONF:DATA:CONT:IPVF:ADDR:TYPE %s" % possible_mode[mode])

        if ip_type in ["IPV6" , "ALL"]:
            possible_mode = {"AUTO": "AUTO", "STATIC": "STAT", "DYNAMIC": "ACON"}
            self.get_logger().info("Configuring IPV6 address mode in %s" % mode)
            self._visa.send_command("CONF:DATA:CONT:IPVS:ADDR:TYPE %s" % possible_mode[mode])

    def set_dut_ip_address(self, ip_addr, dut_ip_addr_pool=[]):
        """
        Add ip_addr to the DUT IP address configuration of the DAU.
        Every DUT IP address previously configured that is not in dut_ip_addr_pool is removed

        :warning: this function has effect only if ip address attribution is set to static.
        :type ip_addr: str
        :param ip_addr: the IP address to set.
        :type dut_ip_addr_pool: list
        :param dut_ip_addr_pool: list of DUT IP address which have to be set in the DAU
        :raise TestEquipmentException: LTE network emulator is still enabled when this function is called
        """
        self.get_logger().info("Checking current signaling operational status...")
        status = self.get_su_state()
        is_ipv6 = False
        if ":" in ip_addr:
            is_ipv6 = True

        # Ensure DAU is ON
        self.start_dau(300)
        # Verify that provided IP address str is well formed as per SCPI requirements ("xxx.xxx.xxx.xxx")
        cfg_ip_addr = ip_addr.strip('\'"')

        if status == "OFF":
            if is_ipv6:
                # Query instrument for the list of current IPV6 addresses in the pool
                self.get_logger().info("Querying instrument for the list of current IPV6 prefix addresses in the pool...")
                self._visa.send_command("CONF:DATA:CONT:IPVS:MOB:PREF:TYPE STAT")
                ip_addresses_list = self._visa.query_command("SENS:DATA:CONT:IPVS:STAT:PREF:CAT?").split(",")
                for ip_address in ip_addresses_list:
                    self.get_logger().info(ip_address)

                # Remove previously configured IPV6 addresses from pool if needed
                self.get_logger().info("Removing previously configured IPV6 prefix addresses from pool...")
                for ip_address in ip_addresses_list:
                    if ip_address.strip('\'"') not in dut_ip_addr_pool:
                        time.sleep(1)
                        self._visa.send_command("CONF:DATA:CONT:IPVS:STAT:PREF:DEL %s" % ip_address)

                ip_addresses_list = [tmp.strip('\'"') for tmp in ip_addresses_list]

                # Add IPV6 address ip_addr to the pool
                if cfg_ip_addr not in ip_addresses_list:
                    time.sleep(1)
                    self.get_logger().info("Configuring the pool to only contain IPV6 prefix address %s..." % cfg_ip_addr)
                    self._visa.send_command("CONF:DATA:CONT:IPVS:STAT:PREF:ADD '%s'" % cfg_ip_addr)
            else:
                # Query instrument for the list of current IPV4 addresses in the pool
                self.get_logger().info("Querying instrument for the list of current IPV4 addresses in the pool...")
                ip_addresses_list = self._visa.query_command("SENS:DATA:CONT:IPVF:STAT:ADDR:CAT?").split(",")
                for ip_address in ip_addresses_list:
                    self.get_logger().info(ip_address)

                # Remove previously configured IPV4 addresses from pool if needed
                self.get_logger().info("Removing previously configured IPV4 addresses from pool if needed...")
                for ip_address in ip_addresses_list:
                    if ip_address.strip('\'"') not in dut_ip_addr_pool:
                        time.sleep(1)
                        self._visa.send_command("CONF:DATA:CONT:IPVF:STAT:ADDR:DEL %s" % ip_address)

                ip_addresses_list = [tmp.strip('\'"') for tmp in ip_addresses_list]

                # Add IPV4 address ip_addr to the pool
                if cfg_ip_addr not in ip_addresses_list:
                    time.sleep(1)
                    self.get_logger().info("Adding IPV4 address %s to the pool..." % cfg_ip_addr)
                    self._visa.send_command("CONF:DATA:CONT:IPVF:STAT:ADDR:ADD '%s'" % cfg_ip_addr)

            self.get_logger().info(
                "Configuring the system PING destination IP address to match DUT assigned IP address...")
            self._visa.send_command("CONF:DATA:MEAS:PING:DIP '%s'" % cfg_ip_addr)
        # This is to force PINGs generated by the DAU to be sent to DUT IP address
        else:
            msg = "Signaling unit must be disabled before modifying DUT IP configuration!!!"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def set_ip6_network_parameters(self, server_ipv6_address):
        """
        This command set the network parameters for IPv6.
        :type server_ipv6_address: str
        :param server_ipv6_address: IPv6 server address composed by:
        IPv6 Prefix and IPv6 ID
        """
        self.get_logger().info("Configure DAU IPV6 address to %s" % server_ipv6_address)
        self._visa.send_command("CONFigure:DATA:CONTrol:IPVSix:ADDRess:TYPE STATic")
        self._visa.send_command("CONFigure:DATA:CONTrol:IPVSix:STATic:ADDRess '%s'" % server_ipv6_address)

    # DATA connection status functions

    def get_data_connection_status(self, cell_id):
        """
        GetDataConnectionStatus
        :param cell_id: id of the cell for which the status is being requested
        :type cell_id: str
        :return:
            - integer: error code of the driver function
            - str: the data connection status. Possible returned values:
                - "OFF": Default returned value
                - "IDLE"
                - "CON"
                - "REG"
                - "LOOP"
                - "REL"
                - "UNAV"
                - "UNAV" => Default returned value
        :raise TestEquipmentException: if cell_id value is incorrect

        [JPHC, 28 juillet 2012]
        Need to retrieve current connection status from R&S (much finer detail reported and translate to the PXT language for reporting back to UC)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_data_connection_state(self, state, timeout=0, blocking=True, cell_id=None):
        """
        Checks that the data connection is set at the required state
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
                - "OFF": Default returned value
                - "IDLE"
                - "CON"
                - "REG"
                - "LOOP"
                - "REL"
                - "UNAV"
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        :type blocking: boolean
        :param blocking: boolean to know if the function raises an error
        or simply return true or false if the status is reached or not
        :type cell_id : str
        :param cell_id: cell used for the test. Possible values:
            - "A"
            - "B"
        .. warning:: This parameter is only used in 4G (LTE)
        :rtype: boolean
        :return: True if state is reached, else returns False
        :raise TestEquipmentException: if expected state is not reached before timeout
        """
        registered = False
        idle = False
        timer = timeout
        self.get_logger().info(
            "Check data connection is %s before %d seconds",
            state,
            timeout)

        current_state = self.get_data_connection_status(cell_id)
        self.get_logger().debug(
            "Data connection current status is %s",
            current_state)

        while (timer > 0) and (current_state != state):
            if state == "REG" and current_state == "IDLE":
                # Configure Mobile Terminated Connection...
                self.ue_connection()
                registered = True
                break
            elif state == "IDLE" and current_state == "CON":
                idle = True
                break
            # time.sleep(1)
            time.sleep(10)  # Big timeout value to enable debugging while operating CMW500 in local mode
            current_state = self.get_data_connection_status(cell_id)
            timer -= 10

        if current_state != state and not registered and not idle:
            if blocking:
                # Failed to reach desired state
                msg = "Failed to reach %s data state!" % state
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)
            else:
                # Failed to reach desired state (Test failed no TestEquipmentException raised)
                self.get_logger().error("Failed to reach %s data state!", state)

            return False
        else:
            self.get_logger().info("Data connection status %s and has been reached in %d sec", current_state, timeout - timer)
            return True

    # IMS functions

    def check_ims_connection_state(self, state, timeout=0):
        """
        Checks that the ims connection is set to the required state
        before the given timeout. If timeout is <= 0, only one test is performed.
        :raise TestEquipmentException: the required status has not been reached before the timeout
        :type state: str
        :param state: the expected state. Possible values:
                - "UNR": Default returned value
                - "REG"
        :type timeout: integer
        :param timeout: allowed time to reach expected state
        :rtype: boolean
        :return: True if state is reached, else returns False
        """
        timer = timeout
        self.get_logger().info(
            "Check IMS connection is %s before %d seconds",
            state,
            timeout)

        status = False

        while (timer > 0):
            current_state = self._visa.query_command("SENSe:DATA:CONTrol:IMS2:MOBile:STATus?")
            if state == current_state:
                if state == "REG":
                    self.get_logger().info("IMS registration successfully reached")
                    status = True
                    break
                elif state == "UNR":
                    self.get_logger().info("IMS unregistration successfully reached")
                    status = True
                    break
                else:
                    self.get_logger().info("IMS state unknown")
                    status = False
                    break
            time.sleep(1)
            timer -= 1

        if not status:
            # Failed to reach desired state
            msg = "Failed to reach %s IMS state after %s seconds !" % (state, str(timeout))
            self.get_logger().error(msg)
            # raise TestEquipmentException(TestEquipmentException.TIMEOUT_REACHED, msg)

        return status

    def unreg_ims(self):
        """
        Unregister DUT from IMS service
        """
        self.get_logger().info("Unregister DUT from IMS service")
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS2:MOBile:DERegister")

    def set_ims_ip_version(self, ip_version):
        """
        Set IMS IP Version.

        :type ip_version: str
        :param ip_version: IPV4 | IPV6 | IPV4V6

        :rtype: None
        """
        self.get_logger().info("Setting IMS IP Version to %s" % ip_version)
        if ip_version == "IPV4":
            ip_version = "IPVFour"
        else:
            ip_version = "IPVSix"
        self._visa.send_command("CONFigure:DATA:CONTrol:IMS:INTern:PCSCf:ATYPe %s" % ip_version)
