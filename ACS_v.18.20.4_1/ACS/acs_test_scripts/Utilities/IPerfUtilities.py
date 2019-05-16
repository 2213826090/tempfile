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
:summary: Utilities classes and function for Data
:since: 30/08/2010
:author: cco
"""

from Queue import Empty
import re
import subprocess
import time
import random
import threading

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global, run_local_command
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException

from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure, DuplexThroughputMeasure
from acs_test_scripts.Utilities.NetworkingUtilities import wait_for_dut_ipv4_address


class Iperf(object):
    """
    interface class for Iperf enabled objects
    objects having Iperf capabilities MUST inherit from this one
    """
    def __init__(self):
        self._iperf_cmd = None

    def clean_iperf_env(self):
        """
        Do a clean action on iperf like cleaning generated file
        and stopping running server.
        this is designed to be used directly on usecase instead of here.
        """
        pass

    def start_iperf_server(self, settings):
        """
        Start Iperf server

        :type settings: dict
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration
        """
        pass

    def stop_iperf_server(self):
        """
        Stops Iperf server

        :rtype: str
        :return: Iperf server execution log
        """
        pass

    def start_iperf_client(self, settings):
        """
        Start Iperf client

        :type settings: dict
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration

        :rtype: str
        :return: measured throughput value
        """
        pass

    def stop_iperf_client(self):
        """
        Stop Iperf client

        :rtype: str
        :return: measured throughput value
        """
        pass

    def _build_iperf_command(self, settings, role):
        """

        """
        iperf = settings.get('iperf', 'iperf')
        args = IperfOptions(settings).format_iperf_options(role)
        self._iperf_cmd = iperf + " " + args
        return self._iperf_cmd


class Execution_thread(threading.Thread):
    """
    Class that launch a separated thread
    """
    def __init__(self, target, args=()):
        threading.Thread.__init__(self)
        self._exec_output = ""
        self._target = target
        self._args = args

    @property
    def exec_output(self):
        """
        Return std out
        :rtype: str
        :return: measured BLER
        """
        return self._exec_output

    def run(self):
        """
        Perform run
        """
        self._exec_output = self._target(*self._args)


class IperfOptions:

    """
    This class contains all the options that will be contains in the iperf command
    """

    #
    # Definition of iperf options mainly used by ACS     #
    #
    # Defining dictionaries of options containing specific key
    # with the iperf arg and boolean indicating if a value is needed

    # Client / Server options:
    COMMON_OPTIONS = \
        {"format": ("-f", True),  # -f, --format    [kmKM]   format to report: Kbits, Mbits, KBytes, MBytes
         "interval": ("-i", True),  # -i, --interval  #        seconds between periodic bandwidth reports
         "buffer_length": ("-l", True),  # -l, --len       #[KM]    length of buffer to read or write (default 8 KB)
         "print_mss": ("-m", False),  # -m, --print_mss          print TCP maximum segment size (MTU - TCP/IP header)
         "output_file": ("-o", True),  # -o, --output    <filename> output the report or error message to this specified file
         "port_number": ("-p", True),  # -p, --port      #        server port to listen on/connect to
         "udp": ("-u", False),  # -u, --udp                use UDP rather than TCP
         "compatibility": ("-C", False),  # -C, --compatibility      for use with older versions does not sent extra msgs
         "mss_size": ("-M", True),  # -M, --mss       #        set TCP maximum segment size (MTU - 40 bytes)
         "no_delay": ("-N", False),  # -N, --nodelay            set TCP no delay, disabling Nagle's Algorithm
         "ipv6_domain": ("-V", False)  # -V, --IPv6Version        Set the domain to IPv6
         }
    """
    List of common iperf options available for server and client sides :
        -f, --format    [kmKM]   format to report: Kbits, Mbits, KBytes, MBytes
        -i, --interval  #        seconds between periodic bandwidth reports
        -l, --len       #[KM]    length of buffer to read or write (default 8 KB)
        -m, --print_mss          print TCP maximum segment size (MTU - TCP/IP header)
        -o, --output    <filename> output the report or error message to this specified file
        -p, --port      #        server port to listen on/connect to
        -u, --udp                use UDP rather than TCP
        -B, --bind      <host>   bind to <host>, an interface or multicast address
        -C, --compatibility      for use with older versions does not sent extra msgs
        -M, --mss       #        set TCP maximum segment size (MTU - 40 bytes)
        -N, --nodelay            set TCP no delay, disabling Nagle's Algorithm
        -V, --IPv6Version        Set the domain to IPv6
    """

    # Server specific:
    SERVER_OPTIONS = \
        {"single_udp": ("-U", False),  # -U, --single_udp     run in single threaded UDP mode
         "server_dameon": ("-D", False),  # -D, --daemon      run the server as a daemon
         "server_window_size": ("-w", True)  # -w, --window  TCP window size (socket buffer size)
         }
    """
    List of iperf options when launching as server :
        -U, --single_udp         run in single threaded UDP mode
        -D, --daemon             run the server as a daemon
        -w, --window    #[KM]    TCP window size (socket buffer size)
    """

    # Client specific
    CLIENT_OPTIONS = \
        {"bandwidth": ("-b", True),  # -b, --bandwidth                       for UDP, bandwidth to send at in bits/sec (default 1 Mbit/sec, implies -u)
         "dual_test": ("-d", False),  # -d, --dualtest                       Do a bidirectional test simultaneously
         "nb_bytes": ("-n", True),  # -n, --num                              number of bytes to transmit (instead of -t)
         "tradeoff": ("-r", False),  # -r, --tradeoff                        Do a bidirectional test individually
         "duration": ("-t", True),  # -t, --time                             time in seconds to transmit for (default 10 secs)
         "bind_host": ("-B", True),  # -B, --bind <host>                     bind to <host>, an interface or multicast address
         "parallel_thread": ("-P", True),  # -P, --parallel                  number of parallel client threads to run
         "file_input": ("-F", True),  # -F, --fileinput <name>               input the data to be transmitted from a file
         "stdin": ("-I", True),  # -I, --stdin                               input the data to be transmitted from stdin
         "listen_port": ("-L", True),  # -L, --listenport                    port to recieve bidirectional tests back on
         "ttl": ("-T", True),  # -T, --ttl                                   time-to-live, for multicast (default 1)
         "linux_congestion": ("-Z", True),  # -Z, --linux-congestion <algo>  set TCP congestion control algorithm (Linux only)
         "client_window_size": ("-w", True)  # -w, --window                  TCP window size (socket buffer size)
         }
    """
    List of iperf options when launching as client :
        -b, --bandwidth #[KM]    for UDP, bandwidth to send at in bits/sec
                                 (default 1 Mbit/sec, implies -u)
        -c, --client    <host>   run in client mode, connecting to <host>
        -d, --dualtest           Do a bidirectional test simultaneously
        -n, --num       #[KM]    number of bytes to transmit (instead of -t)
        -r, --tradeoff           Do a bidirectional test individually
        -t, --time      #        time in seconds to transmit for (default 10 secs)
        -F, --fileinput <name>   input the data to be transmitted from a file
        -I, --stdin              input the data to be transmitted from stdin
        -L, --listenport #       port to recieve bidirectional tests back on
        -P, --parallel  #        number of parallel client threads to run
        -T, --ttl       #        time-to-live, for multicast (default 1)
        -Z, --linux-congestion <algo>  set TCP congestion control algorithm (Linux only)
        -w, --window    #[KM]    TCP window size (socket buffer size)
    """

    MANDATORY_OPTIONS = {
        "duration": (int, None),
        "protocol": (str, ["udp", "tcp"]),
        "server_ip_address": (str, None),
        "port_number": (int, None),
        "direction": (str, ["up", "down", "both"])
    }

    def __init__(self, options):
        """
        Initialization of the iperf structure

        :type options: dictionary
        :param options: Containing all options used to launch the iperf command
        Based on iperf --help options description.
        All options
        """
        self.options = options

    def __format_common_options(self):
        """
        Return a str of common iperf args from user settings

        :rtype: list
        :return: Formatted list str of common iperf args
        """
        iperf_command = []

        protocol = self.options.get('protocol')

        if protocol == "udp":
            udp_arg = IperfOptions.COMMON_OPTIONS.get('udp')[0]
            iperf_command.append(udp_arg)

        for option in self.options.keys():
            if option == "no_delay" and protocol != "tcp":
                # This option available only on TCP
                continue

            if option in IperfOptions.COMMON_OPTIONS.keys():
                # Retrieve iperf arg
                (iperf_arg, need_value) = IperfOptions.COMMON_OPTIONS.get(option)
                iperf_command.append(iperf_arg)
                if need_value:
                    iperf_arg_value = str(self.options.get(option))
                    iperf_command.append(iperf_arg_value)

                # Prune this option from the extra options parameter
                self.__clean_extra_iperf_options(iperf_arg)

        # Add extra options
        if "extra_options" in self.options.keys():
            extra_options = self.options.get("extra_options")
            iperf_command.append(extra_options)

        return iperf_command

    def __format_server_options(self):
        """
        Return a str of server iperf args from user settings

        :rtype: list
        :return: Formatted list str of common iperf args
        """
        iperf_command = ['-s']
        protocol = self.options.get('protocol')

        for option in self.options.keys():
            if "window_size" in option and protocol != "tcp":
                # This option available only on TCP
                continue
            if option in IperfOptions.SERVER_OPTIONS.keys():
                # Retrieve iperf arg
                (iperf_arg, need_value) = IperfOptions.SERVER_OPTIONS.get(option)
                iperf_command.append(iperf_arg)
                if need_value:
                    iperf_arg_value = str(self.options.get(option))
                    iperf_command.append(iperf_arg_value)

                # Prune this option from the extra options parameter
                self.__clean_extra_iperf_options(iperf_arg)

        return iperf_command

    def __format_client_options(self):
        """
        Return a str of client iperf args from user settings

        :rtype: list
        :return: Formatted list str of common iperf args
        """

        # Add -c option with the host
        iperf_command = ['-c', self.options.get('server_ip_address')]
        protocol = self.options.get('protocol')
        direction = self.options.get('direction', 'both')

        for option, value in self.options.items():
            # Manage specific case
            if option == "bandwidth" and protocol != "udp":
                # This option available only on UDP
                continue
            if option == "tradeoff" and direction != "both":
                # This option available only for bidirectional test
                continue
            if "window_size" in option and protocol != "tcp":
                # This option available only on TCP
                continue
            # Do not add parallel thread option if number of parallel thread is 0 or 1
            if option == "parallel_thread" and int(value) in (0, 1):
                continue

            if option in IperfOptions.CLIENT_OPTIONS.keys():
                # Retrieve iperf arg
                (iperf_arg, need_value) = IperfOptions.CLIENT_OPTIONS.get(option)

                iperf_command.append(iperf_arg)
                if need_value:
                    iperf_arg_value = str(value)
                    iperf_command.append(iperf_arg_value)

                # Prune this option from the extra options parameter
                self.__clean_extra_iperf_options(iperf_arg)

        return iperf_command

    def __clean_extra_iperf_options(self, iperf_arg):
        """
        Function to prune the iperf option that has been just handled,
        from the iperf extra option str

        :type iperf_arg: str
        :param iperf_arg: argument name of the iperf option to remove. Ex: "-P"
        """
        extra_options = self.options.get("extra_options")

        if extra_options is not None and iperf_arg in extra_options:
            extra_options = re.sub(iperf_arg + "[ ]*[^- ]*", "", extra_options)
            self.options.update({"extra_options": extra_options.strip()})

    def format_iperf_options(self, mode="client"):
        """
        This function aims at building the iperf option to launch

        :type mode: str
        :param mode: Specify if the command is server or client specific

        :rtype: str
        :return: the iperf options
        """

        iperf_command = []
        if mode.lower() == "server":
            iperf_command += self.__format_server_options()
        else:
            iperf_command += self.__format_client_options()

        iperf_command += self.__format_common_options()

        return ' '.join(iperf_command)

    def ckeck_for_mandatory_options(self):
        """
        check iperf mandatory options and raise error if they does not match
        change the options cast to match with the accepted one too
        """
        error_msg = []
        for opt in self.MANDATORY_OPTIONS.keys():
            if not (opt in self.options.keys()):
                error_msg.append("'%s' mandatory option is missing from settings." % opt)

            else:
                # data from ref
                m_opt_type, m_opt_range = self.MANDATORY_OPTIONS[opt]
                # data from target
                s_opt_value = self.options[opt]
                s_opt_type = type(self.options[opt])
                # force value to be compared as lowercase
                if s_opt_type is str:
                    s_opt_value = s_opt_value.lower()
                # check type
                # TODO: this way to compare may cause problem if we compare int to float
                if not (s_opt_type is m_opt_type):
                    error_msg.append("'%s' option: wrong type. expecting '%s', got '%s'." %
                                    (opt, m_opt_type, s_opt_type))

                # check value
                if not m_opt_range is None :
                    if not (s_opt_value in m_opt_range):
                        error_msg.append("'%s' option: wrong value. expecting one of %s, got '%s'." %
                                         (opt, m_opt_range, s_opt_value))
                    # force the option to become lower case
                    # TODO : THIS CHANGE MAY BE REVERTED AFTER A CHECK WITH IPERF DEV
                    elif s_opt_type is str:
                        self.options[opt] = s_opt_value

        if len(error_msg) > 0:
            msg = "Error happen with Iperf settings:\n"
            msg += "\n".join(error_msg)
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

def compute_iperf_verdict(meas_throughput,
                          target_throughput,
                          direction=None,
                          bler=0.0):
    """
    Returns the verdict comparing target UL/DL and failure UL/DL
    throughput measurements with UL/DL IPERF measurements.

    :type meas_throughput: DuplexMeasureThroughput
    :param meas_throughput: Measured Throughput
    :type target_throughput: MeasureThroughputTarget
    :param target_throughput: Target Throughput
    :type direction: str
    :param direction: (up|down)
    :type bler: float
    :type bler: Measured bler

    :rtype: list
    :return: the computed verdict: (<PASS/FAIL>, <error_message>)
    """
    result_verdict = Global.SUCCESS
    result_msg = ""
    # if target_throughput is none, IPERF values can't be compared.
    if target_throughput is None:
        result_msg += "UL:%s DL:%s. " \
            % (meas_throughput.ul_throughput,
               meas_throughput.dl_throughput)

        result_msg += \
            "Measured value(s) cannot be compared " + \
            "(no Target and Failure values)."
        return Global.SUCCESS, result_msg

    # convert units if needed
    if direction != "up" and meas_throughput.dl_throughput.value >= 0:
        target_throughput.convert_downlink_to(meas_throughput.dl_throughput.unit)
    if direction != "down" and meas_throughput.ul_throughput.value >= 0:
        target_throughput.convert_uplink_to(meas_throughput.ul_throughput.unit)

    # Compute verdict depending on throughputs
    if direction != "up" and meas_throughput.dl_throughput < target_throughput.dl_failure:
        result_verdict = Global.FAILURE
    elif direction != "down" and meas_throughput.ul_throughput < target_throughput.ul_failure:
        result_verdict = Global.FAILURE
    # Check if BLER is ok
    if bler > target_throughput.bler:
        result_verdict = Global.FAILURE

    warning = ""
    if direction != "up" and meas_throughput.dl_throughput > target_throughput.dl_target:
        warning = "DL"
    if direction != "down" and meas_throughput.ul_throughput > target_throughput.ul_target:
        if warning != "":
            warning += " and UL"
        else:
            warning += "UL"
    if warning != "":
        warning += " measured value(s) higher than target value(s). "

    if direction != "up" and meas_throughput.dl_throughput.value < 0:
        warning += "DL datas have not been parsed."
    elif direction != "down" and meas_throughput.ul_throughput.value < 0:
        warning += "UL datas have not been parsed."
    # Both cases is not possible

    if warning != "":
        warning = "Warning: " + warning

    if direction == "up":
        result_msg += \
            "UL:%s " \
            % meas_throughput.ul_throughput
        result_msg += "(Targets UL:%s, "\
                      % target_throughput.ul_target
        result_msg += "Failures UL:%s) " \
            % target_throughput.ul_failure
    elif direction == "down":
        result_msg += \
            "DL:%s " \
            % meas_throughput.dl_throughput
        result_msg += "(Targets DL:%s, " \
            % target_throughput.dl_target
        result_msg += "Failures DL:%s) " \
            % target_throughput.dl_failure
    else:
        result_msg += \
            "UL:%s DL:%s " \
            % (meas_throughput.ul_throughput,
               meas_throughput.dl_throughput)
        result_msg += "(Targets UL:%s DL:%s, " \
            % (target_throughput.ul_target,
               target_throughput.dl_target.value)
        result_msg += "Failures UL:%s DL:%s) " \
            % (target_throughput.ul_failure,
               target_throughput.dl_failure)

    result_msg += "BLER: (Measured: %.2f%% Failure: %.2f%%)" % (bler, target_throughput.bler)

    result_msg += warning

    return result_verdict, result_msg


def get_iperf_configuration(target_throughput, direction=None):
    """
    This function will return an iperf configuration in function of the target throughput to reach
    The iperf configuration will depends on the greater value between DL and UL throughput

    :type target_throughput: MeasureThroughputTargets
    :param target_throughput: UL/DL target throughput

    :type direction: str
    :param direction: direction of the iperf test ("up"|"down"|None = up/down)

    :rtype: iperf configuration structure
    :return: iperf configuration
    """

    # Convert in kbps to be more deterministic
    target_throughput.convert_downlink_to("kbps")
    target_throughput.convert_uplink_to("kbps")

    # Determine the max throughput reference between DL and UL
    if direction in (None, "None"):
        # Initialize default parameter for iperf
        iperf_configuration = {"no_delay": '',
                               "tradeoff": ''}
        if target_throughput.dl_target_value < target_throughput.ul_target_value:
            throughput_ref = target_throughput.ul_target_value
        else:
            throughput_ref = target_throughput.dl_target_value
    elif direction == "up":
        throughput_ref = target_throughput.ul_target_value
    elif direction == "down":
        throughput_ref = target_throughput.dl_target_value

    # Sets the iperf configuration depending on the throughput reference
    if 0 < throughput_ref <= 64:
        iperf_configuration.update(
            {"parallel_thread": 1, "buffer_length": 24})

    elif 64 < throughput_ref <= 128:
        iperf_configuration.update(
            {"parallel_thread": 1, "buffer_length": 32})

    elif 128 < throughput_ref <= 192:
        iperf_configuration.update(
            {"parallel_thread": 1, "buffer_length": 56})

    elif 192 < throughput_ref <= 256:
        iperf_configuration.update(
            {"parallel_thread": 1, "buffer_length": 84})

    elif 256 < throughput_ref <= 320:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 84})

    elif 320 < throughput_ref <= 384:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 128})

    elif 384 < throughput_ref <= 512:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 128})

    elif 512 < throughput_ref <= 1024:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 512})

    elif 1024 < throughput_ref <= 2500:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 1024, "client_window_size": "64K", "server_window_size": "64K", "bandwidth": "2500K"})

    elif 2500 < throughput_ref <= 5000:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 2048, "client_window_size": "64K", "server_window_size": "64K", "bandwidth": "5000K"})

    elif 5000 < throughput_ref <= 7500:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 4096, "client_window_size": "64K", "server_window_size": "64K", "bandwidth": "7500K"})

    elif 7500 < throughput_ref <= 25000:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 8192, "client_window_size": "64K", "server_window_size": "64K", "bandwidth": "25000K"})

    elif 25000 < throughput_ref <= 54000:
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 10240, "client_window_size": "512K", "server_window_size": "512K", "bandwidth": "54000K"})

    else:
        # Log a warning in case the throughput is not characterized
        warning_msg = "get_iperf_configuration: "
        warning_msg += \
            "iperf options are not defined for throughput %s kbps. " \
            % str(throughput_ref)
        warning_msg += "Using default values (-l 10240 -P 2 -N -r)."
        LOGGER_TEST_SCRIPT.warning(warning_msg)
        iperf_configuration.update(
            {"parallel_thread": 2, "buffer_length": 10240, "client_window_size": "64K", "server_window_size": "64K"})
    LOGGER_TEST_SCRIPT.info("iperf settings %s" % iperf_configuration)
    return iperf_configuration


def compute_iperf_bw(options, throughtput_target=None, iperf_bandwidth=None):
    """
    Compute iperf ul and dl bandwidth depending of the throughput wanted.
    and update dictionnary with these values

    :type options: dict
    :param options: dictionnary in which iperf bandwitdh is stored

    :type throughtput_target: throughtput_target
    :param throughtput_target: target throughput value for ul and dl

    :type iperf_bandwidth: str
    :param iperf_bandwidth: redefined iperf bandwitdh

    """
    if throughtput_target is None:
        dl_target_value = 0
        ul_target_value = 0
    else:
        throughtput_target.dl_target.convert_to(ThroughputMeasure.KBPS_UNIT)
        throughtput_target.ul_target.convert_to(ThroughputMeasure.KBPS_UNIT)
        dl_target_value = throughtput_target.dl_target.value
        ul_target_value = throughtput_target.ul_target.value

    options["dl_bandwidth"] = str(int(dl_target_value) + 100) + 'K'

    if options["direction"].upper() == "DOWN":
        options["bandwidth"] = options["dl_bandwidth"]
    else:
        if iperf_bandwidth in (None, "", "NONE"):
            options["bandwidth"] = str(int(ul_target_value) + 100) + 'K'
        else:
            # Check the unit of the bandwidth for the IPERF command line.
            if iperf_bandwidth[-1] not in ["K", "M"]:
                msg = "The IPERF bandwidth parameter should end with a K or a M."
                LOGGER_TEST_SCRIPT.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            else:
                options["bandwidth"] = iperf_bandwidth


def parse_iperf_options(iperf_options):
    """
    This function parses the iperf options passed as parameter and returns a
    dictonnary that contains known options.
    The dictionnary returned also contains a iperf_options key that give the
    value of the unknown parameters

    :type iperf_options: str
    :param iperf_options: Iperf options to parse

    :return: a dictionnary that contains iperf known parameters
    """
    iperf_configuration = {}

    remaining_options = iperf_options

    # Parses the network port
    m = re.search("(-p[ ]*)([0-9]+)", remaining_options)
    if m is None:
        # "-p port" option is mandatory
        msg = "Port option is missing in IPERF_OPTIONS tc parameter (-p)"
        LOGGER_TEST_SCRIPT.error(msg)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
    port = int(m.group(2))
    iperf_configuration.update({"port_number": port})
    i = remaining_options.find(m.group(1) + m.group(2))
    remaining_options = remaining_options[:i] + \
        remaining_options[i + len(m.group(1)) + len(m.group(2)):]

    # Parses the duration option
    m = re.search("(-t[ ]*)([0-9]+)", remaining_options)
    if m is None:
        # "-t duration" option is mandatory
        msg = "Duration option is missing in IPERF_OPTIONS tc parameter (-t)"
        LOGGER_TEST_SCRIPT.error(msg)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
    duration = int(m.group(2))
    iperf_configuration.update({"duration": duration})
    # Use Interval, set to a value just above duration, in order not to split the report and be able to parse it easily
    iperf_configuration.update({"interval": duration + 1})
    i = remaining_options.find(m.group(1) + m.group(2))
    remaining_options = remaining_options[:i] + \
        remaining_options[i + len(m.group(1)) + len(m.group(2)):]

    # Parses the protocol option
    if "-u" in remaining_options:
        iperf_configuration.update({"protocol": "udp"})
        i = remaining_options.find("-u")
        remaining_options = remaining_options[:i] + remaining_options[i + 2:]
    else:
        iperf_configuration.update({"protocol": "tcp"})

    # Parses the number of threads
    if "-P" in remaining_options:
        m = re.search("(-P[ ]*)([0-9]+)", remaining_options)
        if m is None:
            msg = "Unable to parse the numbler of iperf threads. " + \
                "Please, check IPERF_OPTIONS testcase parameter"
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        nb_threads = int(m.group(2))
        iperf_configuration.update({"parallel_thread": nb_threads})
        i = remaining_options.find(m.group(1) + m.group(2))
        remaining_options = remaining_options[:i] + \
            remaining_options[i + len(m.group(1)) + len(m.group(2)):]

    # Parses the buffer size
    if "-w" in remaining_options:
        m = re.search("(-w[ ]*)([0-9]+[KMkm]?)", remaining_options)
        if m is None:
            msg = "Unable to parse the window size in iperf options. " + \
                "Please, check IPERF_OPTIONS testcase parameter. " + \
                "You should write something like -w 256K or -w 1M"
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        ws = m.group(2)
        iperf_configuration.update({"server_window_size": ws, "client_window_size": ws})
        i = remaining_options.find(m.group(1) + m.group(2))
        remaining_options = remaining_options[:i] + \
            remaining_options[i + len(m.group(1)) + len(m.group(2)):]

    # Parses the bandwidth
    if "-b" in remaining_options:
        m = re.search("(-b[ ]*)([0-9]+[KMkm]?)", remaining_options)
        if m is None:
            msg = "Unable to parse the UDP bandwidth in iperf options. " + \
                "Please, check IPERF_OPTIONS testcase parameter. " + \
                "You should write something like -b 256K or -b 54M"
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        bandwidth = m.group(2)
        iperf_configuration.update({"bandwidth": bandwidth})
        i = remaining_options.find(m.group(1) + m.group(2))
        remaining_options = remaining_options[:i] + \
            remaining_options[i + len(m.group(1)) + len(m.group(2)):]

    # Parses the buffer length
    if "-l" in remaining_options:
        m = re.search("(-l[ ]*)([0-9]+[KMkm]?)", remaining_options)
        if m is None:
            msg = "Unable to parse the buffer length in iperf options. " + \
                "Please, check IPERF_OPTIONS testcase parameter. " + \
                "You should write something like -l 1500 or -l 8K"
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        length = m.group(2)
        iperf_configuration.update({"buffer_length": length})
        i = remaining_options.find(m.group(1) + m.group(2))
        remaining_options = remaining_options[:i] + \
            remaining_options[i + len(m.group(1)) + len(m.group(2)):]

    # Parses the TCP no delay option
    if "-N" in remaining_options:
        iperf_configuration.update({"no_delay": ''})
        i = remaining_options.find("-N")
        remaining_options = remaining_options[:i] + remaining_options[i + 2:]
    else:
        iperf_configuration.pop("no_delay", None)

    # Add the non-parsed Iperf options in the iperf configuration array
    iperf_configuration.update({"extra_options": remaining_options.strip()})

    return iperf_configuration


def set_iperf_ip_settings(settings, direction, computer, networking_api, cellular_network_interface, timeout):
    """
    Search for mandatory settings

    :type settings: dict
    :param settings:  dictionary containing all parameters

    :type direction: str
    :param direction:  iperf test direction

    :type computer: object
    :param computer: computer object on which iperf distant server is located

    :type network_api: object
    :param network_api: instance of network api object to get IPV4 address

    :type network_interface: str
    :param network_interface: interface name (wlan0/wlan1 etc...)

    :type timeout: int
    :param timeout: maximum time to wait for IPV4 address
    """
    # get DUT IP address.
    dut_ip_address = wait_for_dut_ipv4_address(timeout, networking_api, cellular_network_interface)
    if direction == 'down':
        # Downlink: connect from host to DUT, get DUT IP address.
        settings.update({"server_ip_address": dut_ip_address})
    elif computer is not None:
        # Uplink/both: connect from DUT to host, get computer IP address.
        # if computer is None or localhost, use WIFI_SERVER from Bench_Config
        ip = computer.get_host_on_test_network()
        LOGGER_TEST_SCRIPT.info("computer address %s" % ip)
        if ip not in ("localhost", "127.0.0.1"):
            settings.update({"server_ip_address": ip})
        # On windows DUT platform, we must set the outbound address
        # (because ethernet and wlan connections are connected at the same time
        # and to the same network)
        settings.update({"bind_host": dut_ip_address})


def retrieve_parameters_from_tc_params(tc_params):
    """
    This function parses TC parameters to get window size and paralell threads parameters needed by iperf:
        - WINDOW_SIZE: DL:val1,UL:val2 and/or
        - SERVER_WINDOW_SIZE: DL:val3,UL:val4
    note that DL:val1 or UL:val2 only val1 is possible

    The dictionnary returned is of format:
        {"dl_client_window_size": val1, "ul_client_window_size": val2,
         "dl_server_window_size": val3, "ul_server_window_size": val4}

    :type tc_params: object
    :param tc_params: TC parameters

    :return: a dictionnary that contains iperf window size parameters
    """
    # Get window size parameters from TC parameters
    window_size = tc_params.get_param_value("WINDOW_SIZE", "")
    server_window_size = tc_params.get_param_value("SERVER_WINDOW_SIZE", "")
    parallel_threads = tc_params.get_param_value("PARALLEL_THREAD", 1)
    parameter_dict = {}

    if server_window_size == "":
        server_window_size = window_size

    parameter_dict.update(retrieve_uldl_parameter(window_size, "_client_window_size"))
    parameter_dict.update(retrieve_uldl_parameter(server_window_size, "_server_window_size"))

    if parallel_threads not in (0, 1, "0", "1"):
        parameter_dict.update(retrieve_uldl_parameter(parallel_threads, "_parallel_thread"))

    return parameter_dict


def retrieve_uldl_parameter(parameter, end_string):
    """
    This function parses a window size or parallel thread parameter string and return a dictionnnary:
        DL:val1,UL:val2 and/or

    note that DL:val1 or UL:val2 only val1 is possible

    The dictionnary returned is of format:
        {"dl" + end_string: val1, "ul" + end_string: val2} or {}

    :type parameter: str
    :param parameter: parameter string

    :type end_string: str
    :param end_string: _client_window_size or _server_window_size

    :return: a dictionnary that contains iperf window size parameters
    """
    ws_dict = {}
    if parameter != "":
        if "DL" in parameter or "UL" in parameter:
            # Transform DL:val1,UL:val2 in {"dl" + end_string: val1, "ul" + end_string: val2}
            for ws in parameter.replace(" ", "").split(","):
                key, value = ws.split(":")
                ws_dict[key.lower() + end_string] = str(value)
        else:
            # handle val3 case
            ws_dict["dl" + end_string] = ws_dict["ul" + end_string] = parameter
    return ws_dict


def search_for_mandatory_keyword_arguments(settings, mandatory_keys):
    """
    Search for mandatory settings

    :type settings: dict
    :param settings:  dictionary containing all parameters

    :type mandatory_keys: tuple or list
    :param mandatory_keys:  list of mandatory values
    """
    for key in mandatory_keys:
        if key not in settings.keys():
            error_msg = "Setting '%s' not found" % key
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)


def start_iperf_server(settings):
    """
    Execute iperf server command on host

    :type settings: dictionary
    :param settings: Dictionary containing all settings to launch iperf as server
    """

    LOGGER_TEST_SCRIPT.info("Start iperf server")

    # Retrieve iperf settings
    iperf = settings.get('iperf', 'iperf')
    ftp_user = settings.get('ftp_user')
    server_ip_address = settings.get('server_ip_address')

    # Check for mandatory setting
    search_for_mandatory_keyword_arguments(settings, ['port_number'])

    # Build the iperf command
    largs = IperfOptions(settings).format_iperf_options("server").split(' ')

    if iperf.startswith("ssh ") and ftp_user is not None:
        cmd = iperf.split(' ')
        args = []
        rcmd = None
        for arg in cmd[1:]:
            if arg.find('@') != -1:
                pass
            elif arg.find('/') != -1 or arg.find('iperf') != -1:
                rcmd = arg
            elif rcmd is None:
                args.append(arg)
            else:
                largs.append(arg)
        largs = [cmd[0]] + args + [ftp_user + '@' + server_ip_address, rcmd + ' ' + ' '.join(largs)]
    else:
        largs = iperf.split(' ') + largs

    # Run IPERF Server locally
    LOGGER_TEST_SCRIPT.debug("Run command : " + ' '.join(largs))
    (process, queue) = run_local_command(largs)
    time.sleep(1)

    return process, queue


def stop_iperf_server(process, queue):
    """
    Launch iperf server on embedded side
    .. warning:: Using a compiled version (2.0.2) of iperf by intel on.
                !!!! The executable should be present in /system/bin
    :type process: process id of iperf command
    :type queue: queue of iperf command
    """

    LOGGER_TEST_SCRIPT.debug("Stop iperf server")
    # Terminate IPERF server
    serveroutput = ''
    while True:
        try:
            line = queue.get(timeout=.1)
        except Empty:
            break
        else:
            serveroutput += line

    LOGGER_TEST_SCRIPT.debug("iperf server output:\n" + serveroutput)
    process.terminate()


def run_local_iperf_client(settings):
    """
    Execute iperf client command on host

    :type settings: dictionary
    :param settings: Dictionary containing all settings to launch iperf as server

    :rtype: str
    :return: Command result
    """

    # Check for mandatory setting
    search_for_mandatory_keyword_arguments(
        settings,
        ['server_ip_address', 'port_number', 'duration'])

    # Retrieve iperf settings
    iperf = settings.get('iperf', 'iperf')
    protocol = settings.get('protocol', 'tcp')
    ftp_user = settings.get('ftp_user')
    server_ip_address = settings.get('server_ip_address')
    port_number = settings.get('port_number')
    duration = settings.get('duration', '10')
    direction = settings.get('direction', 'both')

    msg = "IPERF %s " % protocol.upper()
    if direction == 'both':
        msg += "UL & DL"
    elif direction == 'up':
        msg += "UL"
    else:
        msg += "DL"

    msg += " measurement on port %s for %s seconds" % (str(port_number), str(duration))
    LOGGER_TEST_SCRIPT.info(msg)

    # Build the iperf command
    largs = IperfOptions(settings).format_iperf_options("client").split(' ')

    if iperf.startswith("ssh ") and ftp_user is not None:
        cmd = iperf.split(' ')
        args = []
        rcmd = None
        for arg in cmd[1:]:
            if arg.find('@') != -1:
                pass
            elif arg.find('/') != -1 or arg.find('iperf') != -1:
                rcmd = arg
            elif rcmd is None:
                args.append(arg)
            else:
                largs.append(arg)
        largs = [cmd[0]] + args + [ftp_user + '@' + server_ip_address, rcmd + ' ' + ' '.join(largs)]
    else:
        largs = iperf.split(' ') + largs

    # Run IPERF Client locally
    LOGGER_TEST_SCRIPT.debug("Run command : " + ' '.join(largs))
    output_message = subprocess.Popen(largs, shell=False, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE).communicate()[0]
    print output_message
    LOGGER_TEST_SCRIPT.debug("iperf client output:\n" + output_message)
    return output_message


def get_random_iperf_port():
    """
    Compute a random port for iperf execution

    :rtype: int
    :return: radom port generated
    """
    # Reset random generator
    random.seed()
    # return a port between 3000 and 7000
    return int(random.randint(3000, 7000))


class IperfExecutionHandler:
    """
    Class handling common algorithm for all execution mode of iperf.
    """

    def __init__(self, settings, target, remote=None):
        """
        Constructor

        :type settings: dictionary
        :param settings:  iperf options dictionary with mandatory list:
        server_ip_address, port_number, duration

        :type target : Iperf
        :param target : instance of a Networking uecmd category on which iperf is executed
        :type remote : Iperf
        :param remote : instance on which iperf is executed

        """
        self.__settings = settings
        self.__logger = LOGGER_TEST_SCRIPT
        # options manager
        self.__option_mgr = IperfOptions(settings)
        # DUT Networking API object
        self.__target = target
        # other side object (Iperf capable)
        self.__remote = remote

    def iperf(self):
        """
        Measures throughputs using IPERF

        :rtype: measured throughput value
        """
        self._check_iperf_options()

        direction = self.__settings['direction']
        self.__logger.info("Perform IPERF (%s - %s) for %s s." % (direction,
                                                                  self.__settings['protocol'],
                                                                  str(self.__settings['duration'])))

        # UL Iperf: Server start on host/second DUT/equipment and client start on DUT
        if 'up' in direction:
            return self._iperf_ul()
        # UL + DL concurrency iperf
        elif 'both' in direction:
            return self._iperf_uldl()
        # DL Iperf: Server start on DUT and client start on host/second DUT/equipment
        else:
            return self._iperf_dl()

    def iperf_async(self):
        """
        Start IPERF in asynchronus mode (not blocking)
        """
        self._check_iperf_options()

        # Set interval Iperf parameter to have current throughput every seconds
        self.__settings['interval'] = '1'

        direction = self.__settings['direction']
        self.__logger.info("Perform Asynchronous IPERF (%s - %s) for %s s." % (direction,
                                                                               self.__settings['protocol'],
                                                                               str(self.__settings['duration'])))

        # UL Iperf: Server start on host/second DUT/equipment and client start on DUT
        if 'up' in direction:
            self.__logger.error("IPERF async only supported in DL")
        # UL + DL concurrency iperf
        elif 'both' in direction:
            self.__logger.error("IPERF async only supported in DL")
        # DL Iperf: Server start on DUT and client start on host/second DUT/equipment
        else:
            # Save start time
            self._iperf_async_start_time = time.time()
            self.__logger.info("Start DL IPERF server on DUT")
            self.__target.start_iperf_server(self.__settings)
            self.__logger.info("Iperf server started wait 5 seconds to ensure server is started")
            time.sleep(5)
            self.__logger.info("Start DL IPERF client on equipment")
            self._dl_client_thread = Execution_thread(self.__remote.start_iperf_client, (self.__settings,))
            self._dl_client_thread.start()

    def stop_iperf_async(self):
        """
        Wait end of Iperf Async
        """
        self.__remote.stop_iperf_client()
        self.__target.stop_iperf_server()

    def perform_continous_measure(self, duration):
        """
        Perform continous measure on Iperf Async

        :type duration: float
        :param duration: Duration of the measure
        """
        self.__logger.info("Perfom Iperf Async continuous measure during %ds" % duration)
        start_measure_time = time.time() - self._iperf_async_start_time
        time.sleep(duration)
        # Retrieve all throughput to calculate the mean throughput
        sum_measure = ThroughputMeasure(0.0, ThroughputMeasure.KBPS_UNIT)
        nb_measure = 0
        if hasattr(self.__target, '__process') and hasattr(self.__target, '__q'):
            # Read all the current lines available
            while True:
                try:
                    line = str(getattr(self.__target, '__q').get(timeout=.1))
                except Empty:
                    break
                else:
                    line = line.strip()
                    # If line contains a measure
                    if re.match("\[.*sec.*bits/sec", line) is not None:
                        begin_time, end_time, throughput = self._parse_iperf_line(line)
                        # If the measure is between start and the stop interval
                        if (start_measure_time <= begin_time <= start_measure_time + duration and
                           start_measure_time <= end_time <= start_measure_time + duration):
                            sum_measure += throughput
                            nb_measure += 1
                        self.__logger.debug(line)
        else:
            self.__logger.error("Iperf server was not started.")

        # Calculate mean throughput
        throughput_target = DuplexThroughputMeasure()
        if nb_measure != 0:
            mean_measure = sum_measure / nb_measure
            mean_measure.convert_to_best_unit()
            throughput_target.dl_throughput = mean_measure
        else:
            self.__logger.error("No measure retrieved during Iperf Async.")

        return throughput_target

    def _parse_iperf_line(self, line):
        """
        Parse Iperf measure line
        :type line: str
        :param line: Line to parse
        :rtype: float, float, ThroughputMeasure
        :return: start time, end time, throughput measure
        """
        # [  3]  0.0- 1.5 sec   386 MBytes  2.16 Gbits/sec
        result = re.match("\[.*\]\s*([\d\.]*)-\s*([\d\.]*)\s*sec.*Bytes\s*([\d\.]*)\s*(\S*)", line)
        # more talkative error
        error_msg = None
        if result is None:
            error_msg = "Parsing iperf measurement line returned zero matching elements!"
        elif len(result.groups()) < 4 or None in result.groups():
            error_msg = "Parsing iperf measurement did not return enough elements to compute throughput : %s" % str(result.groups())
        if error_msg is not None:
            LOGGER_TEST_SCRIPT.error(error_msg)
            raise AcsConfigException(AcsConfigException.ALGORITHM_FAILURE, error_msg)

        try:
            start_time, end_time, throughput_value, throughput_unit = result.groups()
            throughput = ThroughputMeasure(float(throughput_value), ThroughputMeasure.parse_unit(throughput_unit))
            return float(start_time), float(end_time), throughput
        except Exception as e:
            msg = "Exception happen when parsing Iperf Async line return: %s" % str(e)
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.ALGORITHM_FAILURE, msg)

    def _check_iperf_options(self):
        """
        Check Iperf options are valid
        """
        available_remotes = ['computer', 'networking_api2', 'equipment_api']
        # for retro compatibility, search the good equipment
        if self.__remote is None :
            for remote in available_remotes:
                self.__remote = self.__settings.get(remote, None)
                if self.__remote is not None:
                    break

        # check that both target and remote are Iperf capable
        error_msg = []
        if not issubclass(self.__target.__class__, Iperf):
            error_msg.append("DUT is not Iperf capable.")

        # if remote is None it is better to return a right information
        if self.__remote is None:
            error_msg.append("There is no Iperf remote declared.")

        elif not issubclass(self.__remote.__class__, Iperf):
            error_msg.append("Remote is not Iperf capable.")

        if len(error_msg) > 0:
            msg = "\n".join(error_msg)
            LOGGER_TEST_SCRIPT.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # check mandatory options are present and well formed
        self.__option_mgr.ckeck_for_mandatory_options()

        self.__settings["multi"] = "parallel_thread" in self.__settings.keys() \
            and self.__settings.get('parallel_thread') > 1

    def _compute_throughput(self, output_client, output_server):
        # , direction, multi, protocol
        """
        Compute iperf throughput from iperf server and client output

        :type output_client: str
        :param output_client: iperf client output
        :type output_server: str
        :param output_server: iperf server output
        :rtype: measured throughput value
        """
        # import fnmatch
        # import re
        # throughput = MeasureThput()
        # for side, output in zip(["client", "server"], [output_client, output_server]):
        #     output = output.split("\n")
        #     if self.__settings["multi"]:
        #         pattern = "*SUM*bits/sec*"
        #     else:
        #         pattern = "*bits/sec*"
        #     output = fnmatch.filter(output, pattern)
        #     # there should only be 1 match
        #     for o in output:
        #         result = re.search("([0-9.]+) (.bits/sec)", o)
        #         print result.group(0)
        # return

        try:
            # First retrieve throughput from client output
            throughput = self._compute_iperf_throughput(output_client)
        except Exception as e:
            self.__logger.error("Error happen when retrieving throughput from client output: %s" % str(e))
            # If failed retrieve throughput from server output
            self.__logger.debug("Can't retrieve throughput from client output, trying to get it from server output")
            self.__logger.debug("Client output: \n %s", output_client)
            try:
                throughput = self._compute_iperf_throughput_from_server_output(output_server)
                # , direction
            except Exception as e:
                self.__logger.error("Error happen when retrieving throughput from server output: %s" % str(e))
                error = "Can't retrieve %s throughput from server and client output" % self.__settings["direction"]
                self.__logger.error(error)
                self.__logger.debug("Server output: \n %s", output_server)
                # Return Throughput set to 0.0 if no report available
                throughput = DuplexThroughputMeasure()
        return throughput

    def _iperf_ul(self):
        # , settings, computer, networking_api2, equipment_api, multi, protocol
        """
        Launch ul iperf

        :rtype: measured throughput value
        """
        # Transform ul_server/client_window_size in server_window_size if needed
        if "ul_server_window_size" in self.__settings:
            self.__settings["server_window_size"] = self.__settings["ul_server_window_size"]
        if "ul_client_window_size" in self.__settings:
            self.__settings["client_window_size"] = self.__settings["ul_client_window_size"]
        if "ul_parallel_thread" in self.__settings:
            self.__settings["parallel_thread"] = int(self.__settings["ul_parallel_thread"])
        self.__settings["multi"] = "parallel_thread" in self.__settings \
            and self.__settings.get('parallel_thread') > 1

        # Start IPERF Server on host/second DUT/equipment
        self.__remote.start_iperf_server(self.__settings)
        self.__logger.info("Iperf server started wait 5 seconds to ensure server is started")
        time.sleep(5)
        output_client = ""
        output_server = ""
        self.__logger.debug("Iperf settings used: %s" % str(self.__settings))
        try:
            # Run IPERF client through adb on DUT
            output_client = self.__target.start_iperf_client(self.__settings)
        finally:
            self.__logger.info("Iperf client ended, wait 15 seconds to allow final report on iperf server")
            time.sleep(15)
            # Stop IPERF server
            output_server = self.__remote.stop_iperf_server()
            self.__logger.info("Iperf server ended")

            throughput = self._compute_throughput(output_client, output_server)
            self.__logger.info("Iperf measured throughput : %s" % str(throughput.ul_throughput))
        return throughput

    def _iperf_uldl(self):
        # , settings, computer, networking_api2, equipment_api, multi, protocol
        """
        Launch ul and dl iperf. It will start an UL IPERF server and a DL IPERF client on remote
        equipment and a DL IPERF server and a UL IPERF client on DUT in order to have an UL and a DL measurement

        :rtype: measured throughput value
        """
        output_dl_client = ""
        output_ul_client = ""
        output_dl_server = ""
        output_ul_server = ""
        # Create an iperf settings dictionnary for DL IPERF client/server
        settings_dl = dict(self.__settings)
        settings_ul = dict(self.__settings)
        if self.__settings["protocol"] == "udp" and "dl_bandwidth" in self.__settings.keys():
            settings_dl["bandwidth"] = self.__settings["dl_bandwidth"]
        settings_ul["direction"] = "up"
        settings_dl["direction"] = "down"

        # Transform ul_server/client_window_size in server_window_size if needed
        if "ul_server_window_size" in settings_ul:
            settings_ul["server_window_size"] = settings_ul["ul_server_window_size"]
        if "ul_client_window_size" in settings_ul:
            settings_ul["client_window_size"] = settings_ul["ul_client_window_size"]
        # Transform dl_server/client_window_size in server_window_size if needed
        if "dl_server_window_size" in settings_dl:
            settings_dl["server_window_size"] = settings_dl["dl_server_window_size"]
        if "dl_client_window_size" in settings_dl:
            settings_dl["client_window_size"] = settings_dl["dl_client_window_size"]

        if "dl_parallel_thread" in settings_dl:
            settings_dl["parallel_thread"] = int(settings_dl["dl_parallel_thread"])
        if "ul_parallel_thread" in settings_ul:
            settings_ul["parallel_thread"] = int(settings_dl["ul_parallel_thread"])

        settings_ul["multi"] = "parallel_thread" in settings_ul and settings_ul.get('parallel_thread') > 1
        settings_dl["multi"] = "parallel_thread" in settings_dl and settings_dl.get('parallel_thread') > 1
        # As DL server and client are swapped compared to UL ones
        # Swap bind_host and server_ip_address
        if "bind_host" in settings_ul:
            settings_dl["server_ip_address"] = settings_ul["bind_host"]
        else:
            settings_dl["server_ip_address"] = settings_ul["dut_ip_address"]
        settings_dl["port_number"] = self.__settings["port_number"] + 1
        if self.__remote.__class__.__name__ == "RemoteComputer":
            # No need to bind address on remote computer side
            if "bind_host" in settings_dl.keys():
                del settings_dl["bind_host"]
        self.__logger.debug("iperf settings %s" % str(self.__settings))
        # Start iperf clients on DUT (UL) and remote computer (DL)
        try:
            self.__logger.info("Start DL IPERF server on DUT")
            self.__target.start_iperf_server(settings_dl)
            self.__logger.info("Start UL IPERF server on equipment")
            self.__remote.start_iperf_server(settings_ul)
            self.__logger.info("IPERF servers started, wait 5 seconds to ensure servers are started")
            time.sleep(5)
            self.__logger.info("Start DL IPERF client on equipment")
            dl_client_thread = Execution_thread(self.__remote.start_iperf_client, (settings_dl,))
            dl_client_thread.start()

            self.__logger.info("Start UL IPERF client on DUT")
            output_ul_client = self.__target.start_iperf_client(settings_ul)
            dl_client_thread.join()
            self.__logger.info("DL IPERF client stopped")
            output_dl_client = dl_client_thread.exec_output

        finally:
            self.__logger.info("Iperf UL and DL client ended, wait 15 seconds to allow final report on iperf server")
            time.sleep(15)
            # Stop IPERF UL Server on host/second DUT/equipment
            output_ul_server = self.__remote.stop_iperf_server()
            self.__logger.info("UL IPERF server stopped")
            # Stop DL IPERF server on DUT
            output_dl_server = self.__target.stop_iperf_server()
            self.__logger.info("DL IPERF server stopped")

            self.__settings["direction"] = "up"
            if "multi" in settings_ul:
                self.__settings["multi"] = settings_ul["multi"]
            elif "multi" in self.__settings:
                del self.__settings["multi"]
            throughput_ul = self._compute_throughput(output_ul_client, output_ul_server)
            # , "up", multi, protocol
            self.__settings["direction"] = "down"
            if "multi" in settings_dl:
                self.__settings["multi"] = settings_dl["multi"]
            elif "multi" in self.__settings:
                del self.__settings["multi"]
            throughput = self._compute_throughput(output_dl_client, output_dl_server)
            self.__settings["direction"] = "both"
            # , "down", multi, protocol
            throughput.ul_throughput = throughput_ul.ul_throughput
            self.__logger.info("Iperf measured UL throughput: %s" % str(throughput.ul_throughput))
            self.__logger.info("Iperf measured DL throughput: %s" % str(throughput.dl_throughput))

        return throughput

    def _iperf_dl(self):
        """
        Launch dl iperf
        :rtype: measured throughput value
        """
        # Transform dl_server/client_window_size in server_window_size if needed
        if "dl_server_window_size" in self.__settings:
            self.__settings["server_window_size"] = self.__settings["dl_server_window_size"]
        if "dl_client_window_size" in self.__settings:
            self.__settings["client_window_size"] = self.__settings["dl_client_window_size"]
        if "dl_parallel_thread" in self.__settings:
            self.__settings["parallel_thread"] = int(self.__settings["dl_parallel_thread"])
        self.__settings["multi"] = "parallel_thread" in self.__settings \
            and self.__settings.get('parallel_thread') > 1
        # Start IPERF server on DUT
        self.__target.start_iperf_server(self.__settings)
        self.__logger.info("Iperf server started wait 5 seconds to ensure server is started")
        time.sleep(5)
        output_client = ""
        server_output = ""
        self.__logger.debug("iperf settings %s" % str(self.__settings))
        try:
            # Run IPERF client on host/second DUT/equipment
            output_client = self.__remote.start_iperf_client(self.__settings)
        finally:
            self.__logger.info("Iperf client ended, wait 15 seconds to allow final report on iperf server")
            time.sleep(15)
            # Stop IPERF server on DUT
            server_output = self.__target.stop_iperf_server()
            self.__logger.info("Iperf server ended")

            throughput = self._compute_throughput(output_client, server_output)
            # , direction, multi, protocol
            self.__logger.info("Iperf measured throughput: %s" % str(throughput.dl_throughput))

        return throughput

    def _compute_iperf_throughput(self, client_output):
        """, multi, direction
        Compute throughput from iperf client output line

        :type client_output: str
        :param client_output:  iperf client output line
        it should contain a line similar to this:
            [  3] Server Report:
            [  3]  0.0-29.9 sec    174 MBytes  48.9 Mbits/sec  0.342 ms 2615/126957 (2.1%)
        or
            [  3] local 172.22.1.100 port 5001 connected with 172.22.1.201 port 44971
            [  3]  0.0-31.1 sec    369 MBytes  99.5 Mbits/sec  0.129 ms 14139/277144 (5.1%)

        :rtype: measured throughput value
        """
        output_lower = client_output.lower()
        if self.__settings["protocol"] == "udp" and not self.__settings["multi"]:
            output_split = client_output.split("Server Report:")
            if len(output_split) > 1:
                if output_split[1].find("local") != -1:
                    output_split = [output_split[0]] + output_split[1].split("local")
            else:
                self.__logger.warning("No Server Report available on client output!")
        elif self.__settings["multi"]:
            output_split = client_output.split("[SUM]")
        else:
            output_split = client_output.split("connected with")

        error_msg = None
        if "error" in output_lower:
            error_msg = "An Error occurs in IPERF command: " + output_lower
        elif len(output_split) > 1:
            if len(output_split) == 2 and self.__settings["direction"] != "up" and self.__settings["direction"] != "down":
                self.__logger.warning("IPERF data results are partial")
            # There are both UL & DL throughput available
            throughput = self._iperf_get_throughput(output_split)
        elif "connection refused" in output_lower:
            error_msg = "Unable to connect to IPERF server. Please check IP server, IPERF port and protocol"
        else:
            error_msg = "Unable to parse IPERF result: " + output_lower

        if error_msg is not None:
            LOGGER_TEST_SCRIPT.error(error_msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_msg)

        return throughput

    def _compute_iperf_throughput_from_server_output(self, server_output):
        # , direction
        """
        Compute throughput from iperf server output line

        :type server_output: str
        :param server_output:  iperf server output line
        :type direction: str
        :param direction:  iperf direction "up"/"down"
        it must contain a line similar to this:
        [  3] local 172.22.1.100 port 5001 connected with 172.22.1.201 port 44971
        [  3]  0.0-31.1 sec    369 MBytes  99.5 Mbits/sec  0.129 ms 14139/277144 (5.1%)
        :rtype: measured throughput value
        """

        output_lower = server_output.lower()

        output_split = server_output.split("connected with")

        error_msg = None
        if "error" in output_lower:
            error_msg = "An Error occurred in IPERF command: " + output_lower
        elif len(output_split) > 1:
            if len(output_split) == 2 and self.__settings["direction"] == "both":
                self.__logger.warning("IPERF data results are partial")
            # There are both UL & DL throughput available
            throughput = self._iperf_get_throughput(output_split)
        elif "connection refused" in output_lower:
            error_msg = "Unable to connect to IPERF server. Please check IP server, IPERF port and protocol"
        else:
            error_msg = "Unable to parse IPERF result: " + output_lower

        if error_msg is not None:
            LOGGER_TEST_SCRIPT.error(error_msg)
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, error_msg)

        return throughput

    def _iperf_get_throughput(self, output_split):
        """
        Extract the data rates from the output strings contained
        in the output_split str array

        :type output_split: list
        :param output_split: str array that contains iperf output separated
                            '[SUM]' by token
        :type direction: str
        :param direction: Direction in which the iperf command has been run
                            ('up' 'down' or 'both')
        :type protocol: str
        :param protocol: protocol in which the iperf command has been run
                            ('tcp'|'udp')
        """
        direction = self.__settings["direction"]
        protocol = self.__settings["protocol"]
        throughput = DuplexThroughputMeasure()

        # Sometime Iperf displays DOWN SUM before UP SUM (thread scheduling ?)
        # With a modified Iperf 2.0.2, a marker is added for each [SUM] report
        # in order to avoid inversion in parsing
        id_up = 1
        id_down = 2
        if direction == 'up':
            id_up = 1
        elif direction == 'down':
            id_down = 1
        elif output_split[0].find("[DOWN]") > 0:
            id_down = 1
            id_up = 2
        elif direction == "both" and protocol == "tcp":
            # With TCP protocol and bidirectionnal transfer there is two lines containing
            # "connected with  " str before UL throughput report (see output line example below)
            id_up = 2

        # Parse output to get measured UL throughput & units
        if direction == "up":
            if id_up < len(output_split):
                output_line_ul = output_split[id_up].strip()
                output_string_ul = output_line_ul.split("Bytes")[1].strip()
                throughput.ul_throughput.set(float(output_string_ul.split()[0]),
                                             ThroughputMeasure.parse_unit(output_string_ul.split()[1]))
            else:
                throughput.ul_throughput.set(-1, ThroughputMeasure.BPS_UNIT)
        # Parse output to get measured DL throughput & units
        elif direction == "down":
            if id_down < len(output_split):
                output_line_dl = output_split[id_down].strip()
                output_string_dl = output_line_dl.split("Bytes")[1].strip()
                throughput.dl_throughput.set(float(output_string_dl.split()[0]),
                                             ThroughputMeasure.parse_unit(output_string_dl.split()[1]))
            else:
                throughput.dl_throughput.set(-1, ThroughputMeasure.BPS_UNIT)
        elif direction == "both":
            # Output_line example for UDP protocol with iperf dual_test option
            # [  4] Server Report:
            # [  4]  0.0-60.1 sec    355 MBytes  49.6 Mbits/sec  0.324 ms 2000/255319 (0.78%) UL report
            # [  3]  0.0-60.1 sec    356 MBytes  49.8 Mbits/sec  0.341 ms 1037/255319 (0.41%) DL report
            #
            # Output_line example for TCP protocol with iperf dual_test option
            # [  4] local 10.102.160.50 port 36225 connected with 10.102.160.45 port 5009
            # [  5] local 10.102.160.50 port 5009 connected with 10.102.160.45 port 34935
            # [ ID] Interval       Transfer     Bandwidth
            # [  4]  0.0-61.6 sec    760 KBytes    101 Kbits/sec UL report
            # [  5]  0.0-62.6 sec    832 KBytes    109 Kbits/sec DL report

            if id_up < len(output_split):
                output_line = output_split[id_up].strip()
                bytes_line = output_line.split("Bytes")
                output_string_ul = bytes_line[1].strip()
                throughput.ul_throughput.set(float(output_string_ul.split()[0]),
                                             ThroughputMeasure.parse_unit(output_string_ul.split()[1]))
                if len(bytes_line) > 2:
                    output_string_dl = bytes_line[2].strip()
                    throughput.dl_throughput.set(float(output_string_dl.split()[0]),
                                                 ThroughputMeasure.parse_unit(output_string_dl.split()[1]))
                else:
                    throughput.dl_throughput.set(-1, ThroughputMeasure.BPS_UNIT)

            else:
                throughput.ul_throughput.set(-1, ThroughputMeasure.BPS_UNIT)
        return throughput

    def _stop_iperf_server(self, process, queue):
        """
        Launch iperf server on embedded side
        .. warning:: Using a compiled version (2.0.2) of iperf by intel on.
                    !!!! The executable should be present in /system/bin
        :type process: process id of iperf command
        :type queue: queue of iperf command
        :rtype: iperf server output line
        """

        self.__logger.info("Stop IPERF server on DUT:")
        # Terminate IPERF server
        process.terminate()
        serveroutput = ''
        while True:
            try:
                line = queue.get(timeout=.1)
            except Empty:
                break
            else:
                serveroutput += str(line)
        self.__logger.debug("iperf server output:\n %s" % serveroutput)

        return serveroutput

if __name__ == "__main__":
    iperf_settings = \
        {'iperf': 'iperf',
         'protocol': 'tcp',
         'direction': 'both',
         'server_ip_address': 'localhost',
         'duration': 10,
         'port_number': 1234,
         'bandwidth': '54M',
         'no_delay': '',
         'tradeoff': ''}

    (p, q1) = start_iperf_server(iperf_settings)

    run_local_iperf_client(iperf_settings)

    stop_iperf_server(p, q1)
