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
:summary: Implements methods controlling an external audio framework
:since: 09/06/2014
:author: fbelvezx
"""

import socket
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global


class AudioFramework(object):
    """
    Class to manage communication with an external audio framework

    The following measurements are possible:
    - Audio routing
    - Audio bandwidth

    The audio framework includes individual modules:
    - Acquisition (ACQ)
    - Record (REC)
    - Play of the audio samples being acquired (PLAY)
    - Compute the energy of a degraded audio file, in time domain (POW)
    - Compute the energy of a degraded audio file, in spectral domain (SPEC_POW)
    - Playback of a wave file (WAV_PLAY)

    /!\ WARNING /!\
    connect_fwk must be called before any other method of this class, as it is responsible for creating
    the python socket, and connecting to the audio framework
    """

    module_id = {}
    stream_id = {}

    # The connection with the audio framework is parametrized using a server IP address and a port number.
    # Hence, the server_address attribute should be in the form of the tuple (IP_ADDRESS, port_number),
    # and should be obtained from the benchconfig xml file, set from the main python script
    server_address = None
    sock = None

    def send_command_fwk(self, cmd):
        """
        Sends a command to the audio framework

        :param cmd: command to be sent to the audio framework
        :type cmd: str

        :return data: Message returned by the audio framework
        :rtype: str
        """
        LOGGER_TEST_SCRIPT.debug("Send command: %s" % cmd)
        try:
            self.sock.send(cmd)

            data = self.sock.recv(250)
        except:
            err_msg = "Communication failed with the Audio Framework. Please make sure that the connection has been " \
                      "established (using connect_fwk)"
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)

        return data

    def connect_fwk(self):
        """
        Connects to the the audio framework using a socket
        """
        # Create socket object
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.server_address)
        except socket.error:
            LOGGER_TEST_SCRIPT.error("The socket seems to have been closed by the server.")

    def release_fwk(self):
        """
        Releases the connection with the audio framework
        """
        try:
            self.sock.close()
            self.sock = None
        except (socket.error, AttributeError):
            LOGGER_TEST_SCRIPT.error("Could not close the socket")

    def start(self, module, stream_device, opts=None):
        """
        Starts a module of the audio framework

        :param module: Module of the audio framework to start
        :type module: str
        :param stream_device: ID of the device corresponding to the current stream_id (e.g. "DUT", "REF1", ...)
        :type stream_device: str
        :param opts: Additional option(s)
        :type opts: str
        """
        cmd = self.stream_id[stream_device] + module + ':start'
        if opts:
            cmd += opts
        LOGGER_TEST_SCRIPT.debug("Start %s module" % module)

        self.send_command_fwk(cmd)

    def stop(self, module, stream_device, opts=None):
        """
        Stops a module of the audio framework

        :param module: Module of the audio framework to stop
        :type module: str
        :param stream_device: ID of the device corresponding to the current stream_id (e.g. "DUT", "REF1", ...)
        :type stream_device: str
        :param opts: Additional option(s)
        :type opts: str
        """
        cmd = self.stream_id[stream_device] + module + ':stop'
        if opts:
            cmd += opts
        LOGGER_TEST_SCRIPT.debug("Stopping %s module" % module)
        self.send_command_fwk(cmd)

    def get_status(self, module, stream_device, opts=None):
        """
        Gets the status of a module of audio framework

        :param module: Module of the audio framework to get status from
        :type module: str
        :param stream_device: ID of the device corresponding to the current stream_id (e.g. "DUT", "REF1", ...)
        :type stream_device: str
        :param opts: Additional option(s)
        :type opts: str

        :return: status of the input module for device stream_device
        :rtype: str
        """
        if module in "FWK":
            cmd = '@@ ' + module + ':status'
        else:
            cmd = self.stream_id[stream_device] + module + ':status'
        if opts:
            cmd += opts
        LOGGER_TEST_SCRIPT.debug("Get status of %s module" % module)
        return self.send_command_fwk(cmd)

    def get_verdict(self, module, stream_device, opts=None):
        """
        Gets the verdict associated with a check/verification module of the audio framework

        :param module: Module of the audio framework to get status from
        :type module: str
        :param stream_device: ID of the device corresponding to the current stream_id (e.g. "DUT", "REF1", ...)
        :type stream_device: str
        :param opts: Additional option(s)
        :type opts: str

        :return: verdict issued by the audio framework for the input check module
        :rtype: str
        """
        # So far, only ROUTE_CHECK module has the get_verdict function available
        if module in "ROUTE_CHECK":
            cmd = self.stream_id[stream_device] + module + ':get_verdict'
            if opts:
                cmd += opts
            LOGGER_TEST_SCRIPT.debug("Stopping %s module" % module)
            return self.send_command_fwk(cmd)
        else:
            err_msg = "Module %s does not support get_verdict !" % module
            LOGGER_TEST_SCRIPT.error("Module %s does not support get_verdict !" % module)
            return err_msg

    def initialize_fwk(self):
        """
        Initializes the audio framework:
        - Resets the modules of the audio framework
        - Reads AUDIO_BENCH_SETUP xml
        """
        LOGGER_TEST_SCRIPT.debug("Setting all modules of the audio framework to idle")

        # Get all the stream_id from AUDIO_BENCH_SETUP xml
        for i in self.send_command_fwk('@@ FWK:get_stream_id').split(';'):

            if "Dut" in i:
                stream_device = "DUT"
                self.stream_id[stream_device] = '@' + i + '@'
            elif "RefPhone" in i:
                if i[-1].isdigit():
                    stream_device = "REF" + i[-1]
                    self.stream_id[stream_device] = '@' + i + '@'
                else:
                    stream_device = "REF"
                    self.stream_id[stream_device] = '@' + i + '@'
            else:
                err_msg = "No device declared in AUDIO_BENCH_SETUP xml"
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)

            # For each stream_id, get the status of all its modules,
            # and stop them if they are running
            for j in self.send_command_fwk('@@ FWK:get_module_id stream_id=%s' % i).split(';'):
                self.module_id[j] = self.get_status(j, stream_device)

                if self.module_id[j] is "Issue with the socket":
                    LOGGER_TEST_SCRIPT.error()

                if self.module_id[j].split(':')[-1] not in "IDLE":
                    LOGGER_TEST_SCRIPT.debug("Stopping %s module... " % self.module_id[j])
                    self.stop(self.module_id[j], stream_device)
                else:
                    LOGGER_TEST_SCRIPT.debug("%s module status is IDLE" % self.module_id[j])

    def start_audio_routing(self,
                            stream_device,
                            ref_file,
                            timeout,
                            wait_for_timeout,
                            start_playback=True,
                            stop_playback=True):
        """
        Starts a routing check on the audio framework

        :param stream_device: ID of the device corresponding to the current stream_id (e.g. DUT, RefPhone1, ...)
        :type stream_device: str
        :param timeout: The framework returns the verdict at least after timeout seconds.
        :type timeout: int
        :param wait_for_timeout: Controls the way the verdict is computed
        :type wait_for_timeout: bool

        :rtype: str
        :return: detailed verdict
        Note that:

        - Regarding timeout:
        By default (timeout=0), the ROUTE_CHECK module returns an answer with the last computed verdict.
        If the timeout value is upper than 0, the verdict is returned as soon as the Fwk encountered a PASS
        verdict. The maximum processing duration is set by this timeout value.

        - Regarding wait_for_timeout:
            - If wait_for_timeout == 0, the framework returns a verdict as soon as possible. (Default value)
            - If wait_for_timeout == 1, the framework waits until the timeout value before returning the verdict.

        """
        opt = ""

        if timeout >= 0:
            opt = " timeout=%d" % timeout
            LOGGER_TEST_SCRIPT.info("Start routing check on audio framework, timeout = %ds" % timeout)
            if wait_for_timeout:
                opt += " wait_for_timeout=1"
            else:
                opt += " wait_for_timeout=0"
        else:
            LOGGER_TEST_SCRIPT.info("Start routing check on audio framework, no timeout")

        if start_playback:
            self.start("WAV_PLAY", stream_device, " filename=%s" % ref_file)

        self.start("ACQ", stream_device)
        self.start("POW", stream_device)
        self.start("ROUTE_CHECK", stream_device)

        ret = self.get_verdict("ROUTE_CHECK", stream_device, opt)

        self.stop("POW", stream_device)
        self.stop("ACQ", stream_device)

        if stop_playback:
            self.stop("WAV_PLAY", stream_device)

        if "PASS" in ret:
            verdict = Global.SUCCESS
            err_msg = "Audio correctly routed (Power-based criteria): %s" % ret.split('_')[-1]
            LOGGER_TEST_SCRIPT.info(err_msg)
        elif "OK" in ret:
            return "Audio routing check started"
        else:
            verdict = Global.FAILURE
            err_msg = "No audio routed (Power-based criteria): %s" % ret.split('_')[-1]
            LOGGER_TEST_SCRIPT.error(err_msg)

        return [verdict, err_msg]

