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
:summary: implementation of RS UPV audio analyzer
:since:12/04/2011
:author: ymorel
"""

import time
from acs_test_scripts.Equipment.AudioAnalyzer.Interface.IAudioAnalyzer import IAudioAnalyzer
from acs_test_scripts.TestStep.Utilities.Visa import VisaEquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException


class RsUpv(IAudioAnalyzer, VisaEquipmentBase):
    """
    Implementation of RS UPV audio analyzer
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: BenchConfigParameters
        :param bench_params: the dictionary containing equipment bench parameters
        """

        IAudioAnalyzer.__init__(self)
        VisaEquipmentBase.__init__(self, name, model, eqt_params, bench_params)
        self.__bench_params = bench_params

        # Initialize attributes
        self.__name = name
        self.__model = model
        self.__eqt_params = eqt_params

        # Meaning of bits in STATus:OPERation register
        self._op_status_sweep = {
            # Sweep status bits (bits 3, 9, and 8 of the operation register)
            "000": "sweep_off",
            "001": "not_used",
            "010": "sweep_stopped",
            "011": "sweep_waiting",
            "100": "sweep_run_manual",
            "101": "sweep_run_single",
            "110": "sweep_run_cont",
            "111": "not_used"}

        self._op_status_meas = {
            # Measurement status bits (bits 4 and 10 of the operation register)
            "00": "measurement_terminated",
            "01": "measurement_stopped",
            "10": "measurement_single",
            "11": "measurement_cont"}

    def __del__(self):
        """
        Destructor
        """
        self.release()

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        pass

    def release(self):
        """
        Release the equipment and all associated resources
        """
        pass

    def send_command(self, command, bts=None):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :type bts: str
        :param bts: the BTS to apply the command
        """
        # Send command to the equipment
        self._visa.write("*CLS")
        self._visa.write(command)
        self.check_error()

    def query_command(self, command, log=True):
        """
        Query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :return: The response from agilent for the command
        """

        # Send command *CLS to the equipment to clear the Error Queue
        self._visa.write("*CLS")

        # Query the command
        response = self._visa.query(command, log)

        self.check_error()

        return response

    def check_error(self):
        """
        Check if an error occurred on 8960
        """
        # Retrieve error code from 8960
        err = self._visa.query("SYSTEM:ERROR?")
        # Clear error message queue
        self._visa.write("*CLS")
        self._visa.write("DISPlay:WINDow:ERRor:CLEar")

        if "No error" in err:
            return
        else:
            # Error returned by 8960 is similar to -300, Device specific error
            # So split error code from error message
            err_list = err.split(",")
            err_code = int(err_list[0])
            err_msg = err_list[1]
            if err_code < 0:
                self._logger.error(err_msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
            elif err_code > 0:
                self._logger.warning(err_msg)

    def copy_to_upv(self, source, dest):
        """
        Transmits stored instrument settings, trace files, waveform files,
        filter coefficient files to the UPV
        :type source: str
        :param source: the source file to copy
        :type dest: str
        :param dest: the destination file for the copy
        """
        with open(source, 'rb') as input_file_content:
            source_bin = input_file_content.read()

            # Get the binary data corresponding to the source file to copy,
            # as well as the length of the file in terms of bytes
            self.send_command('MMEM:DATA \'%s\',#%s%s%s' % (dest,
                                                            len(str(len(source_bin))),
                                                            len(source_bin),
                                                            source_bin))

    def copy_from_upv(self, source, dest):
        """
        Reads stored instrument settings, trace files, waveform files,
        filter coefficient files from the UPV
        :type source: str
        :param source: the source file to copy
        :type dest: str
        :param dest: the destination file for the copy
        """
        tmp = self.query_command('MMEM:DATA? \'%s\'' % source, log=False)

        if 'wav' in source:
            tmp = 'RIFF' + tmp.split('RIFF')[-1]

        with open(dest, 'wb') as file_handle:
            file_handle.write(tmp)

    def load_configuration(self,
                           test_type,
                           ns_model=None,
                           aa_ref_file_path=None,
                           codec_type=None,
                           option=[]):
        """
            Launch a configuration for audio codec testing in RS UPV firmware

            :type test_type: str
            :param test_type: type of measurement to perform
            :type ns_model: str
            :param ns_model: str identifying the network simulator
            :type aa_ref_file_path: str
            :param aa_ref_file_path: the reference audio file
            :type codec_type: str
            :param codec_type: the type of audio codec (NB, WB, SWB) to be tested
            :type option: str
            :param option: additional option
            """
        # Select the appropriate setup depending on the type of audio test to perform
        if "AQ" in test_type:
            if "PESQ" in test_type or "COD_SWI" in test_type:
                upv_setup = "PESQ" + "_" + codec_type + "_" + ns_model.split('_')[1]
            else:
                self._logger.error("Unknown audio quality test")
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Unknown audio quality test")
        elif "CSV" in test_type:
            if "DTMF" in test_type:
                if option:
                    upv_setup = "FFT_DTMF_CMU200_%s_%s" % (option[0], option[1])
                else:
                    self._logger.error("No option specified for DTMF testing")
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                             "No option specified for DTMF testing")
            elif "MO" in test_type or "MT" in test_type:
                if option:
                    upv_setup = "RMS_routing_CMU200_%s" % option
                else:
                    self._logger.error("Unknown CSV call test")
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                             "Unknown CSV call test")
        elif "CAPTURE" in test_type:
            if "RECORDTONES" in test_type:
                upv_setup = "FFT_rectones_CMU200"
            else:
                self._logger.error("Unknown call capture test")
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Unknown call capture test")
        elif "VOIP" in test_type:
            if "RI" in test_type:
                upv_setup = "POLQA_NB_CMW500_%s" % option
            elif option:
                upv_setup = "RMS_routing_VOIP_%s" % option
            else:
                self._logger.error("Need to specify a direction for audio stream")
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Need to specify a direction for audio stream")
        elif "FIT" in test_type:
            if "FLIGHTMODE" in test_type:
                if option:
                    upv_setup = "RMS_routing_CMU200_%s" % option
                else:
                    self._logger.error("Need to specify a direction for audio stream")
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                             "Need to specify a direction for audio stream")
            else:
                self._logger.error("FIT test type unknown of not supported")
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "FIT test type unknown of not supported")
        elif "VOLTE" in test_type:
            if "RI" in test_type:
                upv_setup = "POLQA" + "_" + codec_type + "_" + ns_model.split('_')[1] + '_' + option
        else:
            self._logger.error("Unknown measurement detected")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown measurement detected")

        self.send_command("INSTrument ANLG")

        # Load the setup on the UPV
        self._logger.info("Load test setup %s" % upv_setup)
        self.send_command("MMEMory:LOAD:STATe 'D:/UPV/User/%s.set'" % upv_setup)

        if aa_ref_file_path:
            time.sleep(0.1)
            # Load the audio test file
            self.send_command("MMEMory:LOAD:ARBitrary '" + aa_ref_file_path + "'")

            # Timer to make sure the audio file is properly loaded on the UPV
            time.sleep(1)

    def initialization(self,
                       board_type,
                       test_type):
        """
            init() additional layer to  align RS UPV with APx585 when needed

            :type board_type: str
            :param board_type: board_type from device_catalog - Stub to align with APX585

            :type test_type: str
            :param test_type: test executed on device

            :rtype: str
            :return: executable return code

            """
        self.init()
        return 0

    def start_single_measurement(self, wait_for_result=True):
        """
            Start a single measurement on RS UPV
            """
        cmd = "INIT:CONT OFF"
        if wait_for_result:
            cmd += ";*WAI"
        self.get_logger().debug("Start a single measurement on RS UPV")
        self.send_command(cmd)

    def start_continuous_measurement(self):
        """
            Start a continuous measurement on RS UPV
            """
        self.get_logger().debug("Start a continuous measurement on RS UPV")
        self.send_command("INIT:CONT ON")

    def stop_measurement(self):
        """
            Stops a measurement on RS UPV
            """
        self.get_logger().debug("Stops a measurement on RS UPV")
        self.send_command("INIT:FORC STOP")

    def query_measurement_result(self, function_number, channel_number):
        """
            Query the result of a single measurement on current analyzer channel of RS UPV
            :type function_number: int
            :param function_number: number corresponding to the function performed by UPV analyzer
            :type channel_number: int
            :param channel_number: number corresponding to the UPV analyzer channel on which to perform the query

            :rtype: float
            :return: The function_number-th result of channel_number-th analyzer channel of RS UPV
            """
        self.get_logger().debug("Query the result of a single measurement on current analyzer channel of RS UPV")

        # Send command *CLS to the equipment to clear the Error Queue
        self.send_command("*CLS")

        # Get the main (MOS) result of the chosen channel of RS UPV
        meas_result = self.query_command("SENSe%s:DATA%s?" % (function_number, channel_number))

        return meas_result

    def store_wavefile_UPV(self, aa_deg_file_path):
        """
            Store the degraded audio file after a measurement, as a wav file

            :type aa_deg_file_path: str
            :param aa_deg_file_path: the degraded audio file to be stored in UPV
            """
        self.get_logger().debug("Store the degraded audio file after a measurement, as a wav file")

        self.send_command("MMEMORY:STORe:PWAVeform '" + aa_deg_file_path + "'")

    def audio_quality_mos(self, aa_deg_file_path, aq_test_type, stream_direction, offline=False):
        """
        Start a MOS-based (Mean Opinion Score) audio quality measurement (e.g. PESQ/POLQA)

        :type aa_deg_file_path: str
        :param aa_deg_file_path: the degraded audio file to be stored in UPV
        :type aq_test_type: str
        :param aq_test_type: Test type (PESQ/POLQA)
        :type stream_direction: str
        :param stream_direction: Stream direction for the audio (Downlink/Uplink)
        :param offline: Determines if the measurement is done in offline mode or DUT mode (real-time)
        :type offline: bool
        :rtype: float
        :return: The MOS score obtained after the measurement
        """
        # Measurement can be made in "offline" mode (out of a call)
        if offline:
            self.send_command("SENSe:FUNCtion:MMODe OFFLine")
            # Load degraded wave file
            self.send_command("MMEMory:LOAD:PWAVeform '%s'" % aa_deg_file_path)

            # Offline is by default associated with UL measurement
            self.get_logger().info("Start a %s measurement in UL" % aq_test_type)
        else:
            self.get_logger().info("Start a %s measurement in %s" % (aq_test_type, stream_direction))

        # Start PESQ/POLQA measurement on RS_UPV
        self.start_single_measurement()

        # Wait while measurement is done
        self.wait_for_meas_state("measurement_terminated")

        # Get PESQ/POLQA result value
        if "DL" in stream_direction:
            mos_result = float(self.query_measurement_result(1, 1))
        elif "UL" in stream_direction:
            mos_result = float(self.query_measurement_result(1, 2))
        else:
            self._logger.error("Unknown audio stream direction")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown audio stream direction")

        if not offline:
            # Store waveform
            self.store_wavefile_UPV(aa_deg_file_path)

        return mos_result

    def get_f0(self, time_btwn_tones, time_btwn_measurement):
        """
            Start a series of query for the estimated fundamental frequency of the audio signal

            :type time_btwn_tones: int
            :param time_btwn_tones: time in seconds between 2 successive recording tones
            :type time_btwn_measurement: int
            :param time_btwn_measurement: time in seconds between 2 successive queries

            :rtype: str list
            :return: A list of queries for fundamental frequency
            """
        self.get_logger().debug(
            "Start a series of query for the estimated fundamental frequency of the audio signal")
        f0_list = []

        # Start a continuous acquisition
        self.start_continuous_measurement()

        # Number of measure done
        meas_nb = 4 * time_btwn_tones / time_btwn_measurement

        while meas_nb > 0:
            f0 = str(self.query_command("SENSe3:DATA2?"))
            f0_list.append(f0)
            time.sleep(time_btwn_measurement)
            meas_nb -= 1

        self.stop_measurement()

        return f0_list

    def load_waveform(self, aa_ref_file_path):
        """
            Load a wave file on UPV

            :type aa_ref_file_path: str
            :param aa_ref_file_path: the reference audio file
            """
        self.get_logger().debug("Load a wave file on UPV")

        # Load the audio test file
        self.send_command("INSTrument ANLG")
        self.send_command("MMEMory:LOAD:ARBitrary '" + aa_ref_file_path + "'")

    def basic_audio_check(self, THD_target, call_duration):
        """
            Check that audio is correctly routed to DUT audio output

            :type THD_target: float
            :param THD_target: Target THD value (in dB) above which the audio signal is considered to be only noise/silence
            :type call_duration: integer
            :param call_duration: Duration (in seconds) of the voice call during which the audio check will be performed
            """

        # Start the THD measurement
        self.start_continuous_measurement()

        # Maintain the CSV call for 30s
        self.get_logger().info(
            "Check continuously that audio is correctly routed in DL and UL for %s s" % call_duration)
        time.sleep(call_duration)

        # Check that audio is correctly routed in DL and UL
        res_THD = self.query_command("SENSe1:DATA:ALL? MAX")

        # Stop the THD measurement
        self.stop_measurement()

        # Reshape raw measurement results from UPV
        res_dl = float(res_THD.split(',')[0].split(' ')[0])
        res_ul = float(res_THD.split(',')[1].split(' ')[0])

        # Compare THD values against a threshold
        if res_dl < THD_target:
            is_audio_routed_dl = 1
        else:
            is_audio_routed_dl = 0
        if res_ul < THD_target:
            is_audio_routed_ul = 1
        else:
            is_audio_routed_ul = 0

        return [is_audio_routed_dl, is_audio_routed_ul]

    def store_data_list(self, aa_trace_file_path, host_trace_file_path, option=None):
        """
            Stores an UPV measure as a data list file on the host PC
            :type aa_trace_file_path: str
            :param aa_trace_file_path: the trace file to be stored in UPV
            :type host_trace_file_path: str
            :param host_trace_file_path: the trace file to be stored in UPV
            :type option: basestring
            :param option: Specifies the type of data list to store
            """
        self.get_logger().debug("Store a measure as a data list file")

        self.send_command("MMEMory:" + option + ":STORe '" + aa_trace_file_path + "'")

        self.copy_from_upv(aa_trace_file_path, host_trace_file_path)

    def switch_trigger_source(self, source):
        """
            In case of a measurement which has to be triggered by a specific event (e.g. CH1/CH2 input reaching
            a certain amplitude level), this function selects the appropriate channel on which the trigger event will
            be based.

            :type source: basestring
            :param source: DUT/REF --> The DUT jack output is assigned by default to CH1 input of Audio Analyzer
            REF phone jack is assigned by default to CH2 input of Audio Analyzer
            """

        if source in "DUT":
            source = "CH1"
        elif source in "REF":
            source = "CH2"
        else:
            self._logger.error("Invalid measurement source (Accepted: DUT/REF)")
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, 'Invalid measurement source')

        self.get_logger().debug("Switch the trigger source to %s" % source)

        while self.query_command("SENSe7:TRIGger:SOURce?") not in source:
            self.send_command("SENSe7:TRIGger:SOURce " + source)
            time.sleep(0.1)

    def wait_for_sweep_state(self, expected_state):
        """
            Waits for sweep measurement to end.

            :type expected_state: basestring
            :param expected_state: expected status for sweep measurement state
            """

        self.get_logger().debug("Get Operation register status for sweep measurement")

        # Index of bits of the UPV operation register related to sweep status (see R&S UPV User Manual)
        op_bits_sweep = [3, 9, 8]
        timeout = 30
        t_it = 5

        self.wait_for_state(expected_state, op_bits_sweep, timeout, t_it)

    def wait_for_meas_state(self, expected_state):
        """
            Waits for measurement to end.

            :type expected_state: basestring
            :param expected_state: expected status for measurement state
            """

        self.get_logger().debug("Get Operation register status for measurement")

        # Index of bits of the UPV operation register related to measurement status (see R&S UPV User Manual)
        op_meas_bits = [4, 10]
        timeout = 30
        t_it = 1

        self.wait_for_state(expected_state, op_meas_bits, timeout, t_it)

    def wait_for_state(self, expected_state, op_reg_bits, timeout=10, t_it=1):
        """
            Waits for a specified measurement to end (based on op_reg_bits value)

            :type expected_state: basestring
            :param expected_state: expected status for sweep measurement state
            :type op_bits_meas: list
            :param op_bits_meas: Index of bits of the UPV operation register related to measurement status
            :type t_it: int
            :param t_it: duration between 2 queries for operation status
            """

        self.get_logger().debug("Get Operation register status")

        state_reached = False
        op_status_value = ""

        if len(op_reg_bits) == 3:
            op_status = self._op_status_sweep
        elif len(op_reg_bits) == 2:
            op_status = self._op_status_meas

        while state_reached is False and timeout > 0:

            op_register = "{0:b}".format(int(self.query_command("STATus:OPERation:CONDition?")))

            while len(op_register) < 15:
                op_register = "0" + op_register

            for i in op_reg_bits:
                op_status_value += "%s" % str(op_register[-i - 1])

            self.get_logger().debug("RS UPV operation status: %s" % op_status[op_status_value])

            if op_status[op_status_value] in expected_state:
                state_reached = True
                self.get_logger().info("State %s is reached" % op_status[op_status_value])
            else:
                time.sleep(t_it)

            op_status_value = ""

            timeout -= 1

        if not state_reached:
            err_msg = "Did not reach %s state" % expected_state
            self.get_logger().warning(err_msg)