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
:summary: virtual interface with audio analyzers
:since:12/04/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IAudioAnalyzer(object):

    """
    Virtual interface for audio analyzers equipments
    """

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def perform_full_preset(self):
        """
        Resets all equipments parameters to their default values
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_configuration_file(self, load_file):
        """
        Loads a configuration file
        :type load_file: str
        :param load_file: the configuration file to load
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def save_configuration_file(self, save_file):
        """
        Saves the configuration in a file
        :type save_file: str
        :param save_file: the destination file to save configuration
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def copy_to_upv(self, source, dest):
        """
        Copies a file to the RS UPV
        :type source: str
        :param source: the source file to copy
        :type dest: str
        :param dest: the destination file for the copy
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def copy_from_upv(self, source, dest):
        """
        Copies a file from the RS UPV
        :type source: str
        :param source: the source file to copy
        :type dest: str
        :param dest: the destination file for the copy
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def play(self, src_file):
        """
        Plays an audio file on RS UPV
        :type src_file: str
        :param src_file: the audio file to play
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop(self):
        """
        Stop playing the audio file
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_command(self, command):
        """
        Send a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def query_command(self, command):
        """
        Query a GPIB Command.

        :type command: str
        :param command: String representing a well formed GPIB command.

        :rtype: str
        :return: The response from agilent for the command
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_configuration(self,  function, ns_model=None, aa_ref_file_path=None, codec_type=None, option=[]):
        """
        Launch a configuration for Narrow-band audio codec in RS UPV firmware

        :type function: str
        :param function: type of measurement to perform
        :type ns_model: str
        :param ns_model: str identifying the network simulator
        :type aa_ref_file_path: str
        :param aa_ref_file_path: the reference audio file
        :type codec_type: str
        :param codec_type: the type of audio codec (NB, WB, SWB) to be tested
        :type option: str
        :param option: additional option
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_single_measurement(self):
        """
        Start a single measurement on RS UPV
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_continuous_measurement(self):
        """
        Start a continuous measurement on RS UPV
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_measurement(self):
        """
        Stops a measurement on RS UPV
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def store_wavefile_UPV(self, aa_deg_file_path):
        """
        Store the degraded audio file after PESQ measurement, as a wav file

        :type aa_ref_file_path: str
        :param aa_ref_file_path: the degraded audio file to be stored in UPV
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_pesq(self, aa_deg_file_path, stream_direction):
        """
        Start a PESQ measurement

        :type aa_deg_file_path: str
        :param aa_deg_file_path: the degraded audio file to be stored in UPV
        :type stream_direction: str
        :param stream_direction: Stream direction for the audio (Downlink/Uplink)

        :rtype: float
        :return: The PESQ score obtained after the measurement
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release_visa_equipment(self):
        """
        Release UPV equipment with pyvisa interface
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_visa_equipment(self):
        """
        Get RS equipment with pyvisa interface
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def __manage_apx585(self,
                        execution_type,
                        board_type=None,
                        test_type=None,
                        call_type=None,
                        accessories_type=None,
                        signal_tested_direction=None,
                        dut_bt_address=None):
        """
        launch a command to APx585
        :type execution_type: str
        :param execution_type: Init, pair_bt or run
        :type board_type: str
        :param board_type: board_type from device_catalog
        :type test_type: str
        :type call_type: str
        :param call_type: 2G, 3G, VOIP
        :param test_type: test executed on device
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp
        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL
        :type dut_bt_address: str
        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def initialization(self,
                       board_type,
                       test_type):
        """
        Init Apx585
        :type board_type: str
        :param board_type: board_type from device_catalog
        :type test_type: str
        :param test_type: test executed on device

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def pair_bt(self,
                dut_bt_address):
        """
        Pair Bluetooth between Apx585 and Dut

        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def connect_bt(self,
                   accessories_type,
                   dut_bt_address):
        """
        Connect Bluetooth between Apx585 and Dut

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset, bluetootha2dp, bluetoothhsp

        :type accessories_type: str
        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def run(self,
            call_type,
            accessories_type,
            signal_tested_direction):
        """
        Run Apx585 test

        :type call_type: str
        :param call_type: 2G, 3G, VOIP

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset, bluetootha2dp, bluetoothhsp

        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def calibration(self,
                    board_type,
                    test_type,
                    acc_type,
                    signal_tested_direction,
                    dut_bt_address):
        """
        Run Apx585 calibration

        :type board_type: str
        :param board_type: board_type from device_catalog

        :type test_type: str
        :param test_type: test executed on device

        :type call_type: str
        :param call_type: 2G, 3G, VOIP

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset, bluetootha2dp, bluetoothhsp

        :type signal_tested_direction: str
        :param signal_tested_direction: UL, DL

        :type dut_bt_address: str
        :param dut_bt_address: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_f0(self, time_btwn_tones, time_between_measurement):
        """
        Start a series of query for the estimated fundamental frequency of the audio signal

        :type time_btwn_tones: int
        :param time_btwn_tones: time in seconds between 2 successive recording tones

        :type time_between_measurement: int
        :param time_between_measurement: time in seconds between 2 successive queries

        :rtype: list
        :return: A list of queries for fundamental frequency

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def load_waveform(self, aa_ref_file_path):
        """
        Load a wave file on UPV

        :type aa_ref_file_path: str
        :param aa_ref_file_path: the reference audio file
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def basic_audio_check(self, THD_target, call_duration):
        """
        Check that audio is correctly routed to DUT audio output

        :type THD_target: float
        :param THD_target: Target THD value (in dB) above which the audio signal is considered to be only noise/silence
        :type call_duration: integer
        :param call_duration: Duration (in seconds) of the voice call during which the audio check will be performed
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_volume(self,
                   call_type=None,
                   accessories_type=None):
        """
        launch a command to APx585

        :type call_type: str
        :param call_type: 2G, 3G, VOIP

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset, bluetootha2dp, bluetoothhsp

        :rtype: str
        :return: executable return code
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def write_pipe(self, message="none"):
        """
        Write a message in a pipe
        :type message: str
        :param message: message write in the pipe

        :rtype: None
        :raise UECommandException: if message superior at the buffer size
                                   or message is not write on the pipe
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def read_pipe(self):
        """
        Read a message in a pipe

        :rtype: str
        :return: the message read on the pipe
        :raise UECommandException: if timeout is reached
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def create_pipe(self):
        """
        Create and connect pipe

        :rtype: None
        :raise UECommandException: if message superior at the buffer size
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def close_pipe(self):
        """
        Create and connect pipe

        :rtype: None
        :raise UECommandException: Pipe handle is None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_dtmf(self,
                 accessories_type=None,
                 call_direction=None,
                 key_touch=None):
        """
        launch a command to APx585

        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset, bluetootha2dp, bluetoothhsp

        :type call_direction: str
        :param call_direction: UL, DL

        :type key_touch: str
        :param key_touch: dut mac bluetooth address

        :rtype: str
        :return: executable return code

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def switch_trigger_source(self, source):
        """
        In case of a measurement which has to be triggered by a specific event (e.g. CH1/CH2 input reaching
        a certain amplitude level), this function selects the appropriate channel on which the trigger event will
        be based.

        :type source: basestring
        :param source: DUT/REF --> The DUT jack output is assigned by default to CH1 input of Audio Analyzer
        REF phone jack is assigned by default to CH2 input of Audio Analyzer
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def wait_for_sweep_state(self, expected_state):
        """
        Waits for sweep measurement to end.

        :type expected_state: basestring
        :param expected_state: expected status for sweep measurement state
        :type timeout: int
        :param timeout: timeout for UPV to reach expected state
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def ringtone_detection(self,
                           accessories_type=None):
        """
        launch a command to APx585 to detect ringtone
        :type accessories_type: str
        :param accessories_type: accessories type: earpiece, speaker, headset,
                                bluetootha2dp, bluetoothhsp

        :rtype: str
        :return: executable return code
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)