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
:summary: Implements Audio methods used over the whole ACS solution
:since: 05/08/2013
:author: fbelvezx
"""

import math
from ErrorHandling.TestEquipmentException import TestEquipmentException


def detect_recording_tone(audio_analyzer,
                          time_btwn_tones,
                          f0_target,
                          logger=None):
    """
    Checks for the presence of 1kHz recording tone in the DL & UL audio signal
    :type audio_analyzer: object instance
    :param audio_analyzer: audio analyzer object instance
    :type time_btwn_tones: str
    :param time_btwn_tones: delay in seconds between two consecutives recording tones
    :type f0_target: str
    :param f0_target: expected frequency in Hz of a recording tone
    :type logger: object
    :param logger: [optional] Logger instance
    """

    # Time between 2 measurement : Chosen so that it is inferior to a recording tone duration
    time_btwn_measurement = 0.0625

    # Initiate detection algorithm variables
    t_meas_prev = 0
    tone_detected = [0, 0, 0]

    # Error margin for tone detection in kHz
    eps = 0.005

    # Start acquisition of fundamental frequency on UPV
    f0_acq = audio_analyzer.get_f0(time_btwn_tones, time_btwn_measurement)

    # Go through the acquisition data looking for measurement value around f0_target Hz
    for freq in f0_acq:
        # Make the measurement index continuous (in seconds)
        t_meas = float(f0_acq.index(freq)) * time_btwn_measurement
        freq = float(freq.split(' ')[0])

        # Check if the peak of the current FFT is at f0_target Hz, with error margin of eps
        if abs(freq - float(f0_target)) / float(f0_target) < eps:

            logger.debug("f0 = %s - t = %s s" % (freq, str(t_meas)))
            logger.debug("%s" % str(math.ceil(abs(t_meas_prev - t_meas)) % time_btwn_tones))

            # Check delay value in seconds between two tone detection
            # It is a multiple of time_btwn_measurement, and has to be either around 10s or 0s
            # (1 tone is generated every 10s)
            if math.ceil(abs(t_meas_prev - t_meas)) % time_btwn_tones in [0, 1]:
                if 10 < t_meas < 20:
                    tone_detected[0] = 1
                elif 20 < t_meas < 30:
                    tone_detected[1] = 1
                elif 30 < t_meas < 40:
                    tone_detected[2] = 1
            else:
                # False positive candidates are separated from True positive using the delay between tone : if two tones
                # are separated by a delay value superior to 1 modulo 10 seconds, it is rejected
                #
                # Then, false positive candidates's RMS peak value are checked against a threshold value
                #
                logger.debug("Detected a false positive match for recording tone")

            t_meas_prev = t_meas

    # Pass rate is set to 2 out of 3 the tones correctly detected
    if sum(tone_detected) >= 2:
        return 1
    else:
        return 0


def detect_dtmf_tone(host_trace_file,
                     dtmf_id,
                     dtmf_check,
                     logger=None):
    """
    Read a FFT trace file to check for the presence of spectral peaks corresponding to DTMF tones
    :type host_trace_file: str
    :param host_trace_file: file containing the trace of the FFT of a DTMF tone
    :type dtmf_id: str
    :param dtmf_id: str containing the dtmf character (e.g. 1,2,3,4,#,*, ...) to be detected
    :type dtmf_check: str
    :param dtmf_check: DUT/REF phone
    :type logger: object
    :param logger: [optional] Logger instance

    :rtype: boolean
    :return: a boolean for a OK/KO assessment of the DTMF detection
    """

    host_trace_file_read = open(host_trace_file, 'r')
    data_x_ch1 = []
    data_y_ch1 = []
    data_x_ch2 = []
    data_y_ch2 = []
    freq_cluster = []
    freq_peaks = []

    try:
        row = host_trace_file_read.readlines()

        # The UPV trace file data starts at line 17
        # Create one list for x coordinates (frequency in Hz), and y coordinates (Amplitude in dB) of the FFT trace
        for i in range(16, len(row) - 1):

            if dtmf_check in "DUT":
                if 'VOID' not in [row[i].split('\t')[0], row[i].split('\t')[1]]:
                    data_x_ch1.append(float(row[i].split('\t')[0]))
                    data_y_ch1.append(float(row[i].split('\t')[1]))

                    if data_y_ch1[i - 16] > data_y_ch1[i - 17] \
                        and data_x_ch1[i - 16] - data_x_ch1[i - 17] < 50 \
                        and i > 16:
                        freq_cluster.append({"dB": data_y_ch1[i - 16], "f": data_x_ch1[i - 16]})

                    elif data_y_ch1[i - 16] < data_y_ch1[i - 17] \
                        and data_x_ch1[i - 16] - data_x_ch1[i - 17] < 50 \
                        and i > 16 \
                        and freq_cluster:
                        freq_cluster.append({"dB": data_y_ch1[i - 16], "f": data_x_ch1[i - 16]})

                    elif data_x_ch1[i - 16] - data_x_ch1[i - 17] > 100 \
                        and i > 16 \
                        and freq_cluster:
                        freq_peaks.append({"f_peak": max(freq_cluster)["f"], "dB_peak": max(freq_cluster)["dB"]})
                        freq_cluster = []

            elif dtmf_check in "REF":
                if 'VOID' not in [row[i].split('\t')[5], row[i].split('\t')[6]]:
                    data_x_ch2.append(float(row[i].split('\t')[5]))
                    data_y_ch2.append(float(row[i].split('\t')[6]))

                    if data_y_ch2[i - 16] > data_y_ch2[i - 17] \
                        and data_x_ch2[i - 16] - data_x_ch2[i - 17] < 50 \
                        and i > 16:
                        freq_cluster.append({"dB": data_y_ch2[i - 16], "f": data_x_ch2[i - 16]})

                    elif data_y_ch2[i - 16] < data_y_ch2[i - 17] \
                        and data_x_ch2[i - 16] - data_x_ch2[i - 17] < 50 \
                        and i > 16 \
                        and freq_cluster:
                        freq_cluster.append({"dB": data_y_ch2[i - 16], "f": data_x_ch2[i - 16]})

                    elif data_x_ch2[i - 16] - data_x_ch2[i - 17] > 100 \
                        and i > 16 \
                        and freq_cluster:
                        freq_peaks.append({"dB_peak": max(freq_cluster)["dB"], "f_peak": max(freq_cluster)["f"]})
                        freq_cluster = []
    finally:
        host_trace_file_read.close()

    if freq_cluster and freq_peaks:
        freq_peaks.append({"dB_peak": max(freq_cluster)["dB"], "f_peak": max(freq_cluster)["f"]})

    if freq_peaks:
        freq_peaks.sort()
        if len(freq_peaks) >= 2:
            logger.info("Frequency peaks detected:")
            logger.info("f1 = %f " % (min(freq_peaks[len(freq_peaks) - 1]["f_peak"],
                                          freq_peaks[len(freq_peaks) - 2]["f_peak"])))
            logger.info("f2 = %f " % (max(freq_peaks[len(freq_peaks) - 1]["f_peak"],
                                          freq_peaks[len(freq_peaks) - 2]["f_peak"])))
            return check_dtmf_freq(dtmf_id,
                                   [min(freq_peaks[len(freq_peaks) - 1]["f_peak"],
                                        freq_peaks[len(freq_peaks) - 2]["f_peak"]),
                                    max(freq_peaks[len(freq_peaks) - 1]["f_peak"],
                                        freq_peaks[len(freq_peaks) - 2]["f_peak"])])
        else:
            return 0


def check_dtmf_freq(dtmf_id, freq_peaks):
    """
    Compares the frequency peaks detected against a dictionnary with frequency mapping with the dial keys
    :type dtmf_id: str
    :param dtmf_id: str containing the dtmf character (e.g. 1,2,3,4,#,*, ...) to be detected
    :type freq_peaks: int tuple
    :param freq_peaks: tuple containing the value in Hz of the frequency peaks detected with the audio analyzer

    :rtype: boolean
    :return: a boolean for a OK/KO assessment of the DTMF detection
    """

    dtmf_dict = {"1": [697, 1209],
                 "2": [697, 1336],
                 "3": [697, 1477],
                 "4": [770, 1209],
                 "5": [770, 1336],
                 "6": [770, 1477],
                 "7": [852, 1209],
                 "8": [852, 1336],
                 "9": [852, 1477],
                 "*": [941, 1209],
                 "0": [941, 1336],
                 "#": [941, 1477]}

    # Check that the detected frequencies correspond to the right dial key
    freq_peaks_diff = [abs(x - y) for x, y in zip(dtmf_dict[dtmf_id], freq_peaks)]
    freq_match = max(freq_peaks_diff) < 30

    return freq_match


def get_sweep_trace(host_trace_file):
    """
    Read a sweep trace file
    :type host_trace_file: str
    :param host_trace_file: trace file namey!

    :rtype: list
    :return: trace of the test signal
    """

    host_trace_file_read = open(host_trace_file, 'r')

    data_y = []

    try:
        row = host_trace_file_read.readlines()

        # The UPV trace file data starts at line 17
        for i in range(16, len(row) - 1):

            if 'VOID' not in [row[i].split('\t')[0], row[i].split('\t')[1]]:
                data_y.append(float(row[i].split('\t')[1]))

            elif 'VOID' not in [row[i].split('\t')[5], row[i].split('\t')[6]]:
                data_y.append(float(row[i].split('\t')[6]))

    finally:
        host_trace_file_read.close()

    return data_y


def get_rms_from_sweep(sweep_list, logger=None):
    """
    :type sweep_list: list
    :param sweep_list: list of RMS values corresponding to the test signal
    :type logger:
    :param logger: [optional] Logger instance

    :rtype: float
    :return: ratio of max versus min RMS value of the test signal
    """
    try:
        res = max(sweep_list) / min(sweep_list)
        return res
    except ArithmeticError as test_equipment_exception:
        logger.error(str(test_equipment_exception))
        raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, str(test_equipment_exception))
    except ValueError as test_equipment_exception:
        logger.error(str(test_equipment_exception))
        raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, str(test_equipment_exception))

# Large portions of this script came from http://stackoverflow.com/questions/892199/detect-record-audio-in-python
from array import array
from struct import pack
from sys import byteorder
import copy
import pyaudio
import wave
import time

ffmpeg_search_paths = []

def is_silent(data_chunk, silence_threshold=500):
    """Returns 'True' if below the 'silent' threshold"""
    return max(data_chunk) < silence_threshold

def normalize(data_all):
    """Amplify the volume out to max -1dB"""
    # MAXIMUM = 16384
    frame_max_value = 2 ** 15 - 1
    normalize_minus_one_db = 10 ** (-1.0 / 20)
    normalize_factor = (float(normalize_minus_one_db * frame_max_value)
                        / max(abs(i) for i in data_all))

    r = array('h')
    for i in data_all:
        r.append(int(i * normalize_factor))
    return r

def trim(data_all, threshold, sample_rate):
    _from = 0
    _to = len(data_all) - 1
    trim_append = sample_rate / 4
    for i, b in enumerate(data_all):
        if abs(b) > threshold:
            _from = max(0, i - trim_append)
            break

    for i, b in enumerate(reversed(data_all)):
        if abs(b) > threshold:
            _to = min(len(data_all) - 1, len(data_all) - 1 - i + trim_append)
            break

    return copy.deepcopy(data_all[_from:(_to + 1)])

def record(audio_format=pyaudio.paInt16, channels=1, sample_rate=44100, silence_threshold_seconds=1, silence_callback_func=None,
        audio_resume_callback_func=None, listen_len_seconds=None, silence_threshold=500):
    '''
       Records audio from the first input on the PC and returns the data as an array
       of signed shorts.

       PARAM audio_format=pyaudio.paInt16: The format of the data.
       PARAM channels=1: The number of channels to record.
       PARAM sample_rate=44100: The rate to sample data in Hz.
       PARAM silence_threshold_seconds=1: The number of seconds that silence must be
           detected before its considered to be silence.
       PARAM silence_callback_func=None: A function to call when silence is detected.
       PARAM audio_resume_callback_func=None: A function to call when audio resumes.
       PARAM listen_len_seconds=None: The number of seconds to record data. If none,
           recording continues until silence is detected.
       PARAM silence_threshold=500: Any data below this value is considered silence.

       RETURNS The audio data captured as an array of shorts.
    '''
    p = pyaudio.PyAudio()
    stream = p.open(format=audio_format, channels=channels, rate=sample_rate, input=True, output=True, frames_per_buffer=1024)

    silent_chunks = 0
    non_silent_chunks = 0
    silent_threshold_chunks = silence_threshold_seconds*44100/1024
    audio_started = False
    data_all = array('h')
    silence_offsets = list()
    silence_started = None
    first_chunk = True
    start_time = time.time()
    num_chunks = 0
    if listen_len_seconds:
        listen_time = listen_len_seconds
    else:
        # Listen for a year
        listen_time = 60*60*24*365

    while time.time()-start_time < listen_time:
        # little endian, signed short
        data_chunk = array('h', stream.read(1024))
        if byteorder == 'big':
            data_chunk.byteswap()
        data_all.extend(data_chunk)

        silent = is_silent(data_chunk, silence_threshold)

        if silent:
            silent_chunks += 1
            if silence_started==None and silent_chunks>silent_threshold_chunks:
                non_silent_chunks = 0
                if first_chunk:
                    silence_started = 0.0
                else:
                    silence_started = time.time()-start_time-silence_threshold_seconds
                if silence_callback_func!=None:
                    silence_callback_func(silence_started)
                if listen_len_seconds==None:
                    # If we didn't provide a length of time to listen, then we break on the first silence
                    break
        else:
            non_silent_chunks += 1
            if non_silent_chunks>silent_threshold_chunks:
                # If we've had enough non-silent chunks to consider ourself non-silent, reset the silent_chunks count
                silent_chunks = 0
                if silence_started != None:
                    # We were previously silent
                    silence_offsets.append((silence_started, time.time()-start_time))
                    silence_started = None
        if num_chunks>silent_threshold_chunks:
            first_chunk = False
        num_chunks += 1

    if silence_started != None:
        if listen_len_seconds!=None:
            end_time = float(listen_len_seconds)
        else:
            end_time = time.time()
        silence_offsets.append((silence_started, end_time))

    sample_width = p.get_sample_size(audio_format)
    stream.stop_stream()
    stream.close()
    p.terminate()

    # Uncomment the following lines if you want to clean up the audio
    #data_all = trim(data_all, silence_threshold, sample_rate)  # we trim before normalize as threshhold applies to un-normalized wave (as well as is_silent() function)
    #data_all = normalize(data_all)
    return data_all, silence_offsets

def save_audio_data_to_wav_file(wav_output_file_path, data, channels=1, sample_rate=44100, sample_width=16):
    '''
       Saves data to a wav file. The data parameter is most often the output of the
       record function.

       PARAM wav_output_file_path: The file path to save data to.
       PARAM data: Audio data to save to file.
       PARAM sample_width: The sample width of data.
    '''
    data = pack('<' + ('h' * len(data)), *data)
    wave_file = wave.open(wav_output_file_path, 'wb')
    wave_file.setnchannels(channels)
    wave_file.setsampwidth(sample_width)
    wave_file.setframerate(sample_rate)
    wave_file.writeframes(data)
    wave_file.close()

def verify_audio_input(output_file_path=None, silence_threshold_seconds=1,
        silence_callback_func=None, audio_resume_callback_func=None,
        listen_len_seconds=None, silence_threshold=500):
    '''
       Verifies audio is heard at the line in input.

       PARAM wav_output_file_path: Path to a file where all audio input is saved.
       PARAM silence_threshold_seconds: The number of seconds that audio is below
           silence_threshold before its considered to be true silence.
       PARAM silence_callback_func: A function to call when silence is detected.
       PARAM audio_resume_callback_func: A function to call when audio resumes.
       PARAM listen_len_seconds: The number of seconds to listen. If None, this function
           returns when silence is detected.
       PARAM silence_threshold: The raw value that audio must be below to be considered silence.

       RETURNS A list of silence offsets of the form [ (Seconds offset of silence beginning,
           Seconds offset of silence ending), ... ]
    '''
    data, silence_offsets = record(audio_format=pyaudio.paInt16, channels=1, sample_rate=44100,
        silence_threshold_seconds=silence_threshold_seconds, silence_callback_func=silence_callback_func,
        audio_resume_callback_func=audio_resume_callback_func, listen_len_seconds=listen_len_seconds,
        silence_threshold=silence_threshold)
    if output_file_path:
        output_wav_file_path = ".".join(output_file_path.split('.')[:-1]) + ".wav"
        save_audio_data_to_wav_file(output_wav_file_path, data, channels=1, sample_rate=44100, sample_width=pyaudio.get_sample_size(pyaudio.paInt16))
        if output_file_path.endswith('.wav')==False:
            # We want a non-wav file type. Lets give it a shot.
            try:
                ffmpeg_convert_file(output_wav_file_path, output_file_path, delete_input_file=True)
            except:
                print("Unable to convert file to desired file format. Saving as .wav instead")
                import traceback
                traceback.print_exc()
    return silence_offsets

def ffmpeg_convert_file(input_file_path, output_file_path, delete_input_file=False):
    '''
       Looks for the 'ffmpeg' executable and executes
       ffmpeg -i input_file_path output_file_path. This function will raise an exception
       if its unable to create the output file.

       PARAM input_file_path: A str with the path of the input file.
       PARAM output_file_path: A str with the path to where the output file should go.
       PARAM delete_input_file: A boolean indicating if the input file should be deleted
           after its used to create the output file.
    '''
    import os
    ffmpeg_cmd = _commandline_which('ffmpeg', ffmpeg_search_paths)
    ffmpeg_cmd_line = [ffmpeg_cmd, '-i', input_file_path, output_file_path]
    # An upgrade here would be to guess the timeout based on input file size
    cmd_output = _exe_command(ffmpeg_cmd_line, timeout=None)
    # Returns (Return Code, Standard Output, Standard Error)
    # ffmpeg actually outputs all its prints on stderr so ignore that
    if cmd_output[0]!=0 or os.path.exists(output_file_path)==False:
        raise Exception("Error from executing the command '%s'. Return Code: %d. STDERR: %s. STDOUT: %s."%(" ".join(ffmpeg_cmd_line), cmd_output[0], cmd_output[2], cmd_output[1]))
    if delete_input_file:
        os.remove(input_file_path)

def _commandline_which(command, additional_search_paths=[]):
    '''
       This command acts very similar to the linux 'which' command. It searches the following
       paths, looking for command:
           1. Current working directory
           2. Each of the paths in the OS environment variable PATH
           3. The paths in the argument additional_search_paths

       PARAM command: The command to search for.
       PARAM additional_search_paths: Additional paths to search.

       RETURNS The full path to the executable or raises an exception if the executable
           isn't found.
    '''
    # Reformat command name regarding os
    if _sys.platform=='win32' and command.endswith(".exe")==False:
        command += ".exe"
    # First, check to see if its in our cwd
    if _os.path.exists(command):
        return _os.path.abspath(command)
    # Check to see if its in one of our path locations
    # Get PATH environment value
    os_env_path = _os.environ.get("PATH")
    if os_env_path==None:
        raise Exception('No PATH environment variable found. Unable to located command executable.')
    # Separator is different in Linux
    if _sys.platform=='win32':
        path_list = os_env_path.split(';')
    else:
        path_list = os_env_path.split(':')
    path_list.extend(additional_search_paths)
    for path in path_list:
        file_path = _os.path.join(path, command)
        if _os.path.exists(file_path):
            return file_path
    raise Exception('Unable to find executable for command '+command)

# BEGIN Code copied from PythonSV
# https://subversion.jf.intel.com/deg/pve/csv/pythonsv/trunk/tangier/user/cgsapp/exe_runner.py

import time as _time
import threading as _threading
import subprocess as _subprocess
import sys as _sys
import os as _os

class ExeCommand(object):
    def __init__(self):
        self.stdout = ""
        self.stdout_lock = _threading.Lock()
        self.stderr = ""
        self.stderr_lock = _threading.Lock()
        self.timeout = 60*60*24*365 # 1 Year
        self.exe_and_args = None
        self.blocking = True
        self.return_code = None
        self.stdout_print = False
        self.stderr_print = False
    def execute(self):
        '''This function takes care of kicking off the exe and monitoring the output.'''
        self.return_code = None
        self.stdout = ""
        self.stderr = ""
        start_time = _time.time()
        process = _subprocess.Popen(self.exe_and_args, shell=False, stdout=_subprocess.PIPE,
            stderr=_subprocess.PIPE)
        stdout_thread = _threading.Thread(target=self._copy_file_to_stdout, args=(process.stdout,))
        stdout_thread.start()
        stderr_thread = _threading.Thread(target=self._copy_file_to_stderr, args=(process.stderr,))
        stderr_thread.start()
        if self.blocking:
            self._monitor(process)
            return (self.get_return_code(), self.get_stdout(), self.get_stderr())
        else:
            monitor_thread = _threading.Thread(target=self._monitor, args=(process,))
            monitor_thread.start()
    def _copy_file_to_stdout(self, file_object, read_chunk_size=1 ):
        '''This function is used to spawn the copying of stdout to self.stdout onto
        another thread so its not blocking other tasks.'''
        read_data = file_object.read(read_chunk_size)
        while read_data != "":
            if self.stderr_print:
                _sys.stderr.write(read_data)
            self.stdout_lock.acquire()
            self.stdout += read_data
            self.stdout_lock.release()
            read_data = file_object.read(read_chunk_size)
    def _copy_file_to_stderr(self, file_object, read_chunk_size=1 ):
        '''This function is used to spawn the copying of stdout to self.stdout onto
        another thread so its not blocking other tasks.'''
        read_data = file_object.read(read_chunk_size)
        while read_data != "":
            if self.stdout_print:
                _sys.stdout.write(read_data)
            self.stderr_lock.acquire()
            self.stderr += read_data
            self.stderr_lock.release()
            read_data = file_object.read(read_chunk_size)
    def _monitor(self, process):
        start_time = _time.time()
        while _time.time()<start_time+self.timeout:
            self.return_code = process.poll()
            if self.return_code!=None:
                return self.return_code
        # Time has expired
        process.terminate()
        raise Exception("Timeout waiting for process to complete.")
    def set_blocking(self, blocking_mode=True):
        self.blocking = blocking_mode
    def set_exe_and_args(self, exe_and_args):
        self.exe_and_args = exe_and_args
    def set_timeout(self, seconds):
        self.timeout = seconds
    def set_print(self, stdout=False, stderr=False):
        self.stdout_print = stdout
        self.stderr_print = stderr
    def get_return_code(self):
        return self.return_code
    def get_stdout(self, erase_on_read=False):
        if erase_on_read:
            self.stdout_lock.acquire()
        return_data = self.stdout
        if erase_on_read:
            self.stdout = ""
            self.stdout_lock.release()
        return return_data
    def get_stderr(self, erase_on_read=False):
        if erase_on_read:
            self.stderr_lock.acquire()
        return_data = self.stderr
        if erase_on_read:
            self.stderr = ""
            self.stderr_lock.release()
        return return_data

def _exe_command(exe_and_args, background_process=False, timeout=None, print_output=False):
    '''Returns a tuple with (Return Code, Standard Output, Standard Error) if the process wasn't backgrounded.'''
    cmd = ExeCommand()
    cmd.set_exe_and_args(exe_and_args)
    cmd.set_blocking(not background_process)
    if timeout:
        cmd.set_timeout(timeout)
    cmd.set_print(print_output, print_output)
    return cmd.execute()

# END Code copied from PythonSV

# Plays an audio file on the host computer. This code is being ported from
# AudioHost.py which is currently implemented as a uecmd for Android. We should
# use this going forward as this capability pretains more to the host.
import threading
import pyaudio
import wave
import os.path
from ErrorHandling.AcsConfigException import AcsConfigException

class AudioPlayer():

    """
    Class that handle audio playback on host side.
    """

    def __init__(self, logger):
        """
        Constructor
        """
        self._logger = logger
        self.pc_hostplayer = None
        self._logger.info("constructor in audioutils.player");

    def pc_audio_playback_start(self, input_file, output_device_index=None, loop=False):
        """
        Start playing the audio on PC

        :type input_file: str
        :param input_file: which file used to play sound
        :type output_device_index: int
        :param output_device_index: which device used to play sound

        :return: None
        """
        if self.pc_hostplayer is not None:
            self._logger.info("PC playback already in progress. Stop previous playback on pc...")
            self.pc_hostplayer.stop_hostplay()
            self.pc_hostplayer = None

        self._logger.info("Start playback on PC ...")
        if not os.path.exists(input_file):
            raise TestEquipmentException(AcsConfigException.INVALID_PARAMETER,
                                     "PC playback file doesn't exist. %s" % input_file)
        # todo, check output_device_index valid
        self._logger.info("Host play file: %s" % input_file)
        if output_device_index is not None:
            self._logger.info("Host play devices: %d" % output_device_index)
        else:
            self._logger.info("Host play devices: default")
        self.pc_hostplayer = HostPlayer(input_file, output_device_index, loop)
        self.pc_hostplayer.start_hostplay()

    def pc_audio_playback_stop(self):
        """
        Stop playing the audio on PC

        :return: None
        """
        if self.pc_hostplayer is None:
            self._logger.info("PC playback already stopped")
        else:
            self._logger.info("Stop playback on PC ...")
            self.pc_hostplayer.stop_hostplay()
            self.pc_hostplayer = None


class HostPlayer():

    """
    HostPlay, play audio on pc
    """

    def __init__(self, input_file_name, output_device_index=None, loop=False):
        """
        Constructor
        """
        self._hostplay_event = threading.Event()
        self._input_file_name = input_file_name
        self._output_device_index = output_device_index
        self._loop = loop

    def start_hostplay(self):
        self._hostplay_event.set()

        playback_handler = HostPlayHandler(self._hostplay_event,
                                           self._input_file_name,
                                           self._output_device_index,
                                           self._loop)
        playback_handler.start()

    def stop_hostplay(self):
        self._hostplay_event.clear()


class HostPlayHandler(threading.Thread):

    """
    HostPlayHandler, handles the audio playback on pc
    """

    def __init__(self, hostplay_event, input_file_name, output_device_index=None, loop=False):
        threading.Thread.__init__(self,)

        self._hostplay_event = hostplay_event
        self._input_file_name = input_file_name
        self._output_device_index = output_device_index
        self._chunk_size = 1024
        self._loop = loop;

    def run(self):
        # global playback_event
        pa = pyaudio.PyAudio()
        wave_file = wave.open(self._input_file_name, 'rb')

        # prepare the out stream
        pa_stream = pa.open(format=pa.get_format_from_width(wave_file.getsampwidth()),
                            channels=wave_file.getnchannels(),
                            rate=wave_file.getframerate(),
                            output=True,
                            output_device_index=self._output_device_index)

        # start playback
        data = wave_file.readframes(self._chunk_size)
        while self._hostplay_event.isSet():
            pa_stream.write(data)
            data = wave_file.readframes(self._chunk_size)
            if data == '':
                if self._loop:
                    wave_file.rewind()
                else:
                    break

        # close the input stream
        wave_file.close()

        # close the output stream
        pa_stream.stop_stream()
        pa_stream.close()
        pa.terminate()
