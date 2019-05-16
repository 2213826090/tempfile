from string import Template
from PyUiApi.common.shell_utils import *
from PyUiApi.common.environment_utils import *
import datetime
import numpy
import scipy.io.wavfile


class HostAudioRecorder(object):
    record_cmd = Template("arecord -d $duration -r 16000 -f S16_LE -c1 -t wav $recording_path")

    def __init__(self):
        self.recordings = []

    def start_recording(self, duration, recording_path=None):
        if recording_path is None:
            recording_path = os.path.join(Environment.tmp_dir_path, self.gen_temp_recording_name())
        cmd_string = HostAudioRecorder.record_cmd.substitute(duration=duration, recording_path=recording_path)
        recording_proc = ShellUtils.run_cmd_in_process(cmd_string)
        self.recordings.append(recording_path)
        return recording_proc

    def gen_temp_recording_name(self):
        time_string = datetime.datetime.strftime(datetime.datetime.now(), '%Y_%m_%d_%H_%M_%S')
        file_name = time_string + "_" + "recording.wav"
        return file_name

    def delete_all_recordings(self):
        for recording_file_path in self.recordings:
            os.remove(recording_file_path)


class AudioAnalysisUtils(object):
    threshold_amplitude = 5
    rec_silence_max_amplitude = 10

    def get_spectogram(self, wav_file):

        sample_rate, data = scipy.io.wavfile.read(wav_file)

        sampling_step = int(sample_rate * 0.01)
        sampling_window = int(sample_rate * 0.03)
        smoothing_matrix = numpy.hamming(sampling_window)
        intervals = range(sampling_window, len(data), sampling_step)

        spectogram = numpy.zeros((len(intervals), sampling_window / 2))

        for i, n in enumerate(intervals):
            sampling_window_data = data[n - sampling_window:n]
            z = numpy.fft.fft(smoothing_matrix * sampling_window_data,
                              sampling_window)
            spectogram[i, :] = numpy.log(numpy.abs(z[:sampling_window / 2]))
        return spectogram

    def get_spectograms_sample_differences(self, spectogram1, spectogram2):
        difference_coefficients = []
        for i in range(min(len(spectogram1),
                           len(spectogram2))):
            nr_of_freq = 0
            total_disp = 0
            window1 = spectogram1[i]
            window2 = spectogram2[i]

            small_window, big_window = self.get_small_big_matrix(window1,
                                                                 window2)
            for j in range(len(small_window)):
                nr_of_freq += 1
                if abs(window1[j]) > self.threshold_amplitude or \
                        abs(window2[j]) > self.threshold_amplitude:
                    error = abs(window1[j] - window2[j]) / float(window1[j])
                    total_disp += error
            for k in range(len(small_window), len(big_window)):
                nr_of_freq += 1
                total_disp += big_window[k]
            window_error = total_disp / nr_of_freq
            difference_coefficients.append(window_error)
        return difference_coefficients

    def get_small_big_matrix(self, m1, m2):
        if len(m1) >= len(m2):
            return m2, m1
        else:
            return m1, m2

    def get_starting_silence_length(self, spectogram):
        index = 0
        for i in range(len(spectogram)):
            max_ampl = max(spectogram[i])
            if max_ampl > self.rec_silence_max_amplitude:
                return index
            index += 1

    def get_ending_silence_start(self, spectogram):
        index = len(spectogram)
        for i in reversed(range(index)):
            max_ampl = max(spectogram[i])
            if max_ampl > self.rec_silence_max_amplitude:
                return index
            index -= 1


if __name__ == "__main__":
    utils = AudioAnalysisUtils()
    spectogram = utils.get_spectogram("reference_audio_mono.wav")