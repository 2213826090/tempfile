#!/usr/bin/env python

########################################################################
#
# @filename:    audio_processing.py
# @description: audio digital signal processing library
# @author:      saddam.hussain.abbas@intel.com
#
########################################################################

import pydub
import numpy as np
import commands
import argparse

class audioProcessing():

    def __init__(self):
        self.pydub = pydub
        # seconds to sample audio file for
        self.sample_time = 500
        # number of points to scan cross correlation over
        self.span = 15
        # step size (in points) of cross correlation
        self.step = 1
        # minimum number of points that must overlap in cross correlation
        # exception is raised if this cannot be met
        self.min_overlap = 20
        # report match when cross correlation has a peak exceeding threshold
        self.threshold = 0.5

    def __get_audio_format(self, audio_file):
        return audio_file.split('.')[-1]

    def trim_audio(self, file_name, format = None, silence_thresh = -60,
                   min_silence_len_to_detect = 100,
                   position = "ends"):
        if format is None:
            format = self.__get_audio_format(file_name)

        if position == "ends":
            position = ["initial", "final"]
        else:
            position = [position.lower()]

        sound = self.pydub.AudioSegment.from_file(file_name, format=format)

        silence_ranges = self.pydub.silence.detect_silence(sound,
                        min_silence_len=min_silence_len_to_detect,
                        silence_thresh=silence_thresh)

        silence_ranges = np.array(silence_ranges)

        # return the original final when no silence is detected
        if silence_ranges.size == 0:
            print 'returning as no silence'
            return file_name
        duration = round(sound.duration_seconds * 1000)

        silence_ranges_dimension = silence_ranges.ndim

        trim_range = []
        if "initial" in position:
            if silence_ranges[0, 0] == 0 and silence_ranges[0, 1] != duration:
                trim_range.append(silence_ranges[0, 1])
            else:
                trim_range.append(0)
        else:
            trim_range.append(0)

        if "final" in position:
            if silence_ranges[-1, 1] == duration:
                trim_range.append(silence_ranges[-1, 0])
            else:
                trim_range.append(duration)
        else:
            trim_range.append(duration)

        #print "trim range ", trim_range

        # return original audio if no need to remove any silence
        if trim_range == [0, duration]:
            return file_name

        sound = sound[trim_range[0]: trim_range[1]]

        temp_file_name = file_name.split('.')
        if len(temp_file_name) > 1:
            file_name = '.'.join(temp_file_name[0:-1]) + ".trimmed" + "." + \
                        temp_file_name[-1]
        else:
            file_name = '.'.join(temp_file_name[0:-1]) + ".trimmed"

        sound.export(file_name , format = format)
        return file_name

    # calculate fingerprint
    def calculate_fingerprints(self, filename):
        fpcalc_out = commands.getoutput('fpcalc -raw -length %i %s'
                                        % (self.sample_time, filename))
        fingerprint_index = fpcalc_out.find('FINGERPRINT=') + 12
        # convert fingerprint to list of integers
        fingerprints = map(int, fpcalc_out[fingerprint_index:].split(','))

        return fingerprints

    # returns correlation between lists
    def correlation(self, listx, listy):
        if len(listx) == 0 or len(listy) == 0:
            # Error checking in main program should prevent us from ever being
            # able to get here.
            raise Exception('Empty lists cannot be correlated.')
        if len(listx) > len(listy):
            listx = listx[:len(listy)]
        elif len(listx) < len(listy):
            listy = listy[:len(listx)]

        covariance = 0
        for i in range(len(listx)):
            covariance += 32 - bin(listx[i] ^ listy[i]).count("1")
        covariance = covariance / float(len(listx))

        return covariance / 32

    # return cross correlation, with listy offset from listx
    def cross_correlation(self, listx, listy, offset):
        if offset > 0:
            listx = listx[offset:]
            listy = listy[:len(listx)]
        elif offset < 0:
            offset = -offset
            listy = listy[offset:]
            listx = listx[:len(listy)]
        if min(len(listx), len(listy)) < self.min_overlap:
            # Error checking in main program should prevent us from ever being
            # able to get here.
            return
            # raise Exception('Overlap too small: %i' % min(len(listx), len(listy)))
        return self.correlation(listx, listy)

    # cross correlate listx and listy with offsets from -span to span
    def compare(self, listx, listy, span, step):
        if span > min(len(listx), len(listy)):
            # Error checking in main program should prevent us from ever being
            # able to get here.
            raise Exception('span >= sample size: %i >= %i\n'
                            % (span, min(len(listx), len(listy)))
                            + 'Reduce span, reduce crop or increase sample_time.')
        corr_xy = []
        for offset in np.arange(-span, span + 1, step):
            corr_xy.append(self.cross_correlation(listx, listy, offset))
        return corr_xy

    # return index of maximum value in list
    def max_index(self, listx):
        max_index = 0
        max_value = listx[0]
        for i, value in enumerate(listx):
            if value > max_value:
                max_value = value
                max_index = i
        return max_index

    def get_max_corr(self, corr, source, target):
        max_corr_index = self.max_index(corr)
        max_corr_offset = -self.span + max_corr_index * self.step
        print "max_corr_index = ", max_corr_index, "max_corr_offset = ", max_corr_offset
        # report matches
        if corr[max_corr_index] > self.threshold:
            print('%s and %s match with correlation of %.4f at offset %i'
                  % (source, target, corr[max_corr_index], max_corr_offset))

    def correlate(self, source, target):
        fingerprint_source = self.calculate_fingerprints(source)
        fingerprint_target = self.calculate_fingerprints(target)

        corr = self.compare(fingerprint_source, fingerprint_target, self.span,
                       self.step)

        max_corr_offset = self.get_max_corr(corr, source, target)

def initialize():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i ", "--source-file", help="source file")
    parser.add_argument("-o ", "--target-file", help="target file")
    args = parser.parse_args()

    SOURCE_FILE = args.source_file if args.source_file else None
    TARGET_FILE = args.target_file if args.target_file else None
    if not SOURCE_FILE or not TARGET_FILE:
        raise Exception("Source or Target files not specified.")
    return SOURCE_FILE, TARGET_FILE

if __name__ == "__main__":
    o = audioProcessing()
    SOURCE_FILE, TARGET_FILE = initialize()

    #TARGET_FILE = '/home/saddamhu/Downloads/audioSamples/R1.wav'
    #SOURCE_FILE = '/home/saddamhu/Downloads/audioSamples/1.wav'

    SOURCE_FILE = o.trim_audio(SOURCE_FILE, position="ends",
                               silence_thresh=-60)

    TARGET_FILE = o.trim_audio(TARGET_FILE, position="ends",
                               silence_thresh=-60)

    print SOURCE_FILE, TARGET_FILE

    o.correlate(SOURCE_FILE, TARGET_FILE)
