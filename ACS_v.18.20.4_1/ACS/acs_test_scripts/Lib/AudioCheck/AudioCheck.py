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
:summary: Methods for audio check
:since: 30/01/14
:author: jcoutox
"""

import pyaudio
import wave
import numpy as np
import struct
import math


def fft_on_wav_file(file):
    """
    Read in a WAV and find the freq of fundamental for one note and return list of fundamental freq.
    :type file: str
    :param file: Path to audio file to analyse
    :rtype: list
    :return: Float list of freq find in WAV
    """
    chunk = 2048
    list_freq = []

    # open up a wave
    wf = wave.open(file, 'rb')
    swidth = wf.getsampwidth()
    RATE = wf.getframerate()
    # use a Blackman window
    window = np.blackman(chunk)
    # open stream
    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()), channels=wf.getnchannels(), rate=RATE,
                    output=True)

    # read some data
    data = wf.readframes(chunk)
    # play stream and find the frequency of each chunk
    while len(data) == chunk * swidth:
        # write data out to the audio stream
        stream.write(data)
        # unpack the data and times by the hamming window
        indata = np.array(wave.struct.unpack("%dh" % (len(data) / swidth), data)) * window
        # Take the fft and square each value
        fftData = abs(np.fft.rfft(indata)) ** 2
        # find the maximum
        which = fftData[1:].argmax() + 1
        # use quadratic interpolation around the max
        if which != len(fftData) - 1:
            y0, y1, y2 = np.log(fftData[which - 1:which + 2:])
            x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
            # find the frequency and output it
            thefreq = (which + x1) * RATE / chunk
            #Add freq at list_freq[]
            list_freq.append(thefreq)

        else:
            thefreq = which * RATE / chunk
            list_freq.append(thefreq)
            # read some more data
        data = wf.readframes(chunk)

    if data:
        stream.write(data)
    stream.close()
    p.terminate()

    return list_freq

def extract_sequence(freq):
    """
    Replace remarkable frequency by this corresponding note in third octave and extract the sequence.
    :type freq: list
    :param freq: frequency float list
    :rtype: list
    :return: note str list
    """

    #define remarkable frequency
    DO = [260.0, 262.99]
    RE = [292.0, 294.99]
    MI = [328.0, 330.99]
    FA = [348.0, 350.99]
    SOL = [391.0, 393.99]
    LA = [439.0, 441.99]
    SI = [492.0, 494.99]
    DO4 = [522.0, 524.99]

    #init return list
    note = []

    #the frequency list is rotated
    for thefreq in freq:
        #Check if the frequency is an DO
        if (thefreq >= DO[0] and thefreq <= DO[1]):
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("DO")
                #Check if the note is already save in list
            if note[-1] != "DO":
                #if is not in last position, add the note at note
                note.append("DO")
        #Check if the frequency is an RE
        elif thefreq >= RE[0] and thefreq <= RE[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("RE")
                #Check if the note is already save in list
            if note[-1] != "RE":
                #if is not in last position, add the note at note
                note.append("RE")
        #Check if the frequency is an MI
        elif thefreq >= MI[0] and thefreq <= MI[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("MI")
                #Check if the note is already save in list
            if note[-1] != "MI":
                note.append("MI")
        #Check if the frequency is an FA
        elif thefreq >= FA[0] and thefreq <= FA[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("FA")
                #Check if the note is already save in list
            if note[-1] != "FA":
                #if is not in last position, add the note at note
                note.append("FA")
        #Check if the frequency is an SOL
        elif thefreq >= SOL[0] and thefreq <= SOL[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("SOL")
                #Check if the note is already save in list
            if note[-1] != "SOL":
                #if is not in last position, add the note at note
                note.append("SOL")
        #Check if the frequency is an LA
        elif thefreq >= LA[0] and thefreq <= LA[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("LA")
                #Check if the note is already save in list
            if note[-1] != "LA":
                #if is not in last position, add the note at note
                note.append("LA")
        #Check if the frequency is an SI
        elif thefreq >= SI[0] and thefreq <= SI[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("SI")
                #Check if the note is already save in list
            if note[-1] != "SI":
                #if is not in last position, add the note at note
                note.append("SI")
        #Check if the frequency is an DO4
        elif thefreq >= DO4[0] and thefreq <= DO4[1]:
            #Check if the return list is empty
            if len(note) == 0:
                #If note is empty add the note
                note.append("DO4")
                #Check if the note is already save in list
            if note[-1] != "DO4":
                #if is not in last position, add the note at note
                note.append("DO4")
    return note

class DTMFdetector(object):
    """
    This class is used to detect DTMF tones in a WAV file
    """

    def __init__(self):
        """
        Constructor of the class. Initializes the instance variables and pre-calculates the coefficients.

        :rtype: None
        :return: None
        """

        #Define some constants for the Goertzel algorithm
        self._MAX_BINS = 8
        self._GOERTZEL_N = 590
        self._SAMPLING_RATE = 44100

        #The frequencies we're looking for
        self._freqs = [697, 770, 852, 941, 1209, 1336, 1477, 1633]

        #The coefficients
        self._coefs = [0, 0, 0, 0, 0, 0, 0, 0]

        self.reset()
        self.calc_coeffs()

    def reset(self):
        """
        This will reset all the state of the detector.

        :rtype: None
        :return: None
         """

        #The index of the current sample being looked at
        self._sample_index = 0

        #The counts of samples we've seen
        self._sample_count = 0

        #First pass
        self._q1 = [0, 0, 0, 0, 0, 0, 0, 0]

        #Second pass
        self._q2 = [0, 0, 0, 0, 0, 0, 0, 0]

        #r values
        self._r = [0, 0, 0, 0, 0, 0, 0, 0]

        #This stores the characters seen so far and the times they were seen at for post, post processing
        self._characters = []

        #This stores the final list of characters we believe the audio contains
        self._keytone_list = []

    def post_testing(self):
        """
        Post testing for algorithm figures out what's a valid signal and what's not.

        :rtype: None
        :return: None
        """
        row = 0
        col = 0
        see_digit = False
        peak_count = 0
        max_index = 0
        maxval = 0.0
        t = 0
        i = 0

        row_col_ascii_codes = [["1", "2", "3", "A"], ["4", "5", "6", "B"], ["7", "8", "9", "C"], ["*", "0", "#", "D"]]

        #Find the largest in the row group.
        for i in range(4):
            if self._r[i] > maxval:
                maxval = self._r[i]
                row = i

        #Find the largest in the column group.
        col = 4
        maxval = 0
        for i in range(4, 8):
            if self._r[i] > maxval:
                maxval = self._r[i]
                col = i

        #Check for minimum energy
        if self._r[row] < 4.0e5:
            see_digit = False
        elif self._r[col] < 4.0e5:
            see_digit = False
        else:
            see_digit = True

            #Normal twist
            if self._r[col] > self._r[row]:
                max_index = col
                if self._r[row] < (self._r[col] * 0.398):
                    see_digit = False
            #Reverse twist
            else:
                max_index = row
                if self._r[col] < (self._r[row] * 0.158):
                    see_digit = False


            #Signal to noise test
            #The noise must be 16dB down from the signal. Here we count the number of signals above the threshold and
            #there ought to be only two.
            if self._r[max_index] > 1.0e9:
                t = self._r[max_index] * 0.158
            else:
                t = self._r[max_index] * 0.010

            peak_count = 0

            for i in range(8):
                if self._r[i] > t:
                    peak_count = peak_count + 1
            if peak_count > 2:
                see_digit = False

            if see_digit:
                #stores the character found, and the time in the file in seconds in which the file was found
                self._characters.append(
                    (row_col_ascii_codes[row][col - 4], float(self._sample_index) / float(self._SAMPLING_RATE)))

    def clean_up_processing(self):
        """
        This takes the number of characters found and such and figures out what's a distinct key press.
        So say you pressed 5,3,2,1,1. The algorithm sees 555553333332222221111111111111.
        Cleaning up gives you 5,3,2,1,1.

        :rtype: None
        :return: None
        """

        #This is nothing but a fancy state machine to get a valid key press we need
        MIN_CONSECUTIVE = 2
        #Characters in a row with no more than
        MAX_GAP = 0.3000

        currentCount = 0
        lastChar = ""
        lastTime = 0
        charIndex = -1

        for i in self._characters:

            charIndex += 1
            currentChar = i[0]
            currentTime = i[1]
            timeDelta = currentTime - lastTime

            #check if this is the same char as last time
            if lastChar == currentChar:
                currentCount += 1
            else:
                #Some times it seems we'll get a stream of good input, then some erronous input will just once.
                # So what we're gonna do is peak ahead here and see what if it goes back to the pattern we're getting
                # and then decide if we should let it go, stop th whole thing.
                if len(self._characters) > (charIndex + 2):
                    if (self._characters[charIndex + 1][0] == lastChar) and (
                            self._characters[charIndex + 2][0] == lastChar):
                        #forget this every happened
                        lastTime = currentTime
                        continue

                #Check to see if we have a valid key press on our hands
                if currentCount >= MIN_CONSECUTIVE:
                    self._keytone_list.append(lastChar)
                    currentCount = 1
                    lastChar = currentChar
                    lastTime = currentTime
                    continue

            #Check to see if we have a big enough gap to make us think we've got a new key press
            if timeDelta > MAX_GAP:
                #So de we have enough counts for this to be valid?
                if (currentCount - 1) >= MIN_CONSECUTIVE:
                    self._keytone_list.append(lastChar)
                currentCount = 1

            lastChar = currentChar
            lastTime = currentTime

        #Check the end of the characters
        if currentCount >= MIN_CONSECUTIVE:
            self._keytone_list.append(lastChar)

    def goertzel(self, sample):
        """
        The Goertzel algorithm takes in a 16 bit signed sample

        :type sample: int
        :param sample: 16 bit signed sample
        :rtype: None
        :return: None
        """

        q0 = 0
        i = 0

        self._sample_count += 1
        self._sample_index += 1

        for i in range(self._MAX_BINS):
            q0 = self._coefs[i] * self._q1[i] - self._q2[i] + sample
            self._q2[i] = self._q1[i]
            self._q1[i] = q0

        if self._sample_count == self._GOERTZEL_N:
            for i in range(self._MAX_BINS):
                self._r[i] = (self._q1[i] * self._q1[i]) + (self._q2[i] * self._q2[i]) - (
                    self._coefs[i] * self._q1[i] * self._q2[i])
                self._q1[i] = 0
                self._q2[i] = 0
            self.post_testing()
            self._sample_count = 0

    def calc_coeffs(self):
        """
        Calculate the coefficients ahead of time

        :rtype: None
        :return: None
        """

        for n in range(self._MAX_BINS):
            self._coefs[n] = 2.0 * math.cos(2.0 * math.pi * self._freqs[n] / self._SAMPLING_RATE)

    def getDTMFfromWAV(self, filename):
        """
        This will take in a file name of a WAV file and return a str that contains the characters that were detected.
        So if your WAV file has the DTMFs for 5,5,5,3 then the str it returns will be "5553".

        :type filename: str
        :param filename: path of the WAV file
        :rtype: str
        :return: str that contains the characters that were detected
        """

        self.reset() #reset the current state of the detector

        file = wave.open(filename)

        totalFrames = file.getnframes()

        count = 0

        while totalFrames != count:
            raw = file.readframes(1)
            (sample,) = struct.unpack("h", raw)
            self.goertzel(sample)
            count = count + 1

        file.close()

        self.clean_up_processing()

        return self._keytone_list

