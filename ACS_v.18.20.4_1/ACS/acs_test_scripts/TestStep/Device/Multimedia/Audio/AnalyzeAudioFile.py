"""
@summary: Reset Google Play Music app to a defined state for a specific
audio file.
@since 2 Feb 2016
@author: olilja
@organization: INTEL OTC/ACE

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

import sys
from ctypes import *
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global

class AnalyzeAudioFile(DeviceTestStepBase):

    check_freq_f = 440
    path_analyze = '/lib/acs/'

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self.fname_ref = self._pars.fname_ref
        self.fname_rec = self._pars.fname_rec
        self.mic_distance = int(self._pars.mic_distance)
        self.android_gain = int(self._pars.android_gain)
        self.check_gain_tolerance = int(self._pars.check_gain_tolerance)
        self.check_freq_frequency_tolerance = int(self._pars.check_freq_frequency_tolerance)
        self.check_freq_power_tolerance = int(self._pars.check_freq_power_tolerance)
        self.check_dist_tolerance = int(self._pars.check_dist_tolerance)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        self._logger.info("AnalyzeAudioFile - Parameters ----------------------------")
        self._logger.info("AnalyzeAudioFile - FNAME_REF = " + self.fname_ref)
        self._logger.info("AnalyzeAudioFile - FNAME_REC = " + self.fname_rec)
        self._logger.info("AnalyzeAudioFile - MIC_DISTANCE = " + str(self.mic_distance))
        self._logger.info("AnalyzeAudioFile - ANDROID_GAIN = " + str(self.android_gain))
        self._logger.info("AnalyzeAudioFile - CHECK_GAIN_TOLERANCE = " + str(self.check_gain_tolerance))
        self._logger.info("AnalyzeAudioFile - CHECK_FREQ_FREQUENCY_TOLERANCE = " + str(self.check_freq_frequency_tolerance))
        self._logger.info("AnalyzeAudioFile - CHECK_FREQ_POWER_TOLERANCE = " + str(self.check_freq_power_tolerance))
        self._logger.info("AnalyzeAudioFile - CHECK_DIST_TOLERANCE = " + str(self.check_dist_tolerance))
        self._logger.info("AnalyzeAudioFile - ---------------------------- Parameters")

        self._logger.info("AnalyzeAudioFile - Perform analysis\n")

        cdll.LoadLibrary(self.path_analyze + "libanalyze.so")
        analyze_audio = CDLL(self.path_analyze + "libanalyze.so")

        result = Global.SUCCESS

        ret = analyze_audio.check_gain(self.fname_rec, self.mic_distance, self.android_gain, self.check_gain_tolerance)
	if ret == 0:
            self._logger.info("AnalyzeAudioFile - Check gain - PASS\n")
        else:
            self._logger.error("AnalyzeAudioFile - Check gain - FAILURE\n")
            result = Global.FAILURE

        ret = analyze_audio.check_frequency(self.fname_rec, self.check_freq_f, self.check_freq_frequency_tolerance, self.android_gain, self.check_freq_power_tolerance)
	if ret == 0:
            self._logger.info("AnalyzeAudioFile - Check freq - PASS\n")
        else:
            self._logger.error("AnalyzeAudioFile - Check freq - FAILURE\n")
            result = Global.FAILURE

        ret = analyze_audio.check_distortion(self.fname_rec, self.fname_ref, self.check_dist_tolerance)
	if ret == 0:
            self._logger.info("AnalyzeAudioFile - Check dist - PASS\n")
        else:
            self._logger.error("AnalyzeAudioFile - Check dist - FAILURE\n")
            result = Global.FAILURE

        if result == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "AnalyzeAudioFile - Analyzis failed!")

        return result
