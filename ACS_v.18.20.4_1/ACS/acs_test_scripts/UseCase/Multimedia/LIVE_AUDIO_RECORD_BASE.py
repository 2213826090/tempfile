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
:summary: This file is the base Use Case for audio record
:since: 12/03/2012
:author: cchen59
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveAudioRecordBase(UseCaseBase):

    """
    Class Live Audio Record with mic source.
    """

    def __init__(self, tc_conf, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_conf, global_config)

        self._source = None
        self._audio_file = None

        # Get path to multimedia files
        self._multimedia_path = self._device.multimedia_path
        # Get TC Parameters
        self._codec = self._tc_parameters.get_param_value("CODEC")
        self._container = self._tc_parameters.get_param_value("CONTAINER")
        self._duration = int(self._tc_parameters.get_param_value("DURATION"))
        self._bitrate = self._tc_parameters.get_param_value("BITRATE")
        self._samplerate = self._tc_parameters.get_param_value("SAMPLERATE")
        self._bitrate = int(float(self._bitrate) * 1000)
        self._samplerate = int(float(self._samplerate) * 1000)
        self._channelnum = int(self._tc_parameters.get_param_value("CHANNELNUM"))
        self._play = ((self._tc_parameters.get_param_value("PLAY")).lower() == "true")
        self._deviation_rate = int(self._tc_parameters.get_param_value("DEVIATION_RATE"))
        self._volume = int(self._tc_parameters.get_param_value("VOLUME"))
        self._deviation = self._duration * self._deviation_rate / 100

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call UseCase base Setup function
        UseCaseBase.set_up(self)

        self._check_record_parameters(self._codec, self._container, self._bitrate,
                                      self._samplerate, self._channelnum)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def _check_record_parameters(self, codec, container, bitrate, samplerate, channelnum):  # pylint: disable=W0613
        """
        check if the record parameters are valid

        :type codec: str
        :param codec: recorded audio codec
        :type container: str
        :param container: recorded audio container
        :type bitrate: int
        :param bitrate: recorded audio bitrate, bps
        :type samplerate: int
        :param samplerate: recorded audio samplerate, hz
        :type channelnum: int
        :param channelnum: recorded audio channelnum

        :return: None
        """
        aac_samplerates_map = [8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000, 64000, 88200, 96000]
        # Keep following variable as we do not known if still needed
        # aac_boundary_freq_map = [0, 9391, 11502, 13856, 18783, 23004, 27713, 37566, 46009, 55426, 75132, 92017, sys.maxint]
        amrnb_bitrates_map = [4750, 5150, 5900, 6700, 7400, 7950, 10200, 12200]
        amrwb_bitrates_map = [6600, 8850, 12650, 14250, 15850, 18250, 19850, 23050, 23850]

        # check if input codec is supported.
        if codec == "amrnb":
            # check samplerate
            if samplerate != 8000:
                # return failed, raise exception
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "samplerate %d invalid: amr nb must be 8000" % samplerate)

            # check bit rate
            bitrate_index = -1
            for i in range(len(amrnb_bitrates_map)):
                if bitrate == amrnb_bitrates_map[i]:
                    bitrate_index = i
                    break

            if bitrate_index == -1:
                # return failed, raise exception
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "bitrate %d invalid" % bitrate)

            # check channelnum
            if channelnum != 1:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "channelnum %d invalid: amr nb must be 1" % channelnum)
        elif codec == "amrwb":
            # check samplerate
            if samplerate != 16000:
                # return failed, raise exception
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "samplerate %d invalid: amr wb must be 16000" % samplerate)

            # check bit rate
            bitrate_index = -1
            for i in range(len(amrwb_bitrates_map)):
                if bitrate == amrwb_bitrates_map[i]:
                    bitrate_index = i
                    break

            if bitrate_index == -1:
                # return failed, raise exception
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "bitrate %d invalid" % bitrate)

            # check channelnum
            if channelnum != 1:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "channelnum %d invalid: amr nb must be 1" % channelnum)
        elif codec in ["aaclc", "aac_eld", "he_aac"]:
            # check samplerate
            samplerate_index = -1
            for i in range(len(aac_samplerates_map)):
                if samplerate == aac_samplerates_map[i]:
                    samplerate_index = i
                    break

            if samplerate_index == -1:
                # return failed, raise exception
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "samplerate %d invalid: amr nb must be 8000" % samplerate)

            # check bit rate 6*get_sampling_rate()*get_channelId()
            if bitrate != 6 * samplerate * channelnum:
                self._logger.warning("bitrate %d invalid, will be reset to default value" % bitrate)

        else:
            # for others, we don't check
            pass

        self._logger.info("Check codec parameters completed...")
