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
:summary: Use Case Base for power measurement
:since: 23/09/2010
:author: igholm, vgombert
"""
from LAB_PWRMEAS_BASE import LabPwrMeasBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabPwrMeasAudioPlayback(LabPwrMeasBase):

    """
    Class Lab Audio Power Measurement CDK.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabPwrMeasBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        audio_file = self._tc_parameters.get_param_value("AUDIO_FILE")
        self._music_player = self._tc_parameters.get_param_value("MUSIC_PLAYER")

        if audio_file is not None:
            self._audio_file = self._device.multimedia_path + audio_file
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Audio file is not set in test case parameter")

        self._volume = int(self._tc_parameters.get_param_value("VOLUME"))

        self._sleep_mode = "lpmp3"
