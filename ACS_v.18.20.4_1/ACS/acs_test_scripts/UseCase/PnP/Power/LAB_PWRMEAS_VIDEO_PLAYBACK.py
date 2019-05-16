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

import time
from UtilitiesFWK.Utilities import Global
from LAB_PWRMEAS_BASE import LabPwrMeasBase


class LabPwrMeasVideoPlayback(LabPwrMeasBase):

    """
    Class Lab Video Power Measurement CDK.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        LabPwrMeasBase.__init__(self, tc_name, global_config)

        # Get path to multimedia files
        self._multimedia_path = self._device.multimedia_path

        # Get TC Parameters
        self._video_file = self._tc_parameters.get_param_value("VIDEO_FILE")
        self._volume = int(self._tc_parameters.get_param_value("VOLUME"))

        self.__display = self._device.get_uecmd("Display")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        LabPwrMeasBase.set_up(self)

        self._phonesystem_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        self._system_api.adjust_specified_stream_volume("Media", self._volume)
        self.__display.set_display_orientation("landscape")
        self._video_api.play_native(self._multimedia_path + self._video_file)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """

        self._video_api.close_native()
        self.__display.set_display_orientation("auto")

        # Call tear_down of LabPwrMeasBase
        LabPwrMeasBase.tear_down(self)

        # Release the phone display
        self._phonesystem_api.display_off()

        return Global.SUCCESS, "No errors"
