#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
:summary: This module implements Sphinx Auto-generator of Documentation
:since: 17/12/13
:author: nbrissox
"""

from acs_test_scripts.Device.UECmd.Imp.Android.Common.Multimedia.Multimedia import (
    Multimedia as Common_Multimedia)


class Multimedia(Common_Multimedia):
    """
    Class that handle all audio operations
    """

    def __init__(self, device):
        """
        Constructor
        """
        Common_Multimedia.__init__(self, device)

        # Initialize common intents
        self.video_package = "com.google.android.gallery3d"
        self.audio_package = "com.google.android.music"
        self.video_component = self.video_package + "/com.android.gallery3d.app.MovieActivity"
        self.audio_component = self.audio_package + "/.AudioPreview"

    def take_screenrecord(self, screenrecord_name, save_directory="/sdcard/", verbose=False, rotate=False,
                              bit_rate=4000000, time_limit=180):
        """
        Take screen record on the DUT ans store it on sdcard

        :type screenrecord_name: str
        :param screenrecord_name: Name of screen record from sdcard.

        :type verbose: bool
        :param verbose: If True, display interesting information on stdout. By default False.

        :type rotate: bool
        :param rotate: If True, rotate the output 90 degrees.By default False.

        :type bit_rate: int
        :param bit_rate: Set the video bit rate, in megabits per second.  Default 4Mbps.

        :type time_limit: int
        :param time_limit: Set the maximum recording time, in seconds.  Default / maximum is 180.

        :rtype: None
        :return: None
        """
        self._logger.info("Take screen record and store it on sdcard.")

        #Create command with parameters
        cmd = "adb shell screenrecord "
        cmd += "--bit-rate %d " % bit_rate
        cmd += "--time-limit %d " % time_limit
        if verbose:
            cmd += "--verbose "
        if rotate:
            cmd += "--rotate "
        cmd += (save_directory + screenrecord_name)

        #Execute command with don't wait response from command.
        self._exec(cmd, wait_for_response=False)
