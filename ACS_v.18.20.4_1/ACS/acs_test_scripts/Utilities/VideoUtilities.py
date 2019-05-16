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

:organization: INTEL PEG SVE DSV
:summary: Utilities class for Video implementation
:since: 17 July 2014
:author: Jongyoon Choi
"""
import os
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT

class AwesomePlayerParser:
    """
    This class contains all the options related to resetting the phone
    """

    def __init__(self):
        """
        Constructor
        """
        # Instantiate the logger
        self._logger = LOGGER_TEST_SCRIPT
        self.AWESOME_PLAYER_FLAGS = {
            'PLAYING': 0x01,
            'LOOPING': 0x02,
            'FIRST_FRAME': 0x04,
            'PREPARING': 0x08,
            'PREPARED': 0x10,
            'AT_EOS': 0x20,
            'PREPARE_CANCELLED': 0x40,
            'CACHE_UNDERRUN': 0x80,
            'AUDIO_AT_EOS': 0x0100,
            'VIDEO_AT_EOS': 0x0200,
            'AUTO_LOOPING': 0x0400,
            'PREPARING_CONNECTED': 0x0800,
            'SEEK_PREVIEW': 0x1000,
            'AUDIO_RUNNING': 0x2000,
            'AUDIOPLAYER_STARTED': 0x4000,
            'INCOGNITO': 0x8000,
            'TEXT_RUNNING': 0x10000,
            'TEXTPLAYER_INITIALIZED': 0x20000,
            'SLOW_DECODER_HACK': 0x40000,
        }
        self.current_loop_totals = {}

        self._src_flags_file_path = ''
        self._dest_flags_file_path = ''

    def pull_status_file_to_host(self, device):
        if self._src_flags_file_path is not None and self._dest_flags_file_path is not None:
            cmd = 'adb pull {0} {1}'.format(self._src_flags_file_path, self._dest_flags_file_path)
            device.run_cmd(cmd, timeout=120)
            return True
        else:
            return False

    def parse_status_file(self):
        if os.path.exists(self._dest_flags_file_path):
            all_loop_stats = dict(self.AWESOME_PLAYER_FLAGS)
            for flag in all_loop_stats:
                all_loop_stats[flag] = [1.5, 0, 0, 0] # Min, Max, Avg, Sum (Needed to calculate rolling average)
            loop_count = 1
            self.current_loop_totals = {'Total Iterations': 0}
            flags_file = open(self._dest_flags_file_path)
            for line in flags_file:
                if line.lower().startswith('loop'):
                    if len(self.current_loop_totals)<=1: # We are on our first line
                        continue
                    self.update_loop_stats(self.current_loop_totals, all_loop_stats, loop_count)
                    self.current_loop_totals = {'Total Iterations': 0}
                    loop_count += 1
                    continue
                # Each line that doesn't start with Loop..., looks like this "   25 0x000020"
                line_split = line.split()
                self.current_loop_totals['Total Iterations'] += int(line_split[0])
                if len(line_split) == 1:
                    if 'NOT_OPENED' not in self.current_loop_totals:
                        self.current_loop_totals['NOT_OPENED'] = int(line_split[0])
                    else:
                        self.current_loop_totals['NOT_OPENED'] += int(line_split[0])
                    continue
                line_flag = int(line_split[1],16)
                for flag in self.AWESOME_PLAYER_FLAGS:
                    if line_flag & self.AWESOME_PLAYER_FLAGS[flag] == self.AWESOME_PLAYER_FLAGS[flag]:
                        if flag not in self.current_loop_totals:
                            self.current_loop_totals[flag] = int(line_split[0])
                        else:
                            self.current_loop_totals[flag] += int(line_split[0])
            self.update_loop_stats(self.current_loop_totals, all_loop_stats, loop_count)
            flags_file.close()

            return all_loop_stats
        else:
            return None

    def update_loop_stats(self, loop_totals, loop_stats, loop_count):
        for flag in self.AWESOME_PLAYER_FLAGS:
            if flag not in self.current_loop_totals:
                continue
            flag_percent = float(loop_totals[flag])/float(loop_totals['Total Iterations'])
            if flag_percent < loop_stats[flag][0]:
                loop_stats[flag][0] = flag_percent
            if flag_percent > loop_stats[flag][1]:
                loop_stats[flag][1] = flag_percent
            # Update Sum
            loop_stats[flag][3] += flag_percent
            # Update Average
            loop_stats[flag][2] = loop_stats[flag][3]/loop_count
