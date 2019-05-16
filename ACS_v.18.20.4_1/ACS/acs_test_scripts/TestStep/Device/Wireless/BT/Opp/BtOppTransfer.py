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
:summary: This file implements a class to hold OPP transfer info
:since:03/02/2014
:author: fbongiax
"""
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants


class OppTransfer(object):
    """
    Holds information about OPP file transfer
    """

    def __init__(self):
        """
        Constructor
        """
        self.file_id = None
        self.address = None
        self.file_name = None
        self.file_size = None
        self.downloaded_size = None
        self.status = None
        self.timestamp = None
        self.current_time = None
        self.direction = None
        self.throughput = 0

        self._starting_time = 0
        self._delta_time = 0

    def update(self, src):
        """
        Update its fields from a dictionary (src)
        """
        self._do_update(src['id'], src['address'], src['filename'], long(src['filesize']), long(src['downloadedsize']), \
                        src['status'], src['timestamp'], long(src['currenttime']), src['direction'])

    def _do_update(self, file_id, address, filename, filesize, downloadedsize, status, timestamp, curtime, direction):
        """
        Update fields
        """
        self.file_id = file_id
        self.address = address
        self.file_name = filename
        self.file_size = filesize
        self.downloaded_size = downloadedsize
        self.status = status
        self.timestamp = timestamp
        self.current_time = curtime
        self.direction = direction
        self._calculate_throughput()

    def is_completed(self):
        """
        Return true if download completed
        """
        return self.status == Constants.OPP_STATE_DOWNLOADED

    def is_running(self):
        """
        Return true if download ongoing
        """
        return self.status == Constants.OPP_STATE_DOWNLOADING

    def is_cancelled(self):
        """
        Return true if download failed
        """
        return self.status == Constants.OPP_STATE_CANCELLED

    def is_throughput_above(self, value):
        """
        Returns true if file's throughputs are above the passed value
        """
        if self.throughput < value:
            return False
        return True

    def _calculate_throughput(self):
        """
        Calculate throughput
        """
        if self.status == Constants.OPP_STATE_WAITING:
            return

        if self._starting_time == 0:
            self._starting_time = self.current_time

        self._delta_time = self.current_time - self._starting_time
        meas = 0.0
        if self._delta_time != 0:
            dwnsize = long(self.downloaded_size)
            # add 1 digit precision: 123.4kbps => float(tp*10) /10
            meas = long(80000 * dwnsize / self._delta_time / 1024)
            meas = float(meas) / 10

        self.throughput = meas

    def __str__(self):
        """
        String form
        """
        perc = float(self.downloaded_size) / float(self.file_size) * 100.0
        return "File = %s, Status = %s, Size (Bytes) = %s, Progress = %.2f%%, Average throughput (KBit/sec)= %01.1f, "\
            "Current time = %d, Delta time = %d" % (self.file_name, self.status, self.downloaded_size, \
                                                    perc, self.throughput, self.current_time, self._delta_time)
