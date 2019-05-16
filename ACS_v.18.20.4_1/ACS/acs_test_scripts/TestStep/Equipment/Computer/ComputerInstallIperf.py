"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL QCTV
:summary: This file implements Test Step for install iperf on computer
:since 04/02/2015
:author: jfranchx
"""
import re
from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase


class ComputerInstallIperf(ComputerBase):
    """
    Implements install iperf
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)

        iperf_utility_path = self._pars.iperf_utility
        iperf_utility_bin = re.findall(r'(iperf[0-9].[0-9].[0-9]_COMPUTER)', iperf_utility_path)[0]
        iperf_utility_version = re.findall(r'iperf([0-9].[0-9].[0-9])_COMPUTER', iperf_utility_bin)[0]

        computer_connection = self._computer.init()
        iperf_version = self._computer.ssh_exec(self._computer.get_host_on_test_network(), "root", "iperf -v")
        iperf_version_std = re.findall(r'iperf version ([0-9].[0-9].[0-9])', iperf_version)

        if len(iperf_version_std) > 0:
            if iperf_utility_version == iperf_version_std[0]:
                self._logger.debug("Same version detected %s -, nothing to install" % iperf_utility_version)
                if computer_connection:
                    self._computer.release()
                return

        # Remove old iperf and configure the new version installed
        self._computer.copy_file_in_remote_path(iperf_utility_path, "/usr/bin/%s" % iperf_utility_bin)
        self._computer.ssh_exec(self._computer.get_host_on_test_network(), "root", "rm /usr/bin/iperf")
        self._computer.ssh_exec(self._computer.get_host_on_test_network(), "root",
                                "mv /usr/bin/%s /usr/bin/iperf" % iperf_utility_bin)
        self._computer.ssh_exec(self._computer.get_host_on_test_network(), "root", "chmod 755 /usr/bin/iperf")
        self._computer.ssh_exec(self._computer.get_host_on_test_network(), "root", "chgrp root /usr/bin/iperf")
        self._computer.ssh_exec(self._computer.get_host_on_test_network(), "root", "chown root /usr/bin/iperf")

        if computer_connection:
            self._computer.release()
