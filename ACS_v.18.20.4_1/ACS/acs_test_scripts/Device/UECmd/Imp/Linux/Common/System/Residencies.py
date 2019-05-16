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

:organization: INTEL NDG SW
:summary: This file implements the System UEcmd for Linux device
:author: floeselx
"""
import re
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IResidencies import IResidencies
from acs_test_scripts.Utilities.PnPUtilities import grouped_regex_from_list


class Residencies(Base, IResidencies):
    """
    Residencies reading/cleaning UECmd
    """

    def __init__(self, device):
        """
        Constructor
        """
        Base.__init__(self, device)
        IResidencies.__init__(self, device)

        self._mid_pmu_states_file = "/sys/kernel/debug/mid_pmu_states"
        self._sleep_modes = ["s0", "s0i1", "s0i2", "s0i3", "lpmp3", "s3"]
        self._residency_pattern = None
        self._residencies = None

    def _get_pattern(self):
        """
        Get the pattern to read residencies

        :rtype: list
        :return: list of pattern group to fetch residencies.
        """
        residency_group = []
        residency_group.append(["mode", "^", "[^\s]*", "\s"])

        self._logger.info("Get the power states pattern")

        cmd = "grep -C 1 -i -e '^%s' %s|head -n 1" % ("' -e '^".join(self._sleep_modes),
                                                    self._mid_pmu_states_file)

        status, residency_format = self._internal_exec(cmd)

        groups = residency_format.strip("\t").split("\t")

        for group in groups:
            groupname = group.split("(")[0].split("[")[0].strip().replace(".", "_").lower()
            if groupname != "" and groupname != "state":
                residency_group.append([groupname, "\s*", "[\d\.-]*", "\s"])

        residency_group[-1][3] = "$"

        return residency_group

    def _get_table(self, matchobj):
        """
        Get table of residency output

        :type matchobj: MatchObj
        :param matchobj: A matched line of residency output
        """
        if matchobj is None:
            return

        ret = {}
        for group in matchobj.groupdict().keys():
            residency = matchobj.group(group)
            ret[group] = residency.lower()

        return ret

    def _fetch(self):
        """
        Fetch residencies
        """
        if self._residencies is not None:
            return

        # In case of TestStep use we loose the previous Residencies instance, so we loose pattern retrieved during clear
        if self._residency_pattern is None:
            self._residency_pattern = grouped_regex_from_list(self._get_pattern())

        self._residencies = []
        cmd = "grep -i -e '^%s' %s" % ("' -e '^".join(self._sleep_modes),
                                       self._mid_pmu_states_file)

        status, residency_lines = self._internal_exec(cmd)

        residencies = residency_lines.splitlines()

        for residency in residencies:
            if residency:
                match = re.search(self._residency_pattern, residency)
                self._residencies.append(self._get_table(match))

    def clear(self):
        """
        Clear residencies
        """
        self._residencies = None
        self._residency_pattern = grouped_regex_from_list(self._get_pattern())

        self._logger.info("Reset pwr counters")

        cmd = "echo clear > %s" % self._mid_pmu_states_file
        status, output = self._internal_exec(cmd)

    def get_value(self, name, mode):
        """
        Return the residency column value named "name" for the given mode

        :type name: str
        :param name: Name of the requested residency column

        :type mode: str
        :param mode: Requested mode

        :rtype: float
        :return: The value of the column "name" for the given mode
        """
        self._fetch()
        for residency in self._residencies:
            if residency.get("mode") == str(mode).lower():
                return float(residency.get(name))

    def get(self):
        """
        Return the residencies table

        :rtype: dict
        :return: the residencies table
        """
        self._fetch()
        return self._residencies
