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
:summary: This file implements the SysDebug UECmd for Android device
:since: 17/02/2015
:author: sdubrayx
"""

from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class Interrupts(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
    """

    def __init__(self, device, _config=None):
        """
        Constructor.

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)

        self.__interrupts_file = "/proc/interrupts" # May change for some devices
        self._use_aplog = False
        self._trigger_group = [["name", "\s*", "[0-9A-Za-z]*", ":"],
                               ["numbers", "\s*", "([0-9]*\s*)*", "\s*"],
                               ["information", "\s*", "[A-Za-z].*", "$"]]
        self._infos_before = []
        self._print_debug = False

    def _retrieve_sysdebug_message(self):
        """
        Get the informations from the device
        """
        sysinfo = []

        cmd = "cat %s" % self.__interrupts_file
        _ , output = self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                                          force_execution=True, silent_mode=True)
        sysinfo = output.splitlines()

        return sysinfo

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        cmd = "ls %s 1> /dev/null 2>&1 && echo Ok || echo NOk" % self.__interrupts_file
        testfiles = self._exec("adb shell %s" % cmd, 1, force_execution=True)
        if testfiles == "NOk":
            return False
        return True

    def reset(self, delay_s=0):
        """
        Get interrupts information before
        """
        self._retrieve_sysinfo()
        self._infos_before = self._sys_debug_infos[:]
        self._sys_debug_infos = []
        return True

    def stats(self, start, stop):
        """
        Return stats

        :type start: Integer
        :param start: not used

        :type stop: Integer
        :param stop: not used

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)

        xmltree = etree.Element("Interrupts")
        irqnum = {}

        for item in self._infos_before:
            irqnum[item["name"]] = 0
            for i in item["numbers"].split():
                irqnum[item["name"]] -= int(i)

        for item in self._sys_debug_infos:
            irq = etree.Element("interrupt")
            irq.attrib["irq"] = item["name"]
            if item["name"] not in irqnum:
                 irqnum[item["name"]] = 0
            for i in item["numbers"].split():
                irqnum[item["name"]] += int(i)
            irq.attrib["number"] = str(irqnum[item["name"]])
            irq.attrib["info"] = ' '.join(item["information"].split())

            if irqnum[item["name"]]:
                xmltree.append(irq)

        return xmltree
