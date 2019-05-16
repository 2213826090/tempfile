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
:since: 21/01/2015
:author: sdubrayx
"""

import re
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class Pytimechart(SysDebug):

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

        self.__pytimechart_cmd = "pytimechart-record"
        self._use_aplog = False

    def _retrieve_sysdebug_message(self):
        """
        Get the dstates informations from the device
        """
        sysinfo = []
        output = self._exec("adb shell %s stop" % self.__pytimechart_cmd, force_execution=True, wait_for_response=True)
        sysinfo = output.splitlines()

        return sysinfo

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        cmd = "which %s 1> /dev/null 2>&1 && echo Ok || echo NOk" % self.__pytimechart_cmd
        testfiles = self._exec("adb shell %s" % cmd, 1, force_execution=True)
        if testfiles == "NOk":
            return False
        return True

    def reset(self, delay_s=0):
        """
        Start capture
        """
        output = self._exec("adb shell %s start" % self.__pytimechart_cmd, force_execution=True, wait_for_response=True)
        return True

    def _retrieve_sysinfo(self):
        """
        Redefined because we do not want to use regexps
        """
        sysinfo = self._retrieve_sysdebug_message()
        for item in sysinfo:
            self._sys_debug_infos.append(item)
        return sysinfo

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
        capt_file = ""
        xmltree = etree.Element("Pytimechart")
        for item in self._sys_debug_infos:
            if "adb pull" in item:
                match = re.search("(?P<file>/[^ ]*)", item)
                if match:
                    cap = etree.Element("capture")
                    capt_file = match.group("file")
                    cap.attrib["file"] = capt_file
                    xmltree.append(cap)

                    folder = "Pytimechart"
                    report_dir = self._device.get_report_tree()
                    report_dir.create_subfolder(folder)
                    report_base = report_dir.get_subfolder_path(folder)
                    local_filename = report_base + "/" + capt_file
                    self._exec("adb shell sync", force_execution=True, wait_for_response=True)
                    self._device.pull(capt_file, local_filename, timeout=180)

        self._logger.debug("pytimechart_capture pulled: %s" % capt_file)
        return xmltree
