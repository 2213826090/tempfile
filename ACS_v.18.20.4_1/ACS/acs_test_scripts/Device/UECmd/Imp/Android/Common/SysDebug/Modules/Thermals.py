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
:summary: This file implements the SysDebug UEcmd for Android device
:since: 21/05/2013
:author: pbluniex
"""
import re
import os
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug
from Device.DeviceManager import DeviceManager


class Thermals(SysDebug):

    """
    Read thermals (zone and cpu) on the device
    """

    def __init__(self, device, config=None):
        """
        Constructor.

        :type device: Device
        :param device: The DUT

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)

        self._sys_debug_infos = {}
        self.__thermals = {}
        self.__thermal_cpu_path = "/sys/devices/platform/coretemp.0"
        self.__thermal_soc_file = "soc_temp_input"
        self.__thermal_core_file_temp = "temp[0-9]*_input"
        self.__thermal_core_file_label = "temp[0-9]*_label"
        self.__force_thermal_cooling = False
        self.__thermal_zones_path = "/sys/devices/virtual/thermal"

        self.__campaign_config = DeviceManager().get_global_config().campaignConfig

        self.__config = {}
        if config:
            self.__config = config

        if "waitForCoolDown" in self.__campaign_config:
            self.__force_thermal_cooling = self.__campaign_config.get("waitForCoolDown").lower() == "true"

    def __fill_thermal(self, zone, filename, value):
        """
        Fill sysdebug structure

        :type zone: str
        :param zone: thermal zone defined in the DUT

        :type filename: str
        :param filename: filename of thermal information

        :type value: str
        :param value: thermal information (file content)
        """
        if zone not in self.__thermals.keys():
            self.__thermals[zone] = {}

        if filename == self.__thermal_soc_file:
            self.__thermals[zone]['name'] = "soc"
            self.__thermals[zone]['value'] = value
        elif re.search(".*_label", filename) or filename == "type":
            self.__thermals[zone]["name"] = value
        elif re.search(".*_input", filename) or filename == "temp":
            self.__thermals[zone]["value"] = value

    def __get_thermals_data(self, lines):
        """
        Get dictionnary of thermals:

            zone :
                {
                    name : thermal_name,
                    value : thermal_value
                }

        :type lines: str
        :param lines: measure of a thermal.
                      The line looks like : "filepath/filename:value"
        """
        self.__thermals = {}
        for item in lines:
            data = item.split(":")
            if len(data) != 2:
                self._logger.warning("Thermal entry error : '%s'" % item)
                continue

            filename = os.path.basename(data[0])
            dirname = os.path.basename(os.path.dirname(data[0]))

            if dirname.find("coretemp") != -1:
                zone = ("coretemp", filename.split("_")[0])
            else:
                zone = ("thermal", dirname.split("_")[-1])

            self.__fill_thermal(zone, filename, data[1])

    def __merge_sys_debug_infos(self):
        """
        Add thermals in _sys_debug_infos structure.
        """
        for item in self.__thermals.values():
            if "value" not in item:
                continue

            name = item["name"]
            value = str(float(item["value"]) / 1000)

            if name in self._sys_debug_infos:
                value = self._sys_debug_infos[name] + ";" + value
                self._sys_debug_infos[name] = value
            else:
                self._sys_debug_infos[name] = value

    def _retrieve_sysinfo(self):
        """
        Retrieve informations
        """
        sysinfo = self._retrieve_sysdebug_message()
        self.__get_thermals_data(sysinfo)

    def _retrieve_sysdebug_message(self):
        """
        Retrieve triggered message in aplog

        :rtype: list
        :return: list of lines of thermal informations
        """
        cmd = "find %s -name type -o -name temp|xargs grep -e '^.*$';" % \
              self.__thermal_zones_path

        cmd += "find %s -name '%s' -o -name '%s' -o -name '%s'|xargs grep '^.*$';" % \
               (self.__thermal_cpu_path,
                self.__thermal_core_file_temp,
                self.__thermal_core_file_label,
                self.__thermal_soc_file)

        _ , thermals = self._device.run_cmd("adb shell " + cmd, self._uecmd_default_timeout,
                                       force_execution=True, silent_mode=True)
        return thermals.splitlines()

    def synchronize(self):
        """
        Synchronize

        :rtype: Boolean
        :return: True if module is synchronized with the DUT, False otherwise
        """
        self._retrieve_sysinfo()
        if self.__force_thermal_cooling is True:
            self._logger.debug("thermal_cooling is forced in the global config with waitForCoolDown=true")
            blocking = True
        elif "level" in self.__config.keys() and self.__config["level"].lower() == "blocking":
            blocking = True
        else:
            blocking = False

        for item2 in self.__config.keys():
            for item in self.__thermals.values():
                if "value" not in item:
                    continue
                name = item["name"]
                if str(item2).lower() in name.lower():
                    value = float(item["value"]) / 1000
                    if blocking and value > float(self.__config[item2]):
                        self._logger.warning("Wait requested as thermal %s has too high "
                                             "temperature : %s > %s" %
                                             (name, str(value), self.__config[item2]))
                        return False
                    elif value > float(self.__config[item2]):
                        self._logger.warning("Thermal %s has too high temperature : %s > %s" %
                                             (name, str(value), self.__config[item2]))
        return True

    def fetch(self):
        """
        Fetch sysdebug information on a connected device
        """
        self._retrieve_sysinfo()
        self.__merge_sys_debug_infos()

    def init(self):
        self._retrieve_sysinfo()
        self.__merge_sys_debug_infos()
        return True

    def stats(self, start, stop):
        """
        Return representation of lss changes

        :type start: Integer
        :param start: The timestamp of the beginning of the measure

        :type stop: Integer
        :param stop: The timestamp of the end of the measure

        :rtype: etree.Element
        :return: The Xml tree vue of statistic results of the module
        """
        SysDebug.stats(self, start, stop)

        xmltree = etree.Element("thermals")
        for name, value in self._sys_debug_infos.items():
            thermal = etree.Element("thermal")
            thermal.attrib["name"] = name
            thermal.attrib["value"] = value
            xmltree.append(thermal)

        self._logger.debug("Thermal stats : %s" % str(self._sys_debug_infos))
        return xmltree
