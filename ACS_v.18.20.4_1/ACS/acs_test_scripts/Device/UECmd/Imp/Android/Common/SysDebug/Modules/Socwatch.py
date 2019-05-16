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
:since: 16/02/2015
:author: sdubrayx
"""

import re
import os
import platform
import time
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.SysDebug.SysDebug import SysDebug


class Socwatch(SysDebug):

    """
    :summary: System UEcommands operations for Android platforms
    """

    def __init__(self, device, config=None):
        """
        Constructor.

        :type config: dictionary
        :param config: Configuration of the module. Unused for this module
        """
        SysDebug.__init__(self, device)
        self.__socwatch_command = ""
        self.__socwatch_path = self._device.get_config("SocwatchPath", "", str)
        self._use_aplog = False
        self._os = platform.system()
        self.__socwatch_config = {}
        self.__socwatch_folder = "/data/socwatch"
        self.__socwatch_zip = "socwatch_output.tgz"

        if config:
            for key in config:
                self.__socwatch_config[key] = config[key]
        else:
            self._logger.debug("Warning ! No socwatch command provided, module will be disabled")

    def _retrieve_sysdebug_message(self):
        """

        """
        sysinfo = []
        return sysinfo

    def load_socwatch_modules(self):
        """
        Load the socwatch kernel modules
        """
        # List of the potential modules to load (depending on the platform) sorted by dep order
        socwatch_modules = ["intel_visa", "socperf", "socwatch"]
        _ , output = self._device.run_cmd("adb shell ls /lib/modules", self._uecmd_default_timeout,
                                             force_execution=True, silent_mode=True)
        device_modules = output.splitlines()

        for sw_mod in socwatch_modules:
            for dev_mod in device_modules:
                if sw_mod in dev_mod:
                    self._device.run_cmd("adb shell insmod /lib/modules/%s" % dev_mod,
                                          self._uecmd_default_timeout, force_execution=True,
                                          silent_mode=False)

    def init(self):
        """
        Initialization
        """
        SysDebug.init(self)

        # if not defined
        if not self.__socwatch_path:
            return False

        # install socwatch and run
        install_script = ""
        if "Windows" in self._os:
            self._logger.debug("OS found : Windows")
            install_script = "socwatch_android_install.bat"
        elif "Linux" in self._os:
            self._logger.debug("OS found : Linux")
            install_script = "socwatch_android_install.sh"
        else:
            self._logger.debug("Unknown OS : %s, exiting", self._os)
            return False

        cmd = os.path.abspath("%s/%s" % (self.__socwatch_path, install_script))
        self._logger.debug("Running %s to install socwatch" % cmd)
        os.system(cmd)
        self.load_socwatch_modules()

        return True

    def reset(self, delay_s=0):
        """
        Start capture
        """
        socwatch_cmd = "socwatch"
        if delay_s:
            socwatch_cmd += " -s %d" % delay_s
        if "-t" not in self.__socwatch_config:
            socwatch_cmd += " -t 10" # 10 seconds by default

        for key in self.__socwatch_config:
            value = self.__socwatch_config[key]
            if value:
                socwatch_cmd += " %s %s" % (key, value)
            else:
                if socwatch_cmd.startswith("-"):
                    socwatch_cmd += " %s" % key
                else:
                    socwatch_cmd += " -f %s" % key
        self._logger.debug("socwatch command : %s" % socwatch_cmd)

        sw_script = "/data/socwatch/runSW.sh"
        cmd = "echo \"cd %s; . ./setup_socwatch_env.sh; %s\" > %s" % (self.__socwatch_folder, socwatch_cmd, sw_script)
        self._exec("adb shell %s" % cmd, force_execution=True,
                   wait_for_response=True, timeout=1)
        self._exec("adb shell chmod 777 %s" % sw_script, force_execution=True,
                   wait_for_response=True, timeout=1)
        self._exec("adb shell nohup sh %s" % sw_script, force_execution=True,
                   wait_for_response=False, timeout=1)
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
        xmltree = etree.Element("Socwatch")

        # Create a tgz with socwatch logs and generate a .tgz
        device_file = "%s/%s" % (self.__socwatch_folder, self.__socwatch_zip)
        cmd = "find %s -name '*.csv' -o -name '*.sw1'" % self.__socwatch_folder
        cmd += "| xargs tar -czf %s" % device_file
        self._device.run_cmd("adb shell %s" % cmd, self._uecmd_default_timeout,
                                             force_execution=True, silent_mode=True)
        cmd = "ls %s 1> /dev/null 2>&1 && echo Ok || echo NOk" % device_file
        testfiles = self._exec("adb shell %s" % cmd, 1, force_execution=True)
        if testfiles == "Ok":
            folder = "SocWatch"
            report_dir = self._device.get_report_tree()
            report_dir.create_subfolder(folder)
            report_base = report_dir.get_subfolder_path(folder)
            local_file = report_base + "/socwatch_capture_%s.tgz" % time.strftime("%Y_%m_%d__%H_%M_%S")
            self._device.pull(device_file, local_file, timeout=180)
            self._device.run_cmd("adb shell rm %s" % device_file, self._uecmd_default_timeout,
                                             force_execution=True, silent_mode=True)
            self._logger.debug("socwatch file pulled : %s" % local_file)
            capt = etree.Element("capture")
            capt.attrib["file"] = local_file
            xmltree.append(capt)

        return xmltree
