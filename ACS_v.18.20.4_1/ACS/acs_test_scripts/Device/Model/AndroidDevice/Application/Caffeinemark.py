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
:summary: This script implements the angry birds application for power measurement
:since: 18/02/2013
:author: pbluniex
"""
import os
import re
import time
import zipfile
import subprocess
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException


class Caffeinemark(IAndroidPackage):

    """
    Implementation of caffeinemark as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "CAFFEINEMARK")
        self.is_lower_better = False
        self.__device = device
        self.__cpt_cap = 0
        self._phonesystem = device.get_uecmd("PhoneSystem")
        self._result = None
        self.__screenshot = None
        self.__res_file = None
        self._results = {"score": [], "sieve": [], "loop": [], "logic": [], "string": [], "float": [], "method": []}

    def __get_screenshot(self):
        """
        Capture the screenshot at end of the test
        """
        folder = "SCREEN/CaffeineMark"
        report_dir = self.__device.get_report_tree()
        report_dir.create_subfolder(folder)
        report_base = report_dir.get_subfolder_path(folder)
        filename = report_base + "/" + str(self.__cpt_cap) + "caffeine.png"
        self.__screenshot = self.__device.screenshot(filename=filename)
        self.__res_file = report_base + "/" + str(self.__cpt_cap) + "caffeine"
        self.__cpt_cap += 1

    def __fetch_result124(self):
        """
        Fetch the results for version 1.2.4 of the application
        """
        score_lines = self._result.splitlines()
        pattern = "^(?P<type>\w*) score = (?P<score>\d+)"
        for line in score_lines:
            match = re.search(pattern, line)
            if match:
                tscore = match.group("type")
                if tscore.lower() == "overall":
                    tscore = "score"
                self._results[tscore.lower()].append(float(match.group("score")))

    def __fetch_result31(self):
        """
        Return the score
        """
        self.__get_screenshot()
        cmd = ["tesseract", self.__screenshot, self.__res_file, "-psm", "1"]
        process = subprocess.Popen(cmd,
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT)
        process.wait()
        f = open(self.__res_file + ".txt")
        for line in f:
            splt = line.replace(":", "=").split("=")
            if len(splt) > 1:
                tscore = (splt[0].lower().split())[0]
                if(tscore == "overall"):
                    tscore = "score"
                if tscore in self._results.keys():
                    res = splt[1].replace("(", " ").split()
                    self._results[tscore].append(float(res[0].replace(" ", "")))
        f.close()
        #os.remove(self.__res_file + ".txt")
        #os.remove(self.__screenshot)

    def _fetch_result(self):
        """
        Fetch the result of the application
        """
        version = self._get_version()
        if "1.2.4" in version:
            self.__fetch_result124()
        else:
            self.__fetch_result31()

    def __wait13(self, timeout):
        """
        Wait end of run for version 13 of the application
        """
        end_message = "regex: Displayed com.android.cm3/.CaffeineMark"
        logger = self._get_device_logger()
        start_time = time.time()
        end_time = start_time + timeout
        end_cond = logger.is_message_received(end_message, timeout)
        if not end_cond:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout has been reached in Caffeine Mark")

    def __wait124(self, timeout):
        """
        wait end of application for version 1.2.4
        """
        return

    def wait(self, timeout):
        """
        Wait the end of the application
        """
        version = self._get_version()
        if "1.2.4" in version:
            self.__wait124(timeout)
        else:
            self.__wait13(timeout)

    def post_install(self):
        """
        Post-installation of the application
        """
        version = self._get_version()
        if "1.2.4" in version:
            cfm = zipfile.ZipFile(self._application_uri, 'r')
            cfm.extract("classes.dex")
            self.push_file("classes.dex", "/data/CFM.dex")

    def uninstall(self):
        """
        Uninstall the application
        """
        IAndroidPackage.uninstall(self)
        version = self._get_version()
        if "1.2.4" in version:
            self.adb_shell("rm /data/CFM.dex", 3)

    def start(self):
        self._logger.debug("Caffeinemark: start")

    def drive(self):
        """
        Start the application
        """
        self._logger.debug("Caffeinemark: drive")
        version = self._get_version()
        if "1.2.4" in version:
            cmd = "dalvikvm -cp /data/CFM.dex CaffeineMarkEmbeddedApp"
            self._result = self.adb_shell(cmd, 60)
        else:
            IAndroidPackage.start(self)

    def go_back_home(self):
        self._logger.debug("Caffeinemark: go_back_home")
        self.run_cmd("adb shell input keyevent 4", 5)
