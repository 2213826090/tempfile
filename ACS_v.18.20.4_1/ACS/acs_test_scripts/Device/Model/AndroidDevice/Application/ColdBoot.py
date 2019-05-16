"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This script implements the cold boot benchmark for performance measurement
@since: 16/04/2014
@author: plongepe
"""
import time
import re
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication
from ErrorHandling.DeviceException import DeviceException


class ColdBoot(IApplication):
    """
    Implementation of cold boot as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        @type device: Device
        @param device: The DUT
        """
        IApplication.__init__(self, device)

        self.is_lower_better = True
        self.__pattern = "\[\s*(?P<result>\d*\.\d*)\].*bootanim.exit"
        self.__pattern2 = ".*Boot animation finished.*"
        self.__subscore_pattern = ".*boot_progress_%s.*\s(?P<score>\d+)"
        self._results = {"score" : []}
        self.__subscores = {"start" : [], "preload_start" : [], "preload_end" : [], "system_run" : [], "pms_start" : [],
                            "pms_system_scan_start" : [], "pms_data_scan_start" : [], "pms_scan_end" : [],
                            "pms_ready" : [], "ams_ready" : [], "enable_screen" : []}

        self.__match = None
        self.start_cold_boot = 0
        self.end_cold_boot = 0

    def __fetch_boot_progress(self):
        """
        Fetch boot_progress results
        """
        for key in self.__subscores.keys():
            search = self.__subscore_pattern % key
            res = self._get_device_logger().get_message_triggered_status("regex:" + search)
            if not res:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't get score for %s" % key)

            matchobj = re.match(search, res[0])
            score = matchobj.group("score")
            if score is not None and score.isdigit():
                self.__subscores[key].append(float(matchobj.group("score"))/1000)
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can't fetch score for %s" % key)

            self._results["score"].append(float(matchobj.group("score"))/1000)
            self._get_device_logger().remove_trigger_message("regex:" + search)


    def _fetch_result(self):
        """
        Fetch the score of cold boot
        """
        if self.hard_reboot:
            self._logger.info("++ fetch result !")
            res = self._get_device_logger().get_message_triggered_status("regex:" + self.__pattern2)
            if not res:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Can't get score for BootTime")

            matchobj = re.search(self.__pattern2, res[0])
            #result = matchobj.group("result")
            self._logger.info("result = " + res[0])
            #self._logger.info("match = " + result)
            m = re.match("(?P<mounth>[0-9\.]+)-(?P<day>[0-9\.]+)\s(?P<hour>[0-9\.]+):(?P<min>[0-9\.]+):(?P<sec>[0-9]+)\.(?P<ms>[0-9]+).*", res[0])
            # TBD: How to get the year ?
            convert_time = time.strptime(m.group('day') + " " + m.group('mounth') + " 2014 " + 
                                         m.group('hour') + ":" + m.group('min') + ":" + m.group('sec'),
                                         "%d %m %Y %H:%M:%S")
            self._logger.info("convert_time = " + str(convert_time))
            self.end_cold_boot = time.mktime(convert_time)
            self.end_cold_boot = float(self.end_cold_boot) + float(m.group('ms')) / 1000.0        
            self._logger.info("start time = " + str(self.start_cold_boot))
            self._logger.info("end time = " + str(self.end_cold_boot))
            # TBD: How to do the GMT + 1 ?
            cold_boot_time = self.end_cold_boot - self.start_cold_boot + 2 * 3600
            self._logger.info("cold_boot time = " + str(cold_boot_time))
            #self._results["bootanimexit"].append(float(result))
            self._results["score"].append(cold_boot_time)
            self._get_device_logger().remove_trigger_message("regex:" + self.__pattern2)
            
        self.__fetch_boot_progress()
        for key, value in self.__subscores.items():
            self._results[key] = value


    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        if self.hard_reboot:
            if not self._get_device_logger().is_message_received("regex:" + self.__pattern2, timeout):
                raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                      "Timeout while executing Cold boot")
            
        self.end_cold_boot = time.time()

    def drive(self):
        """
        Drive the application to run the benchmark
        """
        self.adb_shell("date -s $(date +%Y%m%d.%H%M%S)", 3)
        if self.hard_reboot:
            self._get_device_logger().add_trigger_message("regex:" + self.__pattern2)
        
        for key in self.__subscores.keys():
            self._get_device_logger().add_trigger_message("regex:" + self.__subscore_pattern % key)

        self.start_cold_boot = self.reboot_device()


    def post_install(self):
        """
        Prepare the boot
        """
        #self._logger.info("++ set the lock screen !")
        # lock can now be changed
        #self._phonesystem.set_phone_screen_lock_on(0)
        # Lock the phone
        #self._phonesystem.enable_lockscreen()


    def uninstall(self):
        """
        Uninstall an application
        """
        #self._logger.info("++ unset the lock screen !")
        #self._phonesystem.disable_lockscreen()


