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
:summary: This file implements the Energy Management UEcmd for Android device
:since: 13/12/2011
:author: dgonzalez, vgombert
"""

import tempfile
import os
import time
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.EnergyManagement.EnergyManagement \
    import EnergyManagement as EnergyManagementCommon


class EnergyManagement(EnergyManagementCommon):

    """
    :summary: Localization UEcommands operations for Android platform.
    """

    def __init__(self, device):
        """
        Constructor
        """
        EnergyManagementCommon.__init__(self, device)
        self._fake_thermal_file = "/etc/MID_thermal_em.conf"
        self.__thermal_found = False

    def get_thermal_sensor_info(self, behavior=None,
                                behavior_value=None):
        """
        Gets the sensor info (value , threshold , state) for each sensor.

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act normalLy

        if behavior is equal to "scheduled":
        schedule the method to be launch after x seconds
                :type behavior_value: int
                :param behavior_value: time in second to wait
                                       before executing the method
                :rtype: str
                :return: pid of scheduled operation

        if behavior is equal to "read":
        read the output of a previous scheduled operation

                :type behavior_value: int
                :param behavior_value: pid

                :rtype: dict
                :return: a dictionary that contains thermal conf info : [THERMAL_CONF], [DTS], [BACKSKIN], [FRONTSKIN]

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: dict
                :return: a dictionary that contains thermal conf info : [THERMAL_CONF], [DTS], [BACKSKIN], [FRONTSKIN]

        the elements in returned dict are like below :
             - [register name][property] = (value, unit)

        i.e [DTS] =  (35 , DegreeCelsius)
        if there is no unit, unit value will be set to none.
        """
        # MID_thermal.conf is a link to an another thermal.conf
        # find the link:
        if behavior != "read" and not self.__thermal_found:
            result = ""
            thermal_path = ""
            tries = 0
            while result.lower().find("->") == -1:
                cmd = "adb shell ls -la " + self._acs_em_module.em_properties.thermal_file
                result = self._exec(cmd, timeout=10)
                if result.lower().find("->") != -1:
                    thermal_path = result.split("->")[1].strip()
                else:
                    tries += 1
                    if tries > 4:
                        raise DeviceException(DeviceException.OPERATION_FAILED,
                                              "MID_thermal.conf link not found in path: %s" % self._acs_em_module.em_properties.thermal_file)
            self._acs_em_module.em_properties.thermal_file = thermal_path
            self.__thermal_found = True
        # call the inherited function
        return EnergyManagementCommon.get_thermal_sensor_info(self, behavior, behavior_value)

    def build_fake_thermal_conf(self):
        """
        make a fake MID_thermal_em.conf to emule fake temperature
        """
        self._logger.info("[THERMAL] Make a a fake thermal conf of %s" % self._acs_em_module.em_properties.thermal_file)

        # Set /system partition to read/write
        self._device.set_filesystem_rw()
        phone_system = self._device.get_uecmd("PhoneSystem", True)

        # get the content from files
        # get value from the .orig if it exist
        if phone_system.check_file_exist_from_shell(self._acs_em_module.em_properties.thermal_orig_file):
            self._logger.info("[THERMAL] Secure copy .orig exist , get value from it")
            tmp_file = phone_system.read_file(self._acs_em_module.em_properties.thermal_orig_file)
        else:
            tmp_file = phone_system.read_file(self._acs_em_module.em_properties.thermal_file)
            self._logger.info("[THERMAL] Make a secure copy of %s" % self._acs_em_module.em_properties.thermal_file)
            phone_system.copy(self._acs_em_module.em_properties.thermal_file, self._acs_em_module.em_properties.thermal_orig_file)


        thermal_captor_list = ["DTS", "BACKSKIN", "FRONTSKIN"]

        for zone_id in thermal_captor_list:
            phone_system.write_file("/data/fake_temp_%s" % zone_id,
                                          "30000")
        output_file = ""
        zone = ""

        # Config file modifications
        tmp_file = tmp_file.splitlines()

        for line in tmp_file:

            if line.lower().find("name=") != -1:
                zone = line.split("=")[1].strip().upper()

            if zone in thermal_captor_list:
                if line.lower().find("path=") != -1:
                    output_file += line.split("=")[0] + "=/data/fake_temp_%s\n" % zone
                    continue
                elif line.lower().find("slope=") != -1:
                    output_file += line.split("=")[0] + "=1\n"
                    continue
                elif line.lower().find("intercept=") != -1:
                    output_file += line.split("=")[0] + "=0\n"
                    continue

            output_file += line + "\n"
        tmpd = tempfile.gettempdir()
        fd_out = open(os.path.join(tmpd, "MID_thermal.conf"), "w")
        fd_out.write(output_file)
        fd_out.close()
        time.sleep(10)
        # Put the modified file on the phone file system
        self._device.push(os.path.join(tmpd, "MID_thermal.conf"),
                          "/etc/MID_thermal_em.conf", 60)
        # make a link between the fake thermal conf and thermal_file
        # retry many time to make a link
        max_tries = 3
        tries = 0
        while tries < max_tries:
            # make the link with the fake thermal conf
            cmd = "adb shell rm %s" % self._acs_em_module.em_properties.thermal_file
            self._exec(cmd, timeout=5, wait_for_response=True)
            cmd = "adb shell ln -s %s %s" % (self._fake_thermal_file, self._acs_em_module.em_properties.thermal_file)
            self._exec(cmd, timeout=5, wait_for_response=True)
            time.sleep(5)
            # check the link
            cmd = "adb shell ls -l %s" % self._acs_em_module.em_properties.thermal_file
            output = self._exec(cmd, timeout=5, wait_for_response=True)
            if self._fake_thermal_file in output:
                break
            else:
                tries += 1
