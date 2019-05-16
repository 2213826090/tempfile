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
:since: 12 sep 2012
:author: sfusilie
"""
from random import randint
import re
import tempfile
import time
import os
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.EMUtilities import EMConstant as CST
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import AcsDict
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.Imp.Android.ICS.EnergyManagement.EnergyManagement import EnergyManagement as EnergyManagementICS
from lxml import etree


class EnergyManagement(EnergyManagementICS):

    """
    :summary: Localization UEcommands operations for Android platform.
    """

    def __init__(self, device):
        """
        Constructor
        """
        EnergyManagementICS.__init__(self, device)
        self.__task_list = {}
        self.__thermal_secure_copy_done = False
        self.__thermal_file_instance = None
        self.__thermal_change_done = False
        self.__cst_zone = "Zone"
        self.__cst_zone_name = "ZoneName"
        self.__cst_profile = "Profile"
        self.__cst_zone_threshold = {"off": ["ThresholdTOff"],
                          "normal": ["ThresholdNormal", "ZoneThresholdNormal", "ZoneThresholdNormal1", "ZoneThresholdNormal2", "ZoneThresholdNormal3"],
                          "warning": ["ThresholdWarning", "ZoneThresholdWarning", "ZoneThresholdWarning1", "ZoneThresholdWarning2", "ZoneThresholdWarning3"],
                          "alert": ["ThresholdAlert", "ZoneThresholdAlert", "ZoneThresholdAlert1", "ZoneThresholdAlert2", "ZoneThresholdAlert3"],
                          "critical": ["ThresholdCritical"]}

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
        # overwrite the implementation from ICS as thermal path is static here.
        if behavior == "scheduled":
            self._logger.info("[THERMAL] schedule get sensor information")
            output = self._internal_exec_v2(self._thermal_module, "scheduledGetThermalInfo",
                                            "--ei delay %s --es path %s" % (behavior_value, self._acs_em_module.em_properties.thermal_file), is_system=True)
            self._logger.debug("[THERMAL] Task %s will start in %s seconds" %
                               (output[self.OUTPUT_MARKER], behavior_value))
            return output[self.OUTPUT_MARKER]

        elif behavior == "read":
            self._logger.info("[THERMAL] read get thermal sensor info result from task %s" % behavior_value)
            output = self.get_async_result(str(behavior_value))

        else:
            self._logger.info("[THERMAL] get sensor information")
            output = self._internal_exec_v2(self._thermal_module, "getThermalInfo",
                                            "--es path %s" % self._acs_em_module.em_properties.thermal_file, is_system=True)

        for keys in output.keys():
            output[keys] = self._format_text_ouput(output[keys])

        if not (isinstance(output, AcsDict) or isinstance(output, dict)) or not self.OUTPUT_MARKER in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found : %s" % (self.OUTPUT_MARKER, str(output)))

        return self.__parse_thermal_sensor_response(output[self.OUTPUT_MARKER])

    def __parse_thermal_sensor_response(self, output, log_output=True):
        """
        Parses the response gotten from the scheduled get thermal sensor info
        in order to extract the following parameters:
        The function as many dict as sensor available like "CPU", "BACKSKIN, "FRONTSKIN"
        Each object in the returned dictionary is a dictionary build from the "tag = value" parsed line format

        :type output: str
        :param output: output result from thermal conf embd uecmd

        :type log_output: boolean
        :param log_output: if True, will log a pretty output

        :rtype: dict
        :return:dictionary of the thermal conf info
        """
        thermal_conf = {}
        cosmetic_text = ""

        def __cosmetic_constructor(name, dico_object):
            """
            internal function
            """
            unit = ""
            value = ""
            if type(dico_object) in [list, tuple]:
                value = str(dico_object[0])
                if len(dico_object) > 1 and dico_object[1] not in ["none", ""]:
                    unit = str(dico_object[1])
            else:
                value = str(dico_object)
            return ("\t" + name + " = " + value + " " + unit).rstrip() + "\n"

        # get time stamp
        if output.find("<TIME_STAMP>") != -1 and output.find("</TIME_STAMP>") != -1:
            stamp_text = output[output.find("<TIME_STAMP>"):output.find("</TIME_STAMP>") + len("</TIME_STAMP>")]
            time_stamp = output[output.find("<TIME_STAMP>") + len("<TIME_STAMP>"):
                                output.find("</TIME_STAMP>")].strip()
            thermal_conf.update({"TIME_STAMP": (time_stamp, "none")})
            output = output.replace(stamp_text, "")
        else:
            thermal_conf.update({"TIME_STAMP": ("Failed to get", "none")})

        if log_output:
            cosmetic_text = "[GENERAL info]:\n"
            cosmetic_text += "\tTIME_STAMP = " + thermal_conf["TIME_STAMP"][0] + "\n"

        # get sensor info
        splited_output = re.split("</[^>]+>", output)
        for element in splited_output:
            sensor_info = element.splitlines()
            zone_name = None
            sensor = {}
            if sensor_info > 1:
                for info in sensor_info:
                    if info.find("<") != -1 and info.find(">") != -1 and info.find("=") == -1:
                        zone_name = info[info.find("<") + 1: info.find(">")].strip()
                    elif info.find("=") != -1:
                        splited_info = info.split("=")
                        element_tag = splited_info[0].strip().upper()
                        element_value = splited_info[1].strip()

                        if element_tag in [CST.THRESHOLD_TM_OFF, CST.THRESHOLD_NORMAL, CST.THRESHOLD_WARNING,
                                           CST.THRESHOLD_ALERT, CST.THRESHOLD_CRITICAL, CST.VALUE]:
                            sensor.update({element_tag: (float(element_value) / 1000, CST.DEGREE_CELSIUS)})
                        elif element_tag in [CST.STATE, CST.SENSOR_NAME]:
                            sensor.update({element_tag: (element_value, "none")})

                if zone_name is not None:
                    if zone_name.upper().find(CST.DTS) != -1 or \
                    (sensor.get(CST.SENSOR_NAME) is not None and sensor[CST.SENSOR_NAME][0].upper().find(CST.DTS) != -1):
                        zone_name = CST.DTS
                    thermal_conf.update({zone_name: sensor})
                    # prepare logs
                    if log_output:
                        cosmetic_text += "[%s info]:\n" % zone_name
                        for sensor_key in sensor.keys():
                            cosmetic_text += __cosmetic_constructor(sensor_key, sensor[sensor_key])

        self._logger.debug(cosmetic_text)
        return thermal_conf

    def build_fake_thermal_conf(self):
        """
        change thermal file to emulate fake temperatures.

        :return: None
        """
        self._logger.info("[THERMAL] Make a a fake thermal conf of %s" % self._acs_em_module.em_properties.thermal_file)
        phone_system = self._device.get_uecmd("PhoneSystem", True)
        # Set /system partition to read/write
        self._device.set_filesystem_rw()

        # get the content from files
        # get value from the .orig if it exist

        if phone_system.check_file_exist_from_shell(self._acs_em_module.em_properties.thermal_orig_file):
            self._logger.info("[THERMAL] Secure thermal file copy .orig exist , get value from it")
            tmp_file = phone_system.read_file(self._acs_em_module.em_properties.thermal_orig_file)
        else:
            tmp_file = phone_system.read_file(self._acs_em_module.em_properties.thermal_file)
            self._logger.info("[THERMAL] Make a secure copy of %s" % self._acs_em_module.em_properties.thermal_file)
            phone_system.copy(self._acs_em_module.em_properties.thermal_file,
                              self._acs_em_module.em_properties.thermal_orig_file)

        # this dictionnary is necessary in order to align test fake_temp_NAME, and sensor name
        sensor_list = {CST.DTS: CST.DTS,
                       "SKIN1": CST.BACKSKIN,
                       "SKIN0": CST.FRONTSKIN,
                       "SKIN1_BIS": CST.GPUSKIN,
                       CST.BATTERY: CST.BATTERY}
        zone_list = [CST.CPU,
                     CST.BACKSKIN,
                     CST.FRONTSKIN,
                     CST.GPUSKIN,
                     CST.BATTERY]

        for key in sensor_list:
            phone_system.write_file("/logs/fake_temp_%s" % (sensor_list[key]),
                                          "30000")
        output_file = ""
        zone = ""
        sensor = ""
        in_zone = False
        in_sensor = False
        # Config file modifications
        tmp_file = tmp_file.splitlines()

        for line in tmp_file:

            if line.lower().find("<zone>") != -1:
                in_zone = True
            if line.lower().find("<sensor>") != -1:
                in_sensor = True

            # sensor found
            if in_zone:
                if line.lower().find("<zonename>") != -1:
                    zone = line.upper().replace("<ZONENAME>", "").replace("</ZONENAME>", "").strip()

                if zone in zone_list:
                    if in_sensor:
                        if line.lower().find("<sensorname>") != -1:
                            sensor = line.upper().replace("<SENSORNAME>", "").replace("</SENSORNAME>", "").strip()
                            if not (sensor in sensor_list):
                                self._logger.warning("the sensor %s in the zone %s is not in the thermal config file" % (sensor, zone))
                                in_sensor = False
                                sensor = ""
                            else:
                                self._logger.warning("the sensor %s in the zone %s is in the thermal config file" % (sensor, zone))

                        index = line.lower().find("sensorpath")
                        if index != -1:
                            output_file += line[0: index - 1] + \
                                "<SensorPath>/logs/</SensorPath>\n"
                            continue

                        index = line.lower().find("inputtemp")
                        if index != -1:
                            if zone == CST.GPUSKIN:
                                output_file += line[0: index - 1] + \
                                    "<InputTemp>fake_temp_%s</InputTemp>\n" % (sensor_list["SKIN1_BIS"])
                            else:
                                output_file += line[0: index - 1] + \
                                    "<InputTemp>fake_temp_%s</InputTemp>\n" % (sensor_list[sensor])
                            continue

            if line.lower().find("</zone>") != -1:
                in_zone = False
                zone = ""
            if line.lower().find("</sensor>") != -1:
                in_sensor = False
                sensor = ""

            output_file += line.replace("\n", "") + "\n"

        tmpd = tempfile.gettempdir()
        file_path = os.path.join(tmpd, "thermal_sensor_config.xml")
        fd_out = open(file_path, "w")
        fd_out.write(output_file)
        fd_out.close()
        time.sleep(10)
        # Put the modified file on the phone file system
        self._device.push(file_path, self._acs_em_module.em_properties.thermal_file, 60)

    def get_msic_registers(self, behavior=None,
                           behavior_value=None):
        """
        Gets the msic registers (enhanced for JB).
        parsing from shell or apk depending of your boot state

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act like normaly

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
                :return: a dictionary that contains 3 registers : [CHARGER], [BATTERY], [INTERRUPT]

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: dict
                :return: a dictionary that contains 3 registers : [CHARGER], [BATTERY], [INTERRUPT]

        the elements of returned dictionary are like below :
             - (value, unit) = [register name][property]

        eg: (10V, unit) = [CHARGER][VOLTAGE]
        if there is no unit , unit value will be set to none.
        """
        if behavior == "scheduled":
            self._logger.info("schedule get uevent info")
            # avoid using the one in device to earn time
            boot_mode = self._device.get_boot_mode()

            if boot_mode == "MOS":  # if mos call apk code
                task_id = self._internal_exec_v2(self._em_module, "scheduledGetUeventInfo",
                                                 "--ei delay %s" % behavior_value, is_system=True)[self.OUTPUT_MARKER]
                self.__task_list[task_id] = "APK"
            elif boot_mode in["ROS", "COS"]:  # else use shell script
                task_id = self.__get_shell_msic_registers(behavior, behavior_value)
                self.__task_list[task_id] = "SHELL"
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "cant execute get_msic_registers: board boot mode is %s" % boot_mode)

            self._logger.debug("Task %s will start in %s seconds" %
                               (task_id, behavior_value))
            return task_id

        elif behavior == "read":
            self._logger.info("read get uevent info result")
            # try to detect if it is a shell script to adjust the reader
            if behavior_value in self.__task_list:
                task_type = self.__task_list.pop(behavior_value)
                if task_type == "SHELL":
                    return self.__get_shell_msic_registers(behavior, behavior_value)
                elif task_type == "APK":

                    # else get reply from apk
                    output = self.get_async_result(str(behavior_value))
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "cant execute get_msic_registers: Unknown task id %s" % behavior_value)

        else:
            self._logger.info("get uevent info")
            # check boot mode
            boot_mode = self._device.get_boot_mode()
            if boot_mode == "MOS":  # if mos call apk code
                output = self._internal_exec_v2(self._em_module, "getUeventInfo", is_system=True)
            elif boot_mode in["ROS", "COS"]:  # else use shell script
                return self.__get_shell_msic_registers(behavior, behavior_value)
            else:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "cant execute get_msic_registers: board boot mode is %s" % boot_mode)

        # only done if is an apk reading
        for keys in output.keys():
            output[keys] = self._format_text_ouput(output[keys])

        if (not isinstance(output, AcsDict) and not isinstance(output, dict)) or \
                self.OUTPUT_MARKER not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found : %s" % (self.OUTPUT_MARKER, str(output)))

        return self._parse_msic_response(output[self.OUTPUT_MARKER])

    def __get_shell_msic_registers(self, behavior=None,
                                   behavior_value=None):
        """
        Gets the msic registers from shell, to be used for test in COS & ROS.

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act like normaly

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
                :return: a dictionary that contains 3 registers : [CHARGER], [BATTERY], [INTERRUPT]

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: dict
                :return: a dictionary that contains 3 registers : [CHARGER], [BATTERY], [INTERRUPT]

        the elements of returned dictionary are like below :
             - (value, unit) = [register name][property]

        eg: (10V, unit) = [CHARGER][VOLTAGE]
        if there is no unit , unit value will be set to none.
        """
        cmd = "adb shell "
        cmd_core = 'echo \<EM\>\<TIME_STAMP\> > [TASK] && ' + \
                'date +%Y-%m-%d_%Hh%M.%S >> [TASK] && ' + \
                'echo \<\/TIME_STAMP\> >> [TASK] && ' + \
                'echo \<BATTERY\> >> [TASK] && ' + \
                'cat /sys/class/power_supply/*battery/uevent >> [TASK] && ' + \
                'echo \<\/BATTERY\>  >> [TASK] && ' + \
                'echo \<CHARGER\> >> [TASK] && ' + \
                'cat /sys/class/power_supply/*charger/uevent >> [TASK] && ' + \
                'echo \<\/CHARGER\> >> [TASK] && ' + \
                'echo \<CHARGER_EXTRA\> >> [TASK] && ' + \
                'cat /sys/bus/pci/devices/0000:00:02.3/chargers | grep \"Max Charging Current:\" >> [TASK] ; ' + \
                'echo \<\/CHARGER_EXTRA\> >> [TASK] && ' + \
                'echo \<\/EM\> >> [TASK]'

        if behavior == "scheduled":
            task_id = "TASK_" + str(randint(1000, 9999))
            task = self._msic_shell_folder + task_id
            script_name = self._msic_shell_folder + "SCRIPT_" + task_id + ".sh"

            cmd_core_scheduled = cmd_core.replace("[TASK]", task)

            cmd_core = "#!sh\\nsleep " + str(behavior_value) + " && " + cmd_core_scheduled

            cmd = "adb shell echo \'%s\' > %s" % (cmd_core, script_name)
            self._exec(cmd, force_execution=True)

            # Set execution permissions
            cmd = "adb shell chmod 777 %s" % script_name
            self._exec(cmd, force_execution=True)

            # in ROS, nohup is not availabile
            # check if we need a parry
            ros_parry = ""
            if self._device.get_boot_mode() == "ROS":
                output = self._exec("adb shell ls -l /system_mos",
                                    force_execution=True, wait_for_response=True)
                if "No such file or directory" in output:
                    # mount in ROS filesystem the /system of main Os with nohup inside
                    self._exec("adb shell mkdir system_mos", force_execution=True,
                               wait_for_response=False)
                    self._exec("adb shell mount -t ext4 /dev/block/mmcblk0p8 /system_mos",
                               force_execution=True, wait_for_response=False)
                # add the ros_parry to execute nohup
                ros_parry = "/system_mos/xbin/busybox "

            # Run script detached on DUT
            cmd = "adb shell %snohup sh %s" % (ros_parry, script_name)

            self._exec(cmd, force_execution=True, wait_for_response=False)
            self._logger.debug("task result will be store in %s" % task)
            return task_id

        elif behavior == "read":
            task = self._msic_shell_folder + str(behavior_value)
            cmd += "cat " + task
            output = self._exec(cmd, force_execution=True)
            # clean script and generated task
            self._exec("adb shell rm -f " + task, force_execution=True)
            task = self._msic_shell_folder + "SCRIPT_" + str(behavior_value)
            self._exec("adb shell rm -f " + task, force_execution=True)

        else:
            task = self._msic_shell_folder + "NO_SCHEDULED_TASK"
            cmd += cmd_core
            cmd = cmd.replace("[TASK]", task)
            self._exec(cmd, force_execution=True)
            output = self._exec("adb shell cat %s" % task, force_execution=True)
            # clean task
            self._exec("adb shell rm -f " + task, force_execution=True)

        return self._parse_msic_response_from_shell(output)

    def thermal_test_set_temp(self, zone_id, temp):
        """
        Set the fake temperature to a specific zone

        :type  zone_id: int
        :param zone_id: number of the zone to set. Should be between 0 and 2
        :type  temp: int
        :param temp: Temperature value to set in milli degre Celcius
        """
        phone_system = self._device.get_uecmd("PhoneSystem", True)
        phone_system.write_file("/logs/fake_temp_%s" % zone_id,
                                      int(temp), force_execution=True)

    def restore_thermal_captor_temp(self, thermal_captor, temperature, timeout):
        """
        restore the thermal captor temperature

        :type  thermal_captor: str
        :param thermal_captor: captor file path

        :type  temperature: int
        :param temperature: temperature to restore.

        :rtype: Boolean
        :return: return true if the captor temp was succefully restored
        """
        start_time = time.time()
        self._logger.info("[THERMAL] try to restore thermal captor in"
                          + " captor:%s" % thermal_captor)
        while(time.time() - start_time) < timeout:
            # write temp in captor
            cmd_write = "adb shell echo -e '%s' > /logs/fake_temp_%s"\
                % (temperature, thermal_captor)
            result = self._device.run_cmd(cmd_write, 3, force_execution=True)
            # check if uecmd was received by the phone
            if result[0] == Global.SUCCESS:
                self._logger.info("[THERMAL] the board is on,"
                                  + " check change on thermal captor")
                # check if the change was done
                cmd_read = "adb shell cat /logs/fake_temp_%s" % thermal_captor
                result = self._device.run_cmd(cmd_read, 3, force_execution=True)
                if str(temperature) in result[1]:
                    return True
                else:
                    self._logger.error("[THERMAL] fail to restore thermal"
                                       + " captor temp on %s" % thermal_captor)
                    return False

    def set_thermal_management(self, mode):
        """
        set or deactivate the thermal management
        a reboot is requiered to take this change into account

        :type mode: boolean
        :param mode: True to activate thermal management , False to stop it
        """
        if mode:
            cmd = 1
        else:
            cmd = 0

        cmd = "adb shell setprop persist.service.thermal %s" % cmd
        output = self._exec(cmd)

        if not self.is_shell_output_ok(output):
            txt = "fail to change thermal management: %s" % output
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

    def load_thermal_file(self, load_from_orig=False):
        """
        Load the contents of thermal file from the DUT.
        This will allow you to modify the file several times without interacting with the DUT.

        @type load_from_orig:boolean
        @param load_from_orig: try to load thermal file from the original file.
                               the original file is the .orig generated or by default the normal file.

        """
        # create a fake file with all the data
        tmpd = tempfile.gettempdir()
        tmp_file = os.path.join(tmpd, "thermal_loaded_file.xml")
        file_to_pull = self._acs_em_module.em_properties.thermal_file
        if load_from_orig:
            file_api = self._device.get_uecmd("File", True)
            if file_api.exist(self._acs_em_module.em_properties.thermal_orig_file)[0]:
                self._logger.info("thermal file will be loaded from original file:%s" % self._acs_em_module.em_properties.thermal_orig_file)
                file_to_pull = self._acs_em_module.em_properties.thermal_orig_file

        output = self._exec("adb pull %s %s" % (file_to_pull, tmp_file), force_execution=True)
        if not self.is_shell_output_ok(output):
            txt = "fail to change thermal management: %s" % output
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        self.__thermal_file_instance = etree.parse(tmp_file, parser=None)

    def update_thermal_file(self):
        """
        push the thermal file on the DUT
        if no secure copy was done before, a secure copy will be do
        """
        if self.__thermal_file_instance is None:
            txt = "Cant update thermal file, you need first to upload it using load_thermal_file uecmd"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        if self.__thermal_change_done:
            # Set /system partition to read/write
            self._device.set_filesystem_rw()
            # create a secure copy of thermal file first
            file_api = self._device.get_uecmd("File", True)
            if not file_api.exist(self._acs_em_module.em_properties.thermal_orig_file)[0]:
                self._logger.info("[THERMAL] Make a secure copy of %s" % self._acs_em_module.em_properties.thermal_file)
                file_api.copy(self._acs_em_module.em_properties.thermal_file,
                                  self._acs_em_module.em_properties.thermal_orig_file)
                # copy the file permission
                permission = file_api.get_file_permissions(self._acs_em_module.em_properties.thermal_file, "root")[1]
                file_api.set_file_permission(self._acs_em_module.em_properties.thermal_orig_file, permission)
            else:
                permission = file_api.get_file_permissions(self._acs_em_module.em_properties.thermal_orig_file, "root")[1]
            # create a fake file with all the data
            tmpd = tempfile.gettempdir()
            tmp_file = os.path.join(tmpd, "thermal_temp_file.xml")
            docinfo = self.__thermal_file_instance.docinfo
            with open(tmp_file, "w") as xml_file:
                self.__thermal_file_instance.write(xml_file, pretty_print=False,
                                                encoding=docinfo.encoding, xml_declaration=True)
                # push it on the board
            time.sleep(2)
            self._exec("adb push %s %s" % (tmp_file, self._acs_em_module.em_properties.thermal_file), force_execution=True)
            self._exec("adb shell dos2unix %s" % self._acs_em_module.em_properties.thermal_file, force_execution=True)
            file_api.set_file_permission(self._acs_em_module.em_properties.thermal_file, permission)
            self.__thermal_change_done = False
        else:
            self._logger.warning("not change detected on loaded thermal file, cant update it on DUT")

    def modify_thermal_threshold(self, value, sensor="all", threshold="all", operation="="):
        """
        modify the sensor temperature threshold to a value depending of the operation you want to do.
        this command does not take care of the consistent of the value you put.
        For example, it will allow you to put a CRITICAL threshold at a value below ALERT threshold
        which does not make any sense.

        :type value: int
        :param value: the temperature value use to modify the threshold, the unit is milli degree.

        :type operation: str
        :param operation: the type of operation to apply on the threshold, to be chosen among + - / * =
                          leave empty to perform a = operation by default

        :type sensor: str
        :param sensor: the sensor to modify, by default will target all sensors

        :type threshold: str
        :param threshold: the threshold level to modify for a given sensor, by default will target all threshold

        """
        if self.__thermal_file_instance is None:
            txt = "Cant modify thermal file, you need first to upload it using load_thermal_file uecmd"
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)
        threshold = threshold.lower()
        threshold_list = []
        # read thermal file content
        main_root_node = self.__thermal_file_instance.getroot()
        # try to detect that we are in presence of profile
        root_node = main_root_node.xpath("%s[Name='Default']" % self.__cst_profile)
        if root_node in [None, []]:
            root_node = main_root_node
        else:
            root_node = root_node[0]

        self._logger.info("trying to modify %s threshold of %s sensor with operation %s%s" % (threshold, sensor,
                                                                                              operation, value))
        # ALL THRESHOLD
        if threshold == "all":
            threshold_list = self.__cst_zone_threshold.itervalues()
        # GIVEN THRESHOLD
        else:
            # create a single element list with the only threshold to set
            if threshold in self.__cst_zone_threshold.keys():
                threshold_list.append(self.__cst_zone_threshold[threshold.lower()])
            else:
                txt = "unknown threshold %s mapping on ACS side, you need to update the code if necessary" % threshold
                self._logger.error(txt)
                raise DeviceException(DeviceException.INVALID_PARAMETER, txt)

        for threshold_possible_names in threshold_list:
            for possible_name in threshold_possible_names:

                # ALL ENSOR
                if sensor == "all":
                    xpath_exp = "%s//%s" % (self.__cst_zone, possible_name)
                else:
                    # CASE GIVEN SENSOR
                    xpath_exp = "%s[translate(%s,'%s','%s')='%s']//%s" % (self.__cst_zone, self.__cst_zone_name,
                                                                            sensor.upper(),
                                                                            sensor.lower(),
                                                                            sensor.lower(),
                                                                            possible_name)
                for ele in root_node.xpath(xpath_exp):
                    if sensor != "all":
                        msg = "Trying to change threshold %s of sensor %s from %s to %s%s" % (possible_name, sensor,
                                                                                  str(ele.text), operation, value)
                    if operation != "=":
                        result = int(eval("%s%s%s" % (ele.text, operation, value)))
                        # I expect an ugly python crash here if eval fail
                        # value cant be < 0 then force it to 0
                        if result < 0 :
                            msg += ", the computed value %s is < 0, forcing it to 0 " % result
                            result = 0

                        ele.text = str(result)
                    else:
                        ele.text = str(value)
                    if sensor != "all":
                        msg += ": the new value is " + str(ele.text)
                        self._logger.debug(msg)
                    self.__thermal_change_done = True

    def restore_thermal_file(self):
        """
        restore the thermal file that is change by the other uecmd

        :rtype: boolean
        :return: True if the file has been restored, False otherwise
        """
        result = False
        file_api = self._device.get_uecmd("File", True)
        if file_api.exist(self._acs_em_module.em_properties.thermal_orig_file)[0]:
            # Set /system partition to read/write
            self._logger.info("[THERMAL] restore %s to .orig value" % self._acs_em_module.em_properties.thermal_file)
            # enable root
            self._device.enable_adb_root()
            # Set /system partition to read/write
            self._device.set_filesystem_rw()
            time.sleep(5)
            result, _ = file_api.copy(self._acs_em_module.em_properties.thermal_orig_file,
                              self._acs_em_module.em_properties.thermal_file)
        else:
            self._logger.error("[THERMAL] restore failed because file %s is missing" % self._acs_em_module.em_properties.thermal_orig_file)

        return result
