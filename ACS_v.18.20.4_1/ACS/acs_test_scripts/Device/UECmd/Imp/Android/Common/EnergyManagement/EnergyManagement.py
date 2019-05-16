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
:since: 21/06/2011
:author: vgombert, apairex
"""


import re
import tempfile
import time
import os
from random import randint
from acs_test_scripts.Utilities.EMUtilities import EMConstant as CST
from UtilitiesFWK.Utilities import Global, forced_str
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import AcsDict
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.EnergyManagement.IEnergyManagement import IEnergyManagement
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

from lxml import etree
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException


class EnergyManagement(BaseV2, IEnergyManagement):

    """
    :summary: Localization UEcommands operations for Android platform.
    """
    AUTOLOG_UEVENT = "msicLogger"
    AUTOLOG_THERMAL = "thermalLogger"

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IEnergyManagement.__init__(self, device)
        self._thermal_module = "acscmd.em.ThermalModule"
        self._em_module = "acscmd.em.EnergyModule"
        self._logger = device.get_logger()
        self.icategory = "intel.intents.category.ENERGYMANAGEMENT"
        self.component = "com.intel.acs.agent/.EnergyManagement"

        # GENERAL VAR
        self._right_bcu_path = None

        self.__charger_enable_file_path_checked = False

        # Get em module to get hardware specific config
        self._acs_em_module = self._device.get_device_module("EnergyManagementModule")
        self._acs_em_module.init()

        # VAR RELATED TO DIGITAL BATTERY
        self._valid_digital_batt_string = "ChrgProf"
        # VAR RELATED TO THERMAL
        self._thermal_captor_list = [CST.DTS, CST.BACKSKIN, CST.FRONTSKIN]

        # modified during execution
        self._mode_mapping = self._acs_em_module.em_properties.mode_mapping
        self._msic_shell_folder = self._acs_em_module.em_properties.msic_shell_folder
        # modified during execution
        self._charger_enable_file_path = self._acs_em_module.em_properties.charger_enable_file_path
        # to override in UC
        self._batt_capacity = self._acs_em_module.em_properties.batt_capacity
        # no more used
        self._as_extra_charger_info = self._acs_em_module.em_properties.as_extra_charger_info

    def __parse_interrupt_response(self, output):
        """
        Parses the response gotten from the scheduled get interrupt register
        in order to extract the following parameters:
        The function return at most 1 dictionary (for interrupt)
        Each object in the returned dictionary is a dictionary build from the "tag=value" parsed line format

        :type output: str
        :param output: output result from interrupt info read from board

        :type output: str
        :param output: interrupt name you are looking for

        :rtype: dict
        :return: a dictionary that contains the msic interrupt info
        """
        interrupt_register = {}
        interrupt = {}

        # Parse INTERRUPT section
        interrupt_info = output.splitlines()
        for interrupt_line in interrupt_info:
            interrupt_line = interrupt_line.strip()
            interrupt_name = interrupt_line.split("  ")[-1].strip()

            # ignore line where interrupt has no name or name is pure digit
            if len(interrupt_name) < 1 or interrupt_name.isdigit():
                continue
            # compute the sum of all interrupt
            combinated_value = None
            for value in re.findall(' [0-9]+ ', interrupt_line):
                if value.strip().isdigit():
                    if combinated_value is None:
                        combinated_value = 0
                    combinated_value += int(value)
                else:
                    # exist of the first non digit value
                    break
            # if there is no int value skip this entry
            if combinated_value is not None:
                interrupt.update({interrupt_name: (combinated_value, "none")})

        interrupt_register.update({"INTERRUPT": interrupt})
        return interrupt_register

    def _parse_msic_response(self, output, log_output=True):
        """
        Parses the response gotten from the scheduled get msic register
        in order to extract the following parameters:
        The function return at most 2 dictionaries (for battery and charger info)
        Each object in the returned dictionary is a dictionary build from the "tag=value" parsed line format

        :type output: str
        :param output: output result from msic registers embd uecmd

        :type  log_output: boolean
        :param log_output: allow output logging , used to avoid spaming log on autolog

        :rtype: dict
        :return: a dictionary that contains the msic battery and charger info
        """
        # if output is not valid raise an error
        if not self.is_shell_output_ok(output):
            txt = "failed to get msic info : %s" % output
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        msic_register = {}
        battery = {}
        charger = {}
        try:
            document = etree.fromstring(output.strip())
        except etree.Error as e:
            tmp_txt = "failed to parse msic uecmd response: " + str(e)
            self._logger.error(tmp_txt)
            self._logger.error("output file parsed :\n" + output.strip())
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        tmp_txt = None

        # check potential error first
        if document is None:
            tmp_txt = "parsing uevent msic info contains no information"
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        # get battery info
        battery_info = document.find("BATTERY")
        if battery_info is not None:
            error = battery_info.get("ERROR")
            if error is not None:
                tmp_txt = "Error on msic registers parsing battery info : " + error
        else:
            tmp_txt = "parsing uevent battery info contains no information"

        # check charger info
        charger_info = document.find("CHARGER")
        if charger_info is not None:
            error = charger_info.get("ERROR")
            if error is not None:
                tmp_txt = "Error on msic registers parsing charger info: " + error
        else:
            tmp_txt = "parsing uevent charger info contains no information"

        if tmp_txt is not None and "EXTRA CHARGER INFO" in tmp_txt :
            # raise error only if extra charger info is available
            if  self._as_extra_charger_info:
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        # get time stamp
        cosmetic_text = ""
        time_stamp = document.get("TIME_STAMP")
        if time_stamp is not None:
            msic_register.update({"TIME_STAMP": (str(time_stamp), "none")})
            if log_output:
                cosmetic_text = "[GENERAL info]:\n"
                cosmetic_text += "    TIME_STAMP = " + str(time_stamp) + "\n"
        else:
            msic_register.update({"TIME_STAMP": ("Failed to get", "none")})

        # get charger info
        for tag, value in charger_info.items():
            # anti unicode system
            tag = forced_str(tag)
            value = forced_str(value)
            # clean the tag
            tag = re.sub('^POWER_SUPPLY_', '', tag.upper().strip())
            # avoid adding error tag if there is no extra charger info
            if tag == "ERROR" and not self._as_extra_charger_info:
                continue

            # normal tag parsing
            if tag == "VOLTAGE_NOW":
                charger["VOLTAGE"] = (float(value) / 1000000, "V")
            elif tag in ["MAX_CHARGING_CURRENT", "INPUT_CUR_LIMIT", "MAX_CHARGE_CURRENT"]:
                temp_tag = tag
                if tag == "MAX_CHARGE_CURRENT":
                    temp_tag = 'MAX_CHARGING_CURRENT'
                charger[temp_tag] = (float(value) / 1000, "A")
            # below key are found on digital battery
            elif tag == "CHARGE_VOLTAGE":
                charger["VOLTAGE"] = (float(value) / 1000, "V")
            elif tag == "CHARGE_CURRENT":
                charger["CURRENT"] = (float(value) / 1000, "A")
            elif tag in ["ENABLE_CHARGER", "ENABLE_CHARGING"]:
                charger[tag] = (int(value), "none")
            else:
                charger[tag] = (str(value), "none")
        msic_register.update({"CHARGER": charger})

        if log_output:
            cosmetic_text += "[CHARGER info]:\n"
            for key in sorted(charger.keys()):
                value = charger[key]
                cosmetic_text += "    " + key + " = " + str(value[0])
                if value[1] != "none":
                    cosmetic_text += " " + str(value[1])
                cosmetic_text += "\n"

        # get battery info
        for tag, value in battery_info.items():
            # clean the tag
            tag = re.sub('^POWER_SUPPLY_', '', str(tag).upper().strip())
            if tag in ["CAPACITY"]:
                battery[tag] = (int(value), "none")
            # voltage tag
            elif "VOLTAGE" in tag:
                battery[tag] = (float(value) / 1000000, "V")
            elif tag in ["CURRENT_AVG", "CURRENT_NOW"]:
                battery[tag] = (float(value) / 1000000, "A")
            elif tag in ["CHARGE_NOW", "CHARGE_FULL", "CHARGE_FULL_DESIGN"]:
                battery[tag] = (int(value), "C")
            elif tag == "TEMP":
                battery[tag] = (int(value) / 10, "DegreeCelsius")
            elif tag == "STATUS":
                battery[tag] = (str(value).upper(), "none")
            else:
                battery[tag] = (str(value), "none")

        # harmonized tag name into one tag, first look for OCV else look for NOW
        if "VOLTAGE_OCV" in battery:
            battery["VOLTAGE"] = battery["VOLTAGE_OCV"]
        elif "VOLTAGE_NOW" in battery:
            battery["VOLTAGE"] = battery["VOLTAGE_NOW"]

        msic_register.update({"BATTERY": battery})

        if log_output:
            cosmetic_text += "[BATTERY info]:\n"
            for key in sorted(battery.keys()):
                value = battery[key]
                cosmetic_text += "    " + key + " = " + str(value[0])
                if value[1] != "none":
                    cosmetic_text += " " + str(value[1])
                cosmetic_text += "\n"

        if log_output:
            self._logger.debug(cosmetic_text)
            # check vital key on msic_register
        vital_key = ["CAPACITY", "CURRENT_NOW", "VOLTAGE", "STATUS"]

        if "BATTERY" not in msic_register or not set(vital_key).issubset(msic_register["BATTERY"].keys()):
            tmp_txt = "missing vital BATTERY keys on msic registers: %s" \
                      % set(vital_key).difference(msic_register["BATTERY"].keys())
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        return msic_register

    def _parse_msic_response_from_shell(self, output, log_output=True):
        """
        Parses the response gotten from the scheduled get msic register
        in order to extract the following parameters:
        The function return at most 2 dictionaries (for battery and charger info)
        Each object in the returned dictionary is a dictionary build from the "tag=value" parsed line format

        :type output: str
        :param output: output result from msic registers embd uecmd

        :type  log_output: boolean
        :param log_output: allow output logging , used to avoid spaming log on autolog

        :rtype: dict
        :return: a dictionary that contains the msic battery and charger info
        """
        # if output is not valid raise an error
        if not self.is_shell_output_ok(output):
            txt = "failed to get msic info : %s" % output
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        msic_register = self._acs_em_module.parse_msic_response_from_shell(output, log_output)
        return msic_register

    def get_msic_registers(self, behavior=None,
                           behavior_value=None):
        """
        Gets the msic registers.

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
            output = self._internal_exec_v2(self._em_module, "scheduledGetUeventInfo", "--ei delay %s" % behavior_value, is_system=True)
            self._logger.debug("Task %s will start in %s seconds" % (output[self.OUTPUT_MARKER], behavior_value))
            return output[self.OUTPUT_MARKER]

        elif behavior == "read":
            self._logger.info("read get uevent info result")
            output = self.get_async_result(str(behavior_value))

        else:
            self._logger.info("get uevent info")
            output = self._internal_exec_v2(self._em_module, "getUeventInfo", is_system=True)

        for keys in output.keys():
            output[keys] = self._format_text_ouput(output[keys])

        if (not isinstance(output, AcsDict) and not isinstance(output, dict)) or self.OUTPUT_MARKER not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found : %s" % (self.OUTPUT_MARKER, str(output)))

        return self._parse_msic_response(output[self.OUTPUT_MARKER])

    def poll_multi_msic_registers(self, start_delay,
                                  duration, ack_delay):
        """
        start to poll msic registers for a given duration.

        :type start_delay: int
        :param start_delay: delay before starting to poll in seconds

        :type duration: int
        :param duration: polling duration in seconds

        :type ack_delay: int
        :param ack_delay: time between 2 polling in seconds

        :rtype: str
        :return: task id
        """
        self._logger.info("schedule get multi uevent info")
        args = "--ei delay %s --ei duration %s --ei ackDelay %s" % (start_delay, duration, ack_delay)
        output = self._internal_exec_v2(self._em_module, "scheduledGetUeventInfo", args, is_system=True)
        self._logger.debug("Task %s will start in %ss for a duration of %ss and a polling delay of %ss" %
                           (output[self.OUTPUT_MARKER], start_delay, duration, ack_delay))
        return output[self.OUTPUT_MARKER]

    def get_multi_msic_registers(self, task_id):
        """
        Gets the values of msic registers for a given duration.

        :type task_id: str
        :param task_id: id of the task

        :rtype: list of dict
        :return: list of dict which contains msic info
        """
        self._logger.info("get multi uevent info result for task %s " % task_id)
        output = self.get_async_result(task_id)

        for keys in output.keys():
            output[keys] = self._format_text_ouput(output[keys])

        if (not isinstance(output, AcsDict) and not isinstance(output, dict)) or self.OUTPUT_MARKER not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found : %s" % (self.OUTPUT_MARKER, str(output)))

        msic_info = output[self.OUTPUT_MARKER]
        splited_output = msic_info.split(self.AUTO_LOG_NEW_ENTRY_SEPARATOR)
        result = []
        for dico in splited_output:
            result.append(self._parse_msic_response(dico.strip()))
        return result

    def get_proc_interrupt(self, behavior=None, behavior_value=None):
        """
        Measures the number of interrupt which happen when charger type is plugged.
        Work only in MOS

        :type behavior: str
        :param behavior: change the behavior of the method :
                         - "scheduled" to schedule the method execution
                         - "read" to read the output of a scheduled method

        :type behavior_value: int
        :param behavior_value: the value link to the behavior:
                         - time to wait for "scheduled" option
                         - pid given by a former scheduled method execution

        :rtype: str for pid or int for result
        :return: pid of scheduled operation or number of interrupts caught
        """
        if behavior == "scheduled":
            task_id = "TASK_" + str(randint(1000, 9999))
            task = self._msic_shell_folder + "/" + task_id
            cmd_core_scheduled = "cat /proc/interrupts"
            script_name = self._msic_shell_folder + "SCRIPT_" + task_id + ".sh"

            cmd_core = "#!sh\\nsleep " + str(behavior_value) + " && " + cmd_core_scheduled

            cmd = "adb shell echo \'%s\' > %s" % (cmd_core, script_name)
            self._exec(cmd, force_execution=True)

            # Set execution permissions
            cmd = "adb shell chmod 777 %s" % script_name

            # in ROS, nohup is not available
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
            cmd = "adb shell %snohup sh %s > %s" % (ros_parry, script_name, task)
            self._exec(cmd, force_execution=True, wait_for_response=False)
            return task_id

        elif behavior == "read":
            task = self._msic_shell_folder + str(behavior_value)
            cmd = "adb shell cat " + task
            output = self._exec(cmd, force_execution=True)
            # delete task
            self._exec("adb shell rm -f " + task, force_execution=True)
        else:
            self._logger.info("get interrupts info")
            output = self._exec("adb shell cat /proc/interrupts", force_execution=True)

        # code reach only in read or normal execution case
        self._logger.debug(output)
        if not self.is_shell_output_ok(output):
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "get_proc_interrupt - failed to parse output: %s " % output)

        dico = self.__parse_interrupt_response(output)["INTERRUPT"]
        result = None
        for battery_name in self._acs_em_module.em_properties.battery_interrupts:
            result = dico.get(battery_name)
            if result is not None:
                break

        if result is None:
            txt = "interrupt %s failed to be found in /proc/interrupts" \
                  % self._acs_em_module.em_properties.battery_interrupts
            self._logger.debug(txt)
            # test if we can find a default one
            self._logger.debug("check if a default battery interrupt containing string 'battery' can be seen.")
            result = self.__check_default_battery_interrupts(dico)

            if result is None:
                txt = "interrupt %s and default 'battery' failed to be found in /proc/interrupts" % \
                      self._acs_em_module.em_properties.battery_interrupts
                self._logger.error(txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        return result[0]

    def __check_default_battery_interrupts(self, interupt_dict):
        """
        check if we can find a default battery interrupts
        """
        for key in interupt_dict.keys():
            if "battery" in key:
                self._logger.info("found [%s] interrupt that may be the battery interrupts" % key)
                return interupt_dict[key]

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

        i.e [DTS] = (35 , DegreeCelsius)
        if there is no unit, unit value will be set to none.
        """
        if behavior == "scheduled":
            self._logger.info("[THERMAL] schedule get sensor information")
            output = self._internal_exec_v2(self._thermal_module, "scheduledGetThermalInfo",
                                            "--ei delay %s --es path %s"
                                            % (behavior_value, self._acs_em_module.em_properties.thermal_file), is_system=True)
            self._logger.debug("[THERMAL] Task %s will start in %s seconds"
                               % (output[self.OUTPUT_MARKER], behavior_value))
            return output[self.OUTPUT_MARKER]

        elif behavior == "read":
            self._logger.info("[THERMAL] read get thermal sensor info result from task %s" % behavior_value)
            output = self.get_async_result(str(behavior_value))

        else:
            self._logger.info("[THERMAL] get sensor information")
            output = self._internal_exec_v2(self._thermal_module,
                                            "getThermalInfo", "--es path %s" % self._acs_em_module.em_properties.thermal_file, is_system=True)

        for keys in output.keys():
            output[keys] = self._format_text_ouput(output[keys])

        if (isinstance(output, AcsDict) or isinstance(output, dict)) and self.OUTPUT_MARKER in output:
            self._logger.debug(output[self.OUTPUT_MARKER])
        else:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found : %s" % (self.OUTPUT_MARKER, str(output)))

        return self.__parse_thermal_sensor_response(output[self.OUTPUT_MARKER])

    def __parse_thermal_sensor_response(self, output, only_for_interface_matching=True):
        """
        Parses the response gotten from the scheduled get thermal sensor info
        in order to extract the following parameters:
        The function as many dict as sensor available like "CPU", "BACKSKIN, "FRONTSKIN"
        Each object in the returned dictionary is a dictionary build from the "tag = value" parsed line format

        :type output: str
        :param output: output result from thermal conf embd uecmd

        :rtype: dict
        :return:dictionary of the thermal conf info
        .. warning:: only_for_interface_matching paramter is here only to match with other implementation
                 of this private function
        """
        thermal_conf = {}

        # get time stamp
        if output.find("<TIME_STAMP>") != -1 and output.find("</TIME_STAMP>") != -1:
            stamp_text = output[output.find("<TIME_STAMP>"):output.find("</TIME_STAMP>") + len("</TIME_STAMP>")]
            time_stamp = output[output.find("<TIME_STAMP>") + len("<TIME_STAMP>"):
                                output.find("</TIME_STAMP>")].strip()
            thermal_conf.update({"TIME_STAMP": (time_stamp, "none")})
            output = output.replace(stamp_text, "")
        else:
            thermal_conf.update({"TIME_STAMP": ("Failed to get", "none")})

        # get sensor info
        if output.find("<THERMAL_CONF>") != -1:
            splited_output = output.split("<end>")

            for block in splited_output:
                # Parse sensor sections
                if block.lower().find("name=") != -1:
                    sensor = {}
                    zone_info = block.splitlines()
                    sensor_name = ""
                    intercept = None
                    slope = None
                    for element in zone_info:
                        if element.find("=") != -1:
                            element = element.split("=")
                            element_tag = element[0].strip().upper()
                            element_value = element[1].strip()

                            if element_tag == "TEMP_THRESHOLD_NORMAL":
                                sensor.update(
                                    {CST.THRESHOLD_NORMAL: (float(element_value) / 1000,
                                     CST.DEGREE_CELSIUS)})
                            elif element_tag == "TEMP_THRESHOLD_WARNING":
                                sensor.update(
                                    {CST.THRESHOLD_WARNING: (float(element_value) / 1000,
                                     CST.DEGREE_CELSIUS)})
                            elif element_tag == "TEMP_THRESHOLD_ALERT":
                                sensor.update(
                                    {CST.THRESHOLD_ALERT: (float(element_value) / 1000,
                                     CST.DEGREE_CELSIUS)})
                            elif element_tag.find("NAME") != -1:
                                sensor_name = element_value.upper()
                            elif element_tag.find("INTERCEPT") != -1:
                                intercept = float(element_value)
                            elif element_tag.find("SLOPE") != -1:
                                slope = float(element_value)

                    if sensor_name != "" and output.find("<" + sensor_name + ">") != -1:
                        value = output[output.find("<" + sensor_name + ">") +
                                       len("<" + sensor_name + ">"): output.find("</" + sensor_name + ">")].strip()

                        if value.replace(".", "").replace("-", "").isdigit():
                            value = float(value)
                            if slope is not None and intercept is not None:
                                value = (value * slope) + intercept
                            sensor[CST.VALUE] = (float(value) / 1000, CST.DEGREE_CELSIUS)
                            # compute temperature state
                            state = CST.UNKNOWN
                            if CST.THRESHOLD_ALERT in sensor:
                                if sensor[CST.VALUE][0] < sensor[CST.THRESHOLD_ALERT][0]:
                                    state = CST.ALERT
                                else:
                                    state = CST.CRITICAL

                            if CST.THRESHOLD_WARNING in sensor:
                                if sensor[CST.VALUE][0] < sensor[CST.THRESHOLD_WARNING][0]:
                                    state = CST.WARNING

                            if CST.THRESHOLD_NORMAL in sensor:
                                if sensor[CST.VALUE][0] < sensor[CST.THRESHOLD_NORMAL][0]:
                                    state = CST.NORMAL

                            sensor[CST.STATE] = (state, "none")

                    thermal_conf.update({sensor_name: sensor})

        return thermal_conf

    def get_fuel_gauging_monitoring_time_result(self):
        """
        run a script which schedule the battery
        fuel Gauging monitoring delta time

        :rtype: tuple
        :return: tuple of the delta time, and the time of the last battery level update
        """
        self._logger.info("Get delta time result")
        # add 2  aplogs path for retro compatibility
        potential_folder = ["/data/logs", "/logs"]
        aplog_folder = "/logs"
        for folder in potential_folder:
            opt = self._exec("adb shell ls %s" % folder, timeout=10)
            txt = opt[1].lower()
            if txt.find("aplog") != -1 and self.is_shell_output_ok(txt):
                aplog_folder = folder
                break

        cmd = "adb shell grep -s battery_level %s/a* | tail -n2 | cut -d ' ' -f2" % aplog_folder
        txt = self._exec(cmd, timeout=30)
        if self.is_shell_output_ok(txt):
            output = txt
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        # check the response
        if output in [None, ""]:
            tmp_txt = "no aplogs found"
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        splited_output = output.splitlines()
        # Check format
        if len(splited_output) < 2:
            tmp_txt = "response format is incorrect from delta_time_result execution"
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
        # compute the delta time
        line0 = splited_output[0].split(":")
        line1 = splited_output[1].split(":")
        delta_time = (float(line1[1]) - float(line0[1])) * 60 + (float(line1[2]) - float(line0[2]))

        result = (delta_time, splited_output[1])

        return result

    @need('usb_charging', False)
    def set_usb_charging(self, mode):
        """
        set the usb charging on or off.
        need to be refresh once in a while.

        :type mode: str or int
        :param mode: can be ('on') to enable
                            ('off') to disable
                            ('low', 'medium', 'high') to choose the value of the current

        :return: None
        """
        if mode in self._mode_mapping:
            self._logger.info("set usb charging to %s" % mode)
        else:
            self._logger.error("set_usb_charging : Parameter mode %s is not valid" % mode)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "mode is not valid !")
        # check usb charging path
        self.__configure_set_usb_charging()

        cmd = "adb shell echo %s > %s" % (self._mode_mapping[mode],
                                          self._charger_enable_file_path)
        output = self._exec(cmd)
        if not self.is_shell_output_ok(output):
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "file %s not found " % self._charger_enable_file_path)

    def __configure_set_usb_charging(self):
        """
        check file path and cooling device to find out which file to control
        """
        if not self.__charger_enable_file_path_checked:
            self._logger.debug("configuring set_usb_charging uecmds")
            # try with the path set in init
            found = False
            cmd = "adb shell ls %s" % self._charger_enable_file_path
            output = self._exec(cmd).strip()
            if self.is_shell_output_ok(output, True) and len(output) > 0:
                found = True

            # else parse cooling device
            if not found:
                for key in self._acs_em_module.em_properties.possible_charger_name:
                    cmd = "adb shell grep -l -i %s /sys/class/thermal/cool*/type" % key
                    output = self._exec(cmd).strip()
                    if self.is_shell_output_ok(output, True) and len(output) > 0:
                        # if a cooling device is found, reevaluate the mode mapping
                        self._mode_mapping = self._acs_em_module.em_properties.mode_mapping_cooling_device
                        output = output.replace("/type", "/cur_state")
                        found = True
                        break

            if found:
                self._charger_enable_file_path = output
                self.__charger_enable_file_path_checked = True
                return

            msg = "set_usb_charging: problem to modif the charge level, %s not found and no cooling device for charger seen" % self._charger_enable_file_path
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

    @need('usb_charging', False)
    def get_charger_level(self):
        """
        Gets the charger level info

        :rtype: int
        :return: The charger level (1=>100mA, 2=>500mA, 3=>1A, 4=>1.5A)
        """
        self._logger.info("get Charger level")

        # check usb charging path
        self.__configure_set_usb_charging()

        cmd = "adb shell cat " + self._charger_enable_file_path
        output = self._exec(cmd, force_execution=True).strip()

        if output.isdigit():
            return int(output)
        else:
            self._logger.error("get_charger_level : unable to get Charger level")
            raise DeviceException(DeviceException.OPERATION_FAILED, "get_charger_level : unable to get Charger level")

    def get_bcu_status(self):
        """
        Get the burst control unit status.

        :rtype: str
        :return: str with 2 digits as hexa like 0xFF
        """
        # evaluate possible path for BCU, this will be done only one time for
        # all uecmds that need this path
        bcu_path = self._get_right_bcu_path()
        if not bcu_path:
            self._logger.warning("cannot get bcu path, nothing to do!")
            return None
        else:
            file_path = "%s%s" % (self._right_bcu_path, "action_status")

            cmd = "adb shell cat %s" % file_path
            output = self._exec(cmd)
            if output.find("No such file or directory") != -1:
                raise DeviceException(DeviceException.OPERATION_FAILED, "file %s not found " % file_path)
        return output.strip()

    def _get_right_bcu_path(self):
        """
        init the right path for bcu uecmds
        """
        if self._right_bcu_path is None:
            if self._acs_em_module.em_properties.get("msic_bcu_current_path"):
                if isinstance(self._acs_em_module.em_properties.msic_bcu_current_path, list):
                    # work arround for ctp K3.0 and K3.4
                    for path in self._acs_em_module.em_properties.msic_bcu_current_path:
                        output = self._exec("adb shell ls " + path)
                        if self.is_shell_output_ok(output):
                            self._right_bcu_path = path
                            break
                else:
                    self._right_bcu_path = self._acs_em_module.em_properties.msic_bcu_current_path

                if self._right_bcu_path is None:
                    msg = "BCU folder not found"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                self._right_bcu_path = ""
        return self._right_bcu_path

    def get_bcu_interrupt(self):
        """
        Measures the number of interrupt related to the BCU.

        :rtype: int
        :return: number of interrupts caught
        """
        cmd = "adb shell cat /proc/interrupts"
        output = self._exec(cmd)
        # code reach only in read or normal execution case
        self._logger.debug(output)
        if not self.is_shell_output_ok(output):
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "get_bcu_interrupt - failed to parse output: %s " % output)

        result = self.__parse_interrupt_response(output)["INTERRUPT"]
        bcu = None
        for bcu_path in self._acs_em_module.em_properties.bcu_interrput:
            bcu = result.get(bcu_path)
            if bcu is not None:
                break

        if bcu is None:
            txt = "known interrupt %s failed to be found in /proc/interrupts" % self._acs_em_module.em_properties.bcu_interrput
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        return bcu[0]

    def get_autolog_msic_registers(self):
        """
        Gets the uevent battery/charger auto logging result.

        :rtype: list of (dict, dict)
        :return: return a list containing tuples of (special stamp, msic dict)
            special stamp contains the following keys ("AUTOLOG_HOURS", "REBOOT", "REBOOT_TIME")
        """
        self._logger.info("read auto logger result for msic register")
        output = self._get_autolog_result(self.AUTOLOG_UEVENT)
        self._logger.debug(output)

        msic_dict_list = []
        i = 0
        for (dico, element) in output:
            i += 1
            try:
                msic_dict_list.append((dico, self._parse_msic_response(element, False)))
            # raise error only if it is not the last element read
            # this is to avoid reading half cut info
            except Exception as e:
                if i < len(output):
                    raise e
                else:
                    self._logger.warning("error happen in last info read : %s" % str(e))

        return msic_dict_list

    def get_autolog_thermal_sensor_info(self):
        """
        Gets the thermal conf auto logging result.

        :rtype: list of (dict, dict)
        :return: return a list containing tuples of (special stamp, thermal conf dict)
            special stamp contains the following keys ("AUTOLOG_HOURS", "REBOOT", "REBOOT_TIME")
        """
        self._logger.info("read auto logger result for thermal sensor")
        output = self._get_autolog_result(self.AUTOLOG_THERMAL)

        thermal_dict_list = []
        i = 0
        for (dico, element) in output:
            i += 1
            try:
                thermal_dict_list.append((dico, self.__parse_thermal_sensor_response(element, False)))
                # raise error only if it is not the last element read
                # this is to avoid reading half cut info
            except Exception as e:
                if i < len(output):
                    raise e
                else:
                    self._logger.warning("error happen in last info read : %s" % str(e))

        return thermal_dict_list

    def build_fake_thermal_conf(self):
        """
        make a fake MID_thermal_em.conf to emule fake temperature
        """

        self._logger.info("[THERMAL] Make a a fake thermal conf of %s" % self._acs_em_module.em_properties.thermal_file)
        phone_system = self._device.get_uecmd("PhoneSystem", True)

        # Set /system partition to read/write
        self._device.set_filesystem_rw()

        # get the content from files
        # get value from the .orig if it exist

        if phone_system.check_file_exist_from_shell(self._acs_em_module.em_properties.thermal_orig_file):
            self._logger.info("[THERMAL] Secure copy .orig exist , get value from it")
            tmp_file = phone_system.read_file(self._acs_em_module.em_properties.thermal_orig_file)
        else:
            tmp_file = phone_system.read_file(self._acs_em_module.em_properties.thermal_file)
            self._logger.info("[THERMAL] Make a secure copy of %s" % self._acs_em_module.em_properties.thermal_file)
            phone_system.copy(self._acs_em_module.em_properties.thermal_file,
                              self._acs_em_module.em_properties.thermal_orig_file)

        for zone_id in self._thermal_captor_list:
            phone_system.write_file("/data/fake_temp_%s" % zone_id,
                                          "30000")
        output_file = ""
        zone = ""

        # Config file modifications
        tmp_file = tmp_file.splitlines()

        for line in tmp_file:

            if line.lower().find("name=") != -1:
                zone = line.split("=")[1].strip().upper()

            if zone in self._thermal_captor_list:
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
                          self._acs_em_module.em_properties.thermal_file, 60)

    def restore_thermal_file(self):
        """
        restore the real thermal configuration

        :rtype: boolean
        :return: True if the file has been restored, False otherwise
        """
        # In case of the backup conf file was the same as the working temp file,
        # we restore the temperatures to normal state
        result = False
        phone_system = self._device.get_uecmd("PhoneSystem", True)
        for zone_id in self._thermal_captor_list:
            phone_system.write_file("/data/fake_temp_%s" % zone_id,
                                          "30000")
        # restore conf file
        if phone_system.check_file_exist_from_shell(self._acs_em_module.em_properties.thermal_orig_file):
            self._logger.info("[THERMAL] restore %s to .orig value" % self._acs_em_module.em_properties.thermal_file)
            cmd = "adb shell rm %s" % self._acs_em_module.em_properties.thermal_file
            self._exec(cmd, timeout=5, wait_for_response=False)
            phone_system.copy(self._acs_em_module.em_properties.thermal_orig_file,
                              self._acs_em_module.em_properties.thermal_file)
            result = True
            time.sleep(10)
        else:
            self._logger.error("[THERMAL] restore failed because file %s is missing" % self._acs_em_module.em_properties.thermal_orig_file)

        return result

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
            cmd_write = "adb shell echo -e '%s' > /data/fake_temp_%s"\
                % (temperature, thermal_captor)
            result = self._device.run_cmd(cmd_write, 3, force_execution=True)
            # check if uecmd was received by the phone
            if result[0] == Global.SUCCESS:
                self._logger.info("[THERMAL] the board is on,"
                                  + " check change on thermal captor")
                # check if the change was done
                cmd_read = "adb shell cat /data/fake_temp_%s" % thermal_captor
                result = self._device.run_cmd(cmd_read, 3, force_execution=True)
                if str(temperature) in result[1]:
                    return True
                else:
                    self._logger.error("[THERMAL] fail to restore thermal"
                                       + " captor temp on %s" % thermal_captor)
                    return False

    def thermal_test_set_temp(self, zone_id, temp):
        """
        Set the fake temperature to a specific zone

        :type  zone_id: int
        :param zone_id: number of the zone to set. Should be between 0 and 2
        :type  temp: int
        :param temp: Temperature value to set in milli degre Celcius
        """
        phone_system = self._device.get_uecmd("PhoneSystem", True)
        phone_system.write_file("/data/fake_temp_%s" % zone_id,
                                      int(temp), force_execution=True)

    def is_digital_battery_valid(self):
        """
        This UECmd permits to verify the battery validity
        Search the "researched_string" in the dmesg logs

        :type  researched_string: string
        :param researched_string: String in the dmesg logs who permits to find the battery information
        :rtype : bool
        :return : True is battery is value, false otherwise
        """

        battery_verdict = False
        cmd = 'adb shell "dmesg |grep %s"' % self._valid_digital_batt_string
        output = self._exec(cmd)
        output = output.strip()

        if output == "":
            battery_verdict = False
        else:
            battery_verdict = True

        return battery_verdict

    def get_battery_state(self):
        """
        This URCmd permits to check the battery state
        :rtype: str
        :return: the battery state
        """
        status = ""

        cmd = "adb shell dumpsys battery | grep status"
        result = self._device.run_cmd(cmd, 3, force_execution=True)

        try:
            result = int(result[-1].split(':')[-1])
        except:
            pass

        if result == 1:
            status = "BATTERY_STATUS_UNKNOWN"
        elif result == 2:
            status = "BATTERY_STATUS_CHARGING"
        elif result == 3:
            status = "BATTERY_STATUS_DISCHARGING"
        elif result == 4:
            status = "BATTERY_STATUS_NOT_CHARGING"
        elif result == 5:
            status = "BATTERY_STATUS_FULL"

        return status

    def get_bcu_activation_status(self):
        """
        Check if the BCU is running or not.

        :rtype: int
        :return: 1 if it is running, 0 otherwise
        """
        # evaluate possible path for BCU, this will be done only one time for
        # all uecmds that need this path
        bcu_path = self._get_right_bcu_path()
        if not bcu_path:
            self._logger.warning("cannot get bcu path, nothing to do!")
            return None
        else:
            file_path = "%s%s" % (self._right_bcu_path, "bcu_status")

            cmd = "adb shell cat %s" % file_path
            output = self._exec(cmd)
            # if output is not valid raise an error
            if not self.is_shell_output_ok(output):
                raise DeviceException(DeviceException.OPERATION_FAILED, "file %s not found " % file_path)
        return int(output.strip())

    def set_bcu_warn_level(self, voltage, warn_type):
        """
        Set the bcu warn A or B voltage level

        :type voltage: float
        :param voltage: the voltage level to set as warn level
        :type warn_type: str
        :param warn_type: the warn type to change; it can be "A" or "B" or "_crit"

        :rtype: none
        """
        # evaluate possible path for BCU, this will be done only one time for
        # all uecmds that need this path
        bcu_path = self._get_right_bcu_path()
        if not bcu_path:
            self._logger.warning("cannot get bcu path, nothing to do!")
        else:
            self._logger.info("Trying to set voltage_warn%s to %s V" % (warn_type, str(voltage)))
            if not warn_type in ["A", "B", "_crit"]:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Parameter warn_type %s is not valid !" % warn_type)
            if voltage < 0 or voltage > 4.2:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Parameter voltage %s is out of the accepted range" % voltage)

            file_path = "%s%s" % (self._right_bcu_path, "voltage_warn" + warn_type)
            cmd = "adb shell echo %s > %s" % (str(int(voltage * 1000)), file_path)
            output = self._exec(cmd)
            # if output is not valid raise an error
            if not self.is_shell_output_ok(output):
                raise DeviceException(DeviceException.OPERATION_FAILED, "file %s error : %s" % (file_path, output))

    def get_bcu_warn_level(self, warn_type):
        """
        get the bcu warn A or B voltage level

        :type warn_type: str
        :param warn_type: the warn type to change; it can be "A" or "B" or "_crit"

        :rtype: float
        :return: the voltage level to set as warn level
        """
        # evaluate possible path for BCU, this will be done only one time for
        bcu_path = self._get_right_bcu_path()
        if not bcu_path:
            self._logger.warning("cannot get bcu path, nothing to do!")
            return None
        else:
            self._logger.info("Trying to get voltage_warn%s" % warn_type)
            if not warn_type in ["A", "B", "_crit"]:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Parameter warn_type %s is not valid !" % warn_type)

            file_path = "%s%s" % (self._right_bcu_path, "voltage_warn" + warn_type)

            cmd = "adb shell cat %s" % file_path
            output = self._exec(cmd)
            # if output is not valid raise an error
            if not self.is_shell_output_ok(output):
                raise DeviceException(DeviceException.OPERATION_FAILED, "file %s not found " % file_path)
            return float(output.strip()) / 1000
