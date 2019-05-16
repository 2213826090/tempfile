"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: SII on behalf of INTEL MCG PSI
:summary: This file implements Energy Management UECmds
:since: 11 Feb 2014
:author: vgomberx
"""
from acs_test_scripts.Device.UECmd.Interface.EnergyManagement.IEnergyManagement import IEnergyManagement
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from ErrorHandling.DeviceException import DeviceException


class EnergyManagement(Base, IEnergyManagement):

    """
    Class that handles all energy management related operations.
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        IEnergyManagement.__init__(self, device)

    # TODO: need to be renamed
    def get_msic_registers(self, behavior=None,
                           behavior_value=None):
        """
        Gets the battery and charger info.

        :type behavior: str
        :param behavior:
                        - "scheduled" to schedule the operation
                        - "read" to read the output from a scheduled operation
                        - None to act like normally

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
                :return: a dictionary that contains 2 keys : [CHARGER], [BATTERY]

        if behavior is equal to None:

                :type behavior_value: None
                :param behavior_value: not used

                :rtype: dict
                :return: a dictionary that contains 2 keys : [CHARGER], [BATTERY]

        the elements of returned dictionary are like below :
             - (value, unit) = [register name][property]

        eg: (10V, unit) = [CHARGER][VOLTAGE]
        if there is no unit , unit value will be set to none.
        """
        module_name, class_name = self._get_module_and_class_names("EnergyManagement")
        if behavior == "scheduled":
            self._logger.info("schedule get uevent info to start in %ss" % behavior_value)
            # leave here if scheduled as we want only an id
            return self.schedule_uecmd(module_name, class_name, "GetBatteryInfo", "",
                                       int(behavior_value))

        elif behavior == "read":
            self._logger.info("read get uevent info result")
            output = self.get_async_result(str(behavior_value))

        else:
            self._logger.info("get uevent info")
            output = self._internal_uecmd_exec(module_name, class_name, "GetBatteryInfo")

        if output.get("values") is None or output["values"].get("info") is None:
            tmp_txt = "Not able to retrieve info on battery/charger from side embedded"
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        return self.__parse_embedded_response(output["values"]["info"])

    def __parse_embedded_response(self, output, log_output=True):
        """
        Parses the response gotten from windows embedded code.
        in order to extract the following parameters:
        The function return at most 2 dictionaries (for battery and charger info)
        Each object in the returned dictionary is a dictionary build from the "tag=value" parsed line format

        :type output: str
        :param output: output result from msic registers embd uecmd

        :type  log_output: boolean
        :param log_output: allow output logging , used to avoid spamming log on autolog

        :rtype: dict
        :return: a dictionary that contains the msic battery and charger info
        """
        main_dict = {}
        battery = {}
        charger = {}
        tmp_txt = ""
        formated_output = {}
        # anti camel json formating
        for key in output:
            formated_output[str(key).upper()] = output[key]

        # get battery info
        battery_info = formated_output.get("BATTERY")
        if battery_info is None or len(battery_info) == 0:
            tmp_txt += "Parsed battery info contains no information;"

        # check charger info
        charger_info = formated_output.get("CHARGER")
        if charger_info is None or len(charger_info) == 0:
            # tmp_txt += "Parsed charger info contains no information;"
            pass

        if tmp_txt != "":
                self._logger.error(tmp_txt)
                raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        # get time stamp
        time_stamp = formated_output.get("TIME_STAMP")
        cosmetic_text = ""
        if time_stamp is not None:
            main_dict.update({"TIME_STAMP": (str(time_stamp), "none")})
            if log_output:
                cosmetic_text = "[GENERAL info]:\n"
                cosmetic_text += "    TIME_STAMP = " + str(time_stamp) + "\n"
        else:
            main_dict.update({"TIME_STAMP": ("Failed to get", "none")})

        # get charger info
        for key in charger_info.keys():
            # anti unicode system
            tag = str(key).upper().strip()
            value = charger_info.get(key)
            # clean the tag
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
            else:
                charger[tag] = (str(value), "none")
        main_dict.update({"CHARGER": charger})

        # format charger info for logger
        if log_output:
            cosmetic_text += "[CHARGER info]:\n"
            for key in sorted(charger.keys()):
                value = charger[key]
                cosmetic_text += "    " + key + " = " + str(value[0])
                if value[1] != "none":
                    cosmetic_text += " " + str(value[1])
                cosmetic_text += "\n"

        # get battery info
        for key in battery_info.keys():
            # anti unicode system
            tag = str(key).upper().strip()
            value = battery_info.get(key)
            # clean the tag
            if tag in ["PRESENT", "CAPACITY"]:
                battery[tag] = (int(value), "none")
            # voltage tag
            elif tag in ["STATUS"]:
                battery[tag] = (str(value.get("item1")).upper(), "none")
            elif tag in ["VOLTAGE"]:
                battery[tag] = (float(value) / 1000, "V")
            elif tag in ["CURRENT_AVG", "CURRENT_NOW"]:
                battery[tag] = (float(value) / 1000000, "A")
            elif tag in ["CHARGE_NOW", "CHARGE_FULL", "CHARGE_FULL_DESIGN"]:
                battery[tag] = (int(value), "C")
            elif tag == "TEMP":
                battery[tag] = (float(value) / 10, "DegreeCelsius")
            else:
                battery[tag] = (str(value), "none")

        # harmonized tag name into one tag, first look for OCV else look for NOW
        if "VOLTAGE_OCV" in battery:
            battery["VOLTAGE"] = battery["VOLTAGE_OCV"]
        elif "VOLTAGE_NOW" in battery:
            battery["VOLTAGE"] = battery["VOLTAGE_NOW"]
        main_dict.update({"BATTERY": battery})

        # format battery info for logger
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
        # check vital key on main_dict
        vital_key = ["CAPACITY", "VOLTAGE", "STATUS"]

        if "BATTERY" not in main_dict or not set(vital_key).issubset(main_dict["BATTERY"].keys()):
            tmp_txt = "missing vital BATTERY keys on EM embedded info: %s" \
                % set(vital_key).difference(main_dict["BATTERY"].keys())
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        return main_dict
