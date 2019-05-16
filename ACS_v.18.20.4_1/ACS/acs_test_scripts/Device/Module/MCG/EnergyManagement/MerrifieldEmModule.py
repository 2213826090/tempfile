#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
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
@summary: Energy Management module to expose energy management properties
@since: 7/8/14
@author: kturban
"""

from re import sub
from lxml import etree
from Device.Module.DeviceModuleBase import DeviceModuleBase
from acs_test_scripts.Device.Module.MCG.EnergyManagement.IEmModule import IEmModule
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.AttributeDict import AttributeDict
from UtilitiesFWK.Utilities import Global


class MerrifieldEmModule(IEmModule, DeviceModuleBase):
    def __init__(self):
        super(MerrifieldEmModule, self).__init__()
        self._em_properties = AttributeDict()

    @property
    def em_properties(self):
        return self._em_properties

    def init(self):
        """
        Initialize em module

        :rtype: UtilitiesFWK.Utilities.Global
        :return: init status
        """
        verdict = Global.SUCCESS
        # for now : em_properties = module config
        # but we can imagine a generic em properties interface between windows/android/linux
        # module would be os specific but keys inside em_properties would be generic
        self._em_properties = self.configuration
        return verdict


    def parse_msic_response_from_shell(self, output, log_output=True):
        """
        Parses the response gotten from the  get shell msic register
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
        msic_register = {}
        battery = {}
        charger = {}
        try:
            document = etree.fromstring(output.strip())
        except etree.Error as e:
            tmp_txt = "failed to parse msic uecmd response: " + str(e)
            self.logger.error(tmp_txt)
            self.logger.error("output file parsed :\n" + output.strip())
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        tmp_txt = ""
        # check potential error first
        if document is None:
            tmp_txt = "parsing uevent msic info contains no information"
            self.logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        # check battery info
        battery_info = document.findtext("BATTERY")
        if battery_info is not None:
            battery_info = str(battery_info).strip().split("\n")
        else:
            tmp_txt += "parsing uevent battery info contains no information."

        # check charger info
        charger_info = document.findtext("CHARGER")
        if charger_info is not None:
            charger_info = str(charger_info).strip().split("\n")
        else:
            tmp_txt += "parsing uevent charger info contains no information."

        if tmp_txt != "":
            self.logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        # get time stamp
        cosmetic_text = ""
        time_stamp = document.findtext("TIME_STAMP")
        if time_stamp is not None:
            time_stamp = str(time_stamp).replace("\n", "").strip()
            msic_register.update({"TIME_STAMP": (str(time_stamp), "none")})
            if log_output:
                cosmetic_text = "[GENERAL info]:\n"
                cosmetic_text += "    TIME_STAMP = " + str(time_stamp) + "\n"
        else:
            msic_register.update({"TIME_STAMP": ("Failed to get", "none")})

        # get charger info
        for element in charger_info:
            # clean the element
            element = element.strip()
            element = element.split("=", 1)
            # if no '=' found skip this element
            if len(element) != 2:
                continue
            tag = element[0].strip().upper()
            value = element[1].strip()

            tag = sub('^POWER_SUPPLY_', '', tag)
            if tag == "CHARGE_VOLTAGE":
                charger["VOLTAGE"] = (float(value) / 1000, "V")
            elif tag in ["MAX_CHARGING_CURRENT", "INPUT_CUR_LIMIT", "MAX_CHARGE_CURRENT"]:
                temp_tag = tag
                if tag == "MAX_CHARGE_CURRENT":
                    temp_tag = 'MAX_CHARGING_CURRENT'
                charger[temp_tag] = (float(value) / 1000, "A")
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
        for element in battery_info:
            # clean the element
            element = element.strip()
            element = element.split("=", 1)
            # if no '=' found skip this element
            if len(element) != 2:
                continue
            tag = element[0].strip().upper()
            value = element[1].strip()
            tag = sub('^POWER_SUPPLY_', '', tag)
            if tag in ["CAPACITY"]:
                battery[tag] = (int(value), "none")
            # voltage tag
            elif tag in ["VOLTAGE_AVG", "VOLTAGE_MIN_DESIGN", "VOLTAGE_NOW", "VOLTAGE_OCV"]:
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
        elif "VOLTAGE_OCV" in battery:
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
            self.logger.debug(cosmetic_text)
            # check vital key on msic_register
        vital_key = ["CAPACITY", "CURRENT_NOW", "VOLTAGE", "STATUS"]

        if "BATTERY" not in msic_register or \
                not set(vital_key).issubset(msic_register["BATTERY"].keys()):
            tmp_txt = "missing vital BATTERY keys on msic registers: %s" % vital_key
            self.logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        return msic_register
