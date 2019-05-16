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
:summary: Utilities class for Energy Management implementation
:author: ssavrimoutou, vgombert
:since: 20/12/2010
"""
from lxml import etree
import os
from shutil import copyfile, move
import time
import numpy

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global, Verdict
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.Hack import open_inheritance_hack

open_inheritance_hack()

CURRENT_UNITS = ["A", "mA"]
VOLTAGE_UNITS = ["V", "mV"]
DEGREE_UNITS = ["DegreeCelsius", "mDegreeCelsius"]
NONE_UNIT = "none"


class EMConstant(object):

    """
    Structure that represents thermal constant used in usecase and uecmd
    """
    EM_INJECT_TAG = "ACS_EM_TEST"
    THRESHOLD_TM_OFF = "THRESHOLD_TM_OFF"
    THRESHOLD_NORMAL = "THRESHOLD_NORMAL"
    THRESHOLD_WARNING = "THRESHOLD_WARNING"
    THRESHOLD_ALERT = "THRESHOLD_ALERT"
    THRESHOLD_CRITICAL = "THRESHOLD_CRITICAL"
    SENSOR_NAME = "SENSOR_NAME"
    TM_OFF = "TM_OFF"
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    ALERT = "ALERT"
    CRITICAL = "CRITICAL"
    UNKNOWN = "UNKNOWN"
    VALUE = "VALUE"
    STATE = "STATE"
    DEGREE_CELSIUS = "DegreeCelsius"
    DTS = "DTS"
    CPU = "CPU"
    BATTERY = "BATTERY"
    GPUSKIN = "GPUSKIN"
    BACKSKIN = "BACKSKIN"
    FRONTSKIN = "FRONTSKIN"
    NUMERIC = "Numeric"
    TIME_TAG = "TIME"
    HR = "HOURS"
    COMMENTS = "COMMENTS"
    REBOOT = "REBOOT"
    GLOBAL_MEAS_FILE = "Global_EM_meas.xml"
    CAPACITY = "CAPACITY"
    VBATT_NOW = "VOLTAGE_NOW"
    IBATT_NOW = "CURRENT_NOW"
    AUTOLOG_TIME = "AUTOLOG_HOURS"
    REBOOT_TIME = "REBOOT_TIME"
    CHAMBER_TAG = "TEMPERATURE_CHAMBER"
    OCV_NOW_LOW_LIM = "OCV_NOW_LOW_LIM"
    OCV_NOW_HIGH_LIM = "OCV_NOW_HIGH_LIM"
    OCV_NOW_LIM_VERDICT = "OCV_NOW_LIM_VERDICT"
    TC_PARAMETER_TO_CHECK = "EM_TARGET"
    TIME_STAMP = "TIME_STAMP"
    VOLTAGE = "VOLTAGE"
    TEMP = "TEMP"
    STATUS = "STATUS"
    BATT_FULL = "FULL"
    BATT_NOT_CHARGING = "NOT CHARGING"
    BATT_CHARGING = "CHARGING"
    BATT_DISCHARGING = "DISCHARGING"


class Measure(object):

    """
    Structure that represents an generic measurement object
    """

    def __init__(self, param_value, param_unit):
        """
        convert to int or float str object otherwise keep the same format
        """
        # by default value is equal to param_value
        self.value = param_value
        if isinstance(param_value, str):
            if param_value.find(".") != -1 and param_value.replace(".", "", 1).isdigit():
                self.value = float(param_value)
            elif param_value.isdigit():
                self.value = int(param_value)

        self.unit = str(param_unit)

    def convert_to(self, unit):
        """
        Convert unit of the measure object to the wanted unit.

        :type unit: str
        :param unit: unit wanted.
        """
        pass

    def get_string_format(self):
        """
        Return the value and unit of the measure object in str format.

        :type unit: str
        :param unit: unit wanted.
        """
        unit = ""
        if self.unit != "none":
            unit = self.unit
        string_value = "%s %s" % (self.value, unit)
        return string_value


class Current(object):

    """
    Structure that represents an object 'current' value and unit (A, mA))
    """

    [A_UNIT, mA_UNIT] = CURRENT_UNITS

    def __init__(self, param_value, param_unit):
        self.value = float(param_value)
        self.unit = str(param_unit)

    def convert_to(self, unit):
        """
        Convert unit of the current to the wanted unit.

        :type unit: str
        :param unit: unit wanted.
        """

        if unit != self.unit:
            if(unit == Current.mA_UNIT
               and self.unit == Current.A_UNIT):
                self.value *= 1000.0
                self.unit = Current.mA_UNIT
            elif(unit == Current.A_UNIT
                 and self.unit == Current.mA_UNIT):
                self.value /= 1000.0
                self.unit = Current.A_UNIT
            else:
                LOGGER_TEST_SCRIPT.error("Current Conversion failed: unknown unit (%s)" % unit)

    def get_string_format(self):
        """
        Return the value and unit of the current in str format.

        :type unit: str
        :param unit: unit wanted.
        """

        if self.unit == Current.mA_UNIT:
            string_value = "%.1f %s" % (self.value, self.unit)
        elif self.unit == Current.A_UNIT:
            string_value = "%.3f %s" % (self.value, self.unit)
        else:
            error_msg = \
                "get Current in string format failed: unknown unit (%s)" \
                % self.unit

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        return string_value


class Voltage(object):

    """
    Structure that represents an object 'voltage' value and unit (V, mV))
    """

    [V_UNIT, mV_UNIT] = VOLTAGE_UNITS

    def __init__(self, volt_value, volt_unit):
        self.value = float(volt_value)
        self.unit = volt_unit

    def convert_to(self, unit):
        """
        Convert unit of the voltage to the wanted unit.

        :type unit: str
        :param unit: unit wanted.
        """

        if unit != self.unit:
            if(unit == Voltage.mV_UNIT
               and self.unit == Voltage.V_UNIT):
                self.value *= 1000.0
                self.unit = Voltage.mV_UNIT
            elif(unit == Voltage.V_UNIT
                 and self.unit == Voltage.mV_UNIT):
                self.value /= 1000.0
                self.unit = Voltage.V_UNIT
            else:
                LOGGER_TEST_SCRIPT.error("Voltage Conversion failed: unknown unit (%s)" % unit)

    def get_string_format(self):
        """
        Return the value and unit of the voltage in str format.

        :type unit: str
        :param unit: unit wanted.
        """

        if self.unit == Voltage.mV_UNIT:
            string_value = "%.1f %s" % (self.value, self.unit)
        elif self.unit == Voltage.V_UNIT:
            string_value = "%.3f %s" % (self.value, self.unit)
        else:
            error_msg = \
                "get Voltage in string format failed: unknown unit (%s)" \
                % self.unit

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        return string_value


class Degree(object):

    """
    Structure that represents an object 'degree' value and unit (Degree, milliDegree))
    """

    (Degree_UNIT, mDegree_UNIT) = DEGREE_UNITS

    def __init__(self, degree_value, degree_unit):
        self.value = float(degree_value)
        self.unit = degree_unit

    def convert_to(self, unit):
        """
        Convert unit of the degree to the wanted unit.

        :type unit: str
        :param unit: unit wanted.
        """

        if unit != self.unit:
            if(unit == Degree. mDegree_UNIT
               and self.unit == Degree.Degree_UNIT):
                self.value *= 1000.0
                self.unit = Degree. mDegree_UNIT
            elif(unit == Degree.Degree_UNIT
                 and self.unit == Degree. mDegree_UNIT):
                self.value /= 1000.0
                self.unit = Degree.Degree_UNIT
            else:
                LOGGER_TEST_SCRIPT.error("Degree Conversion failed: unknown unit (%s)" % unit)

    def get_string_format(self):
        """
        Return the value and unit of the degree in str format.

        :type unit: str
        :param unit: unit wanted.
        """

        if self.unit == Degree. mDegree_UNIT:
            string_value = "%.3f %s" % (self.value, self.unit)
        elif self.unit == Degree.Degree_UNIT:
            string_value = "%.1f %s" % (self.value, self.unit)
        else:
            error_msg = \
                "get Degree in string format failed: unknown unit (%s)" \
                % self.unit

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        return string_value


def update_conf(tmp_dict, fields, value, operation):
    """"
    update energy management parameters value.

    :type tmp_dict: dict
    :param tmp_dict: dict where to search for the keys

    :type fields: str or list
    :param fields: key to change on dict or "all" to change all fields

    :type value: int or str
    :param value: value to affect to given fields

    :type operation: str
    :param operation: operation to apply on the previous value and the submitted value
                        supported operation are * / + - =
    """
    if isinstance(fields, str):
        fields = [fields]

    for field in fields:
        if (isinstance(tmp_dict, dict) and field in tmp_dict) or field == "all":

            if operation == "+":
                tmp_dict.update({field: str(float(tmp_dict[field]) + value)})
            elif operation == "-":
                tmp_dict.update({field: str(float(tmp_dict[field]) - value)})
            elif operation == "*":
                tmp_dict.update({field: str(float(tmp_dict[field]) * value)})
            elif operation == "/":
                tmp_dict.update({field: str(float(tmp_dict[field]) / value)})
            elif operation == "=":
                tmp_dict.update({field: str(value)})
            else:
                LOGGER_TEST_SCRIPT.error("operand %s does not exist: use '+', '-', '*', '/', '='" % operation)
        else:
            LOGGER_TEST_SCRIPT.error("field %s does not exist on %s" % (field, tmp_dict))

#------------------------------------------------------------------------------


class EMMeasurement:

    """
    Energy Management measurement class
    This class will compare all measured parameters, store in a xml document,
    save in data report file, get global verdict of the tests.
    """
    __LOG_TAG = "\t[EM VERDICT MACHINE]\t"
    __ID = "Test_Name"
    __VERDICT = "Status"
    __FAIL_ITER = "Fail_Iteration"
    __PASS_ITER = "Pass_Iteraton"
    __BLOCK_ITER = "Block_Iteration"
    __PASS_RATE = "Pass_Rate"
    __ITER = "Iteration"
    __TCD = "TCD"
    __LOCAL_VERDICT = "EM_verdict_report.xml"
    __FORBIDDEN_TAG = ["", "x", "info", "none"]
    __XML_ROOT = "EnergyManagementData"
    __XML_MAIN = "Measurements"
    __XML_SUB = "Measurement"

    def __init__(self, local_path=".", pass_rate=100):
        """
        Init var and create a xml file for local verdict report.

        :type local_path: str
        :param local_path: path where to store the local verdict file

        :type pass_rate: float
        :param pass_rate: campaign pass_rate
        """
        self.__current_result = None
        self.__data_report_filename = os.path.join(local_path, self.__LOCAL_VERDICT)

        # init xml object
        self.__root_node = etree.Element(self.__XML_ROOT)
        self.__root_node.addprevious(etree.ProcessingInstruction("xml-stylesheet",
                                                                 text="type=\"text/xsl\" href=\"../../Core/Report/html/em.xsl\""))
        self.__xml_document = etree.ElementTree(self.__root_node)
        # __XML_MAIN --> __XML_ROOT
        self.__main_node = etree.SubElement(self.__root_node, self.__XML_MAIN)

        # init verdict value
        self.__iteration = 0
        self.__verdict_list = []
        self.__tested_one_time = False
        self.__limit_file = None
        # __target_from_conf_file is designed to be reset over usecase B2B iteration
        # it help to do measurement and compute iteration of TCD for one usecase execution
        self.__target_from_conf_file = {}
        self.__target_backup = {}
        self.__campaign_pass_rate = pass_rate
        # this var represent the unique test that will generate a verdict.
        self.__tcd_that_participate_to_verdict = None

    def load_target(self, target, target_to_consider=None, consider_all_target=False):
        """
        Load testcase list to enable testcase execution checking and counting.

        :param target: DictionaryList
        :param target: parameter used for comparison

        :param target_to_consider: list
        :param target_to_consider: list of TCD to only consider for verdict generation
                                among all TCDs that cover your test

        :param consider_all_target: Boolean
        :param consider_all_target: if target_to_consider is None then it will use all loaded TCDs in get_current_result_V2 function
        :warning consider_all_target: this a temporary implementation

        :return: None
        """
        self.__target_from_conf_file = {}
        self.__limit_file = target
        for element in target:
            if isinstance(element, dict) and "name" in element and \
                    "description" in element and\
                    "scheduled_time" not in element and \
                    "temperature" not in element:
                self.__target_from_conf_file.update({element["name"]: {"iteration": 0,
                                                                   "pass": 0,
                                                                   "fail": 0,
                                                                   "pass_rate": 0,
                                                                   "comment": "measurement of this value failed to be done (step not reached or missing from DUT)",
                                                                   "status": Verdict.BLOCKED,
                                                                   "testcase_id": element["description"]}})
        self.__target_backup = self.__target_from_conf_file
        # force to consider all TCDs target for verdict reporting
        # this will cause the verdict reporting to be more talkative
        if target_to_consider is None and consider_all_target:
            target_to_consider = self.__target_from_conf_file.keys()

        # evaluate if the tcd you choose it on the target loaded from configuration file
        if type(target_to_consider) is list and len(target_to_consider) > 0 and len(self.__target_from_conf_file) > 0:
            error_msg = ""
            for tcd in target_to_consider:
                if tcd not in self.__target_from_conf_file.keys():
                    error_msg += "%s," % (tcd)

            if error_msg != "":
                error_msg.rstrip(',')
                error_msg = "you asked for %s." % error_msg
                txt = "some of the specific targets that you want to check are wrongs, this testcase configuration can only test the following %s, but " % str(self.__target_from_conf_file.keys())
                txt += error_msg
                txt += "\nNo verdict can be computed without any valid values"
                LOGGER_TEST_SCRIPT.error(self.__LOG_TAG + txt)
                raise AcsConfigException(AcsConfigException.INVALID_TEST_CASE_FILE, txt)
            else:
                self.__tcd_that_participate_to_verdict = target_to_consider

    def __get_em_param_value(self,
                             em_param,
                             attr_param):
        """
        Retrieve the high, low limit target value from built dictionary in the function
        parse_energy_management_targets (see Utilities).

        :type em_param: dictionary
        :param em_param: The dictionary built from parse_energy_management_targets.
        :type attr_param: attr_param
        :param attr_param: The attribute to get from the dictionary.

        :rtype: object (Current, Voltage, Degree, Measure)
        :return: instance that represent your measurement
        """
        if em_param[attr_param] == "none":
            em_param_value = "none"
            em_param_unit = ""
        elif em_param["unit"] == "none":
            em_param_value = em_param[attr_param]
            em_param_unit = ""
        else:
            em_param_value = em_param[attr_param]
            em_param_unit = em_param["unit"]

        if em_param_unit in CURRENT_UNITS:
            em_measure = Current(em_param_value, em_param_unit)

        elif em_param_unit in VOLTAGE_UNITS:
            em_measure = Voltage(em_param_value, em_param_unit)

        elif em_param_unit in DEGREE_UNITS:
            em_measure = Degree(em_param_value, em_param_unit)

        else:
            em_measure = Measure(em_param_value, em_param_unit)

        return em_measure

    def __add_measurement_node(self,
                               attr_name):
        """
        Add a new measurement node each time a new comparison is done

        :rtype: etree.Element
        :return: sub node where to store measurements
        """
        # Create the node <Measurement name="attr_name">
        meas_node = etree.SubElement(self.__main_node, self.__XML_SUB)
        meas_node.set("name", attr_name)
        return meas_node

    def __add_measurement_child_node(self,
                                     meas_node,
                                     node_list):
        """
        Add a new node for each compared measurements
        """
        for node_item in node_list:
            # Create the node with special xml text
            sub_meas_node = etree.SubElement(meas_node, node_item[0])
            sub_meas_node.text = node_item[1]

    def get_global_result(self):
        """
        Get the overall result for current test.
        if all measurements have not finished to be compute
        it will return current verdict value

        :rtype: int
        :return: Global.SUCCESS or Global.FAILURE
        """
        # by default global result is equal to current verdict
        result = self.get_current_result()

        # get the global verdict
        if len(self.__verdict_list) > 0:
            for element in self.__verdict_list:
                if element == Global.FAILURE:
                    result = Global.FAILURE
                    break
                elif element == Global.SUCCESS:
                    result = Global.SUCCESS

        return result

    def reset_current_verdict(self):
        """
        Reset the value of the global result
        """
        self.__tested_one_time = False
        self.__current_result = None
        self.__target_from_conf_file = self.__target_backup

    def __set_current_result(self, result):
        """
        Set the value of the global result

        :type result: int
        :param result: Global.FAILURE or Global.SUCCESS
        """
        # Update the current result only if
        # it is None or Global.SUCCESS
        if (self.__current_result is None) or (self.__current_result == Global.SUCCESS):
            self.__current_result = result

    def get_current_result(self):
        """
        Get the value of the current result.

        :rtype: int
        :return: Global.SUCCESS or Global.FAILURE
        """
        # by default current result is failed
        result = Global.FAILURE
        # if the user want to compute verdict over tcd he/she choose then
        if self.__tcd_that_participate_to_verdict is not None:
            result = self.__get_verdict_depending_of_choosen_tcd()[0]

        # else compute by using the old system
        elif self.__current_result == Global.SUCCESS:
                result = Global.SUCCESS

        return result

    def get_current_result_v2(self):
        """
        Get the value of the current result and an message
        use to return info to acs usecase.

        :rtype: tuple
        :return: (Global.SUCCESS or Global.FAILURE, string message)
        """
        # by default current result is failed
        result = Global.FAILURE
        message = "Check External Tcs detail or your test EM_verdict_report.xml generated for details"
        # if the user want to compute verdict over tcd he/she choose then
        if self.__tcd_that_participate_to_verdict is not None:
            result, message = self.__get_verdict_depending_of_choosen_tcd()

        # else compute by using the old system
        elif self.__current_result == Global.SUCCESS:
                result = Global.SUCCESS

        return result, message

    def __reset_current_result(self, result):  # pylint: disable=W0613
        """
        Reset the value of the current result
        """
        self.__current_result = None

    def judge(self, ignore_blocked_tc=False):
        """
        Set the value of the global result.
        By default Complete the measurement file with testcases that have been declared but not tested.

        :param ignore_blocked_tc: Boolean
        :param ignore_blocked_tc: ignored blocked tc when generating file ,
                                 use this if you do iterative test like fuel gauging or maintenance.

        :rtype: dict
        :return: dict that contains at least the following keys
         - "TotalIteration"
         - "TotalFail"
         - "TotalPass"
         - "PassRate"
         - "LastVerdict"
         - "TestCaseExecutedNumber"
         - "TestCaseExecuted"
         - "TestCaseBlocked"
        """
        tc_number = 0
        tc_executed = ""
        tc_block = ""
        # parse file to check testcase
        if self.__target_from_conf_file not in [None, {}]:
            for key in self.__target_from_conf_file:
                tag_list = self.__target_from_conf_file[key]["testcase_id"].strip()
                # use | to separate several iteration in one name
                tag_list = tag_list.split("|")
                log_done = False
                for tag in tag_list:

                    if tag not in self.__FORBIDDEN_TAG:
                        if self.__target_from_conf_file[key]["iteration"] == 0:
                            if not ignore_blocked_tc:
                                # log only one time below logs
                                if not log_done:
                                    attr_name = str(key)
                                    verdict_msg = Verdict.BLOCKED
                                    error_msg = "Parameter not found in your measurement"
                                    self.__update_data_report(self.__limit_file, attr_name,
                                                              error_msg, verdict_msg)
                                    tc_block += tag + ", "
                                    log_done = True
                                    self.__set_current_result(Global.FAILURE)
                        else:
                            tc_executed += tag + ", "
                            tc_number += 1

            tc_executed = tc_executed.rstrip(", ")
            tc_block = tc_block.rstrip(", ")
            # store test
            self.save_data_report_file()

        if self.__tested_one_time:
            self.__iteration += 1
            self.__verdict_list.append(self.get_current_result())
            self.__tested_one_time = False

        result = self.__get_result_info()
        result.update({"TestCaseExecutedNumber": tc_number})
        result.update({"TestCaseExecuted": tc_executed})
        result.update({"TestCaseBlocked": tc_block})

        return result

    def __get_result_info(self):
        """
        return a dict that contains result info

        :rtype: dict
        :return: dict that contains at least the following keys
         - "TotalIteration"
         - "TotalFail"
         - "TotalPass"
         - "PassRate"
         - "LastVerdict"
        """
        fail_test = 0
        pass_test = 0
        pass_rate = 0
        last_test = "TEST NOT FINISHED"

        for element in self.__verdict_list:
            if element == Global.SUCCESS:
                pass_test += 1
                last_test = "PASS"
            elif element == Global.FAILURE:
                fail_test += 1
                last_test = "FAIL"

        if self.__iteration != 0:
            pass_rate = (pass_test * 100) / self.__iteration
        # Get the global result
        info = {"TotalIteration": self.__iteration,
                "TotalFail": fail_test,
                "TotalPass": pass_test,
                "PassRate": pass_rate,
                "LastVerdict": last_test}

        return info

    def generate_msg_verdict(self):
        """
        Return a human readable text that contains test info.

        :rtype: str
        :return: text that contains the value of all keys generated for current test.
        """
        msg = ""
        verdict = self.__get_result_info()

        for key in verdict:
            if key == "PassRate":
                extra = "% "
            else:
                extra = " "
            msg += key + ":" + str(verdict[key]) + extra
        return msg

    def __compare_string_value(self,
                               read_param,
                               target_param, do_not_store_result=False):
        """
        Save and compare measurement in an xml data report

        :type read_param: tuple
        :param read_param: the measure to compare, read from usecase (value, unit, optional time stamp)

        :type target_param: dict
        :param target_param: parameter used for comparison

        :type do_not_store_result: bool
        :param do_not_store_result: set to true if you want just to compare value
                                    and not editing verdict file

        :rtype: boolean
        :return: True if all values match, False otherwise
        """
        self.__tested_one_time = True
        error_code = Global.FAILURE
        verdict_msg = Verdict.FAIL
        error_msg = "Target failed to be evaluated"

        # cast to upper and clean str
        read_parameter = str(read_param[0]).strip().upper()
        target_parameter = str(target_param["value"]).strip().upper()
        hi_lim = str(target_param["hi_lim"]).strip().upper()
        lo_lim = str(target_param["lo_lim"]).strip().upper()

        if not do_not_store_result:
            # Add a new measurement node with the name of the parameter to check
            meas_node = self.__add_measurement_node(target_param["name"])

            # Add the test number node TNUM
            self.__add_measurement_child_node(meas_node,
                                              [("Test", target_param["description"])])

        if target_parameter not in ["NONE", ""]:
            # case where we got several element in target
            if target_parameter.find("|") != -1:
                is_in_the_list = False
                splited_target = filter(None, target_parameter.split("|"))
                for ele in splited_target:
                    ele = ele.strip()
                    if ele not in ["NONE", ""] and read_parameter == ele:
                        error_code = Global.SUCCESS
                        verdict_msg = Verdict.PASS
                        error_msg = "Read value [%s] is in the possible value  list %s" % (read_parameter, str(splited_target))
                        is_in_the_list = True
                        break
                if not is_in_the_list:
                    error_msg = "Read value [%s] is not in the possible value list %s" % (read_parameter, str(splited_target))

            else:
                # case where we compare directly to target
                if read_parameter == target_parameter:
                    error_code = Global.SUCCESS
                    verdict_msg = Verdict.PASS
                    error_msg = "Read value [%s] as expected" % (read_parameter)
                else:
                    error_msg = "Read value [%s] is different from expected value [%s]" % (read_parameter, target_parameter)
        else:
            # case where limit was set but we read a str from measurement
            if lo_lim != "NONE" or hi_lim != "NONE":
                error_msg = "Value format is different, target expect a numeric, read string [%s]" % (read_parameter)
            else:
                # case where no target are set
                error_code = Global.SUCCESS
                verdict_msg = Verdict.PASS
                error_msg = "no expected value defined, set as PASS by default"

        # Update the global verdict
        result = False
        if not do_not_store_result:
            self.__set_current_result(error_code)
        if error_code == Global.SUCCESS:
            result = True

        # Add following nodes : <Comment>, <Measure>, <LowLimit>,
        # <HighLimit>,<ExpectedValue>, <Verdict> the comment of the result of the test

        if not do_not_store_result:
            if len(read_param) != 3:
                time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S")
            else:
                time_stamp = str(read_param[2])
                # convert time stamp from second to formated time
                if time_stamp.replace(".", "", 1).isdigit():
                    time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S",
                                               time.gmtime(float(time_stamp)))

            self.__add_measurement_child_node(meas_node,
                                              [("Comment", error_msg),
                                               ("Measure", read_parameter),
                                                  ("LowLimit", target_param["lo_lim"]),
                                                  ("HighLimit", target_param["hi_lim"]),
                                                  ("ExpectedValue", target_param["value"]),
                                                  ("Verdict", verdict_msg),
                                                  ("TimeStamp", time_stamp)])

            # Update the global verdict
            self.__set_current_result(error_code)
            name = target_param["name"]
            if self.__target_from_conf_file is not None and name in self.__target_from_conf_file.keys():

                self.__target_from_conf_file[name].update({"iteration":
                                    self.__target_from_conf_file[name]["iteration"] + 1})
                # store the comment to generate verdict message
                self.__target_from_conf_file[name].update({"comment": error_msg})

                if result:
                    self.__target_from_conf_file[name].update({"pass":
                                    self.__target_from_conf_file[name]["pass"] + 1})
                else:
                    self.__target_from_conf_file[name].update({"fail":
                                    self.__target_from_conf_file[name]["fail"] + 1})

                percent = float(self.__target_from_conf_file[name]["pass"] * 100) / float(self.__target_from_conf_file[name]["iteration"])
                self.__target_from_conf_file[name].update({"pass_rate": percent})

                status = Verdict.FAIL
                if percent >= self.__campaign_pass_rate:
                    status = Verdict.PASS
                self.__target_from_conf_file[name].update({"status": status})

        return result

    def __compare_numeric_value(self,
                                meas_param,
                                target_param, do_not_store_result=False):
        """
        compare measurement and save it in an xml data report.
        designed for numeric value.

        :type meas_param: tuple
        :param meas_param: the measure to compare (value, unit, optional time stamp)

        :type target_param: dict
        :param target_param: parameter used for comparison

        :type do_not_store_result: boolean
        :param do_not_store_result: set to true if you want just to compare value
                                    and not editing verdict file

        :rtype: boolean
        :return: True if all values match, False otherwise
        """
        self.__tested_one_time = True
        error_code = Global.SUCCESS
        error_msg = "No errors"
        verdict_msg = Verdict.PASS

        # Get value form meas_param
        meas_param_value = meas_param[0]
        meas_param_unit = meas_param[1]

        # hide none unit
        if meas_param_unit != "none":
            meas_param_str = "%s %s" % (meas_param_value, meas_param_unit)
        else:
            meas_param_str = "%s" % meas_param_value
            meas_param_unit = ""

        high_limit_str = str(target_param["hi_lim"])
        low_limit_str = str(target_param["lo_lim"])
        target_val_str = str(target_param["value"])

        # Add a new measurement node with the name of the parameter to check
        if not do_not_store_result:
            # Check if value is a numeric
            meas_node = self.__add_measurement_node(target_param["name"])
            # Add the test number node TNUM
            self.__add_measurement_child_node(meas_node,
                                              [("Test", target_param["description"])])

        if not self.__is_numeric(meas_param_value):
            error_code = Global.FAILURE
            error_msg = "Problem in read value cast type, read value should be numeric, got %s" % str(meas_param_value)
            verdict_msg = Verdict.FAIL
        else:
            # cast value in float in case it was a str
            meas_param_value = float(meas_param_value)

            # case we have value declared , priority one over low and high lim
            if self.__is_numeric(target_param["value"]):
                target_value = self.__get_em_param_value(target_param, 'value')
                # Align unit of target value with measured unit
                target_value.convert_to(meas_param_unit)
                # Store target str format
                target_val_str = target_value.get_string_format()

                if meas_param_value != target_value.value:
                    error_code = Global.FAILURE
                    verdict_msg = Verdict.FAIL
                    error_msg = "Measured value [%s%s] is different from expected value [%s]" % (meas_param_value, meas_param_unit, target_value.value)
                else:
                    error_msg = "Measured value [%s%s] as expected" % (meas_param_value, meas_param_unit)

            # case  low and high lim declared
            elif self.__is_numeric(target_param["hi_lim"]) and self.__is_numeric(target_param["lo_lim"]):
                high_limit = self.__get_em_param_value(target_param, 'hi_lim')
                low_limit = self.__get_em_param_value(target_param, 'lo_lim')
                # Align current unit of low and high value with measured unit
                low_limit.convert_to(meas_param_unit)
                high_limit.convert_to(meas_param_unit)
                # Store parameter in str format
                high_limit_str = high_limit.get_string_format()
                low_limit_str = low_limit.get_string_format()

                if meas_param_value < low_limit.value or meas_param_value > high_limit.value:
                    error_code = Global.FAILURE
                    error_msg = "Measured value [%s%s] is not in wanted limit range [%s%s, %s%s]" % (meas_param_value, meas_param_unit,
                                                                                                 low_limit.value, low_limit.unit,
                                                                                                 high_limit.value, high_limit.unit)
                    verdict_msg = Verdict.FAIL
                else:
                    error_msg = "Measured value [%s%s] is in wanted limit range [%s%s, %s%s]" % (meas_param_value, meas_param_unit,
                                                                                                 low_limit.value, low_limit.unit,
                                                                                                 high_limit.value, high_limit.unit)
            else:
                # case one lim declared: high
                if self.__is_numeric(target_param["hi_lim"]):
                    high_limit = self.__get_em_param_value(target_param, 'hi_lim')
                    # Align current unit of high value with measured unit
                    high_limit.convert_to(meas_param_unit)
                    # Store parameter in str format
                    high_limit_str = high_limit.get_string_format()

                    if meas_param_value > high_limit.value:
                        error_code = Global.FAILURE
                        error_msg = "Measured value [%s%s] is over high limit [%s%s]" % (meas_param_value, meas_param_unit,
                                                                                         high_limit.value, high_limit.unit)
                        verdict_msg = Verdict.FAIL
                    else:
                        error_msg = "Measured value [%s%s] is under high limit [%s%s]" % (meas_param_value, meas_param_unit,
                                                                                         high_limit.value, high_limit.unit)

                # case one lim declared: low
                elif self.__is_numeric(target_param["lo_lim"]):
                    low_limit = self.__get_em_param_value(target_param, 'lo_lim')
                    # Align current unit of low value with measured unit
                    low_limit.convert_to(meas_param_unit)
                    # Store parameter in str format
                    low_limit_str = low_limit.get_string_format()

                    if meas_param_value < low_limit.value:
                        error_code = Global.FAILURE
                        error_msg = "Measured value [%s%s] is under low limit [%s%s]" % (meas_param_value, meas_param_unit,
                                                                                         low_limit.value, low_limit.unit)
                        verdict_msg = Verdict.FAIL
                    else:
                        error_msg = "Measured value [%s%s] is over low limit [%s%s]" % (meas_param_value, meas_param_unit,
                                                                                        low_limit.value, low_limit.unit)

                else:
                    error_code = Global.SUCCESS
                    error_msg = "No targets defined, set as pass by default"
                    verdict_msg = Verdict.PASS

        # Update the global verdict
        if not do_not_store_result:
            self.__set_current_result(error_code)
        result = False
        if error_code == Global.SUCCESS:
            result = True

        # Add following nodes : <Comment>, <Measure>, <LowLimit>,
        # <HighLimit>,<ExpectedValue>, <Verdict>   the comment of the result of the test

        if not do_not_store_result:
            if len(meas_param) != 3:
                # generate own time stamp
                time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S")
            else:
                time_stamp = str(meas_param[2])
                # convert time stamp from second to formated time
                if self.__is_numeric(time_stamp):
                    time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S",
                                               time.gmtime(float(time_stamp)))

            self.__add_measurement_child_node(meas_node,
                                              [("Comment", error_msg),
                                               ("Measure", meas_param_str),
                                                  ("LowLimit", low_limit_str),
                                                  ("HighLimit", high_limit_str),
                                                  ("ExpectedValue", target_val_str),
                                                  ("Verdict", verdict_msg),
                                                  ("TimeStamp", time_stamp)])

            name = target_param["name"]
            if self.__target_from_conf_file is not None and name in self.__target_from_conf_file.keys():

                self.__target_from_conf_file[name].update({"iteration":
                                                       self.__target_from_conf_file[name]["iteration"] + 1})
                # store the comment to generate verdict message
                self.__target_from_conf_file[name].update({"comment": error_msg})

                if result:
                    self.__target_from_conf_file[name].update({"pass":
                                                           self.__target_from_conf_file[name]["pass"] + 1})
                else:
                    self.__target_from_conf_file[name].update({"fail":
                                                           self.__target_from_conf_file[name]["fail"] + 1})

                percent = float(self.__target_from_conf_file[name]["pass"] * 100) / float(self.__target_from_conf_file[name]["iteration"])
                self.__target_from_conf_file[name].update({"pass_rate": percent})

                status = Verdict.FAIL
                if percent >= self.__campaign_pass_rate:
                    status = Verdict.PASS
                self.__target_from_conf_file[name].update({"status": status})

        return result

    def __update_data_report(self, em_param, attr_name, error_msg, verdict_msg):
        """
        Update reports

        :param em_param: dict
        :param em_param: parameter used for comparison
        :param attr_name: str
        :param attr_name: attribute name
        :param error_code: int
        :param error_code: error code from Gobal.FAILURE or Global.SUCCESS
        :param verdict_msg: str
        :param verdict_msg: verdict message

        """
        # Add a new measurement node with the name of the parameter to check
        meas_node = self.__add_measurement_node(em_param[attr_name]["name"])

        # Add following nodes : <TNUM> <Comment>, <Measure>, <LowLimit>,
        # <HighLimit>,<ExpectedValue>, <Verdict>   the comment of the result of the test
        self.__add_measurement_child_node(meas_node,
                                          [("Test", em_param[attr_name]["description"]),
                                           ("Comment", error_msg),
                                              ("Measure", "none"),
                                              ("LowLimit", em_param[attr_name]["lo_lim"]),
                                              ("HighLimit", em_param[attr_name]["hi_lim"]),
                                              ("ExpectedValue", em_param[attr_name]["value"]),
                                              ("Verdict", verdict_msg)])

    def test_value(self, meas_list, target_element):
        """
        Test values from the target and return true if it match.

        :param meas_list: MeasurementList
        :param meas_list: MeasurementList instance
        :param em_target: dict
        :param em_target: parameter used for comparison

        :rtype: boolean
        :return: True if all values match , False otherwise
        """
        overall_pass = False
        measurement_list = meas_list

        if isinstance(target_element, dict) and \
            "scheduled_time" not in target_element and\
                "name" in target_element:
            attr_name = str(target_element["name"])

            if measurement_list.contain(attr_name):
                measurement = measurement_list.get(attr_name)
                if type(measurement[0]) in [int, float]:
                    overall_pass = self.__compare_numeric_value(measurement,
                                                                target_element, do_not_store_result=True)
                else:
                    overall_pass = self.__compare_string_value(measurement,
                                                               target_element, do_not_store_result=True)

        return overall_pass

    def test_list(self, meas_list, em_target):
        """
        Test if the current measurement list vs em target is passed or failed

        :type meas_list: MeasurementList
        :param meas_list: MeasurementList instance
        :type em_target: DictionaryList
        :param em_target: parameter used for comparison

        :rtype: boolean
        :return: True if all values match, False otherwise
        """
        overall_pass = None

        if meas_list is not None and em_target is not None and not meas_list.is_empty():
            measurement_list = meas_list
            self.__tested_one_time = True
            for target_element in em_target:
                if isinstance(target_element, dict) and "scheduled_time" not in target_element and "name" in target_element:
                    attr_name = str(target_element["name"])
                        # init param
                    if measurement_list.contain(attr_name):
                        if overall_pass is None:
                            overall_pass = True
                        # if it is for test purpose do not remove the element on the list
                        measurement = measurement_list.get(attr_name)
                        one_numeric_found = self.__is_numeric(target_element["value"]) or self.__is_numeric(target_element["lo_lim"]) or self.__is_numeric(target_element["hi_lim"])

                        if one_numeric_found:
                            # case where target is pure numeric
                            overall_pass = self.__compare_numeric_value(measurement,
                                        target_element, do_not_store_result=True) and overall_pass
                        else:
                            # case where target is str
                            overall_pass = self.__compare_string_value(measurement,
                                        target_element, do_not_store_result=True) and overall_pass

        if overall_pass is None:
            overall_pass = False
        return overall_pass

    def compare_list(self, meas_list, em_target, clean_meas_list=False, do_not_store_result=False):
        """
        Save and compare measurement in an xml data report
        also update current verdict.

        :type meas_list: MeasurementList
        :param meas_list: MeasurementList instance
        :type em_target: DictionaryList
        :param em_target: parameter used for comparison
        :type clean_meas_list: boolean
        :param clean_meas_list: Clean meas list after processing
        :type do_not_store_result: boolean
        :param do_not_store_result: just test if the comparison done is True or False
                                    and does not generate file

        :rtype: boolean
        :return: True if all values match, False otherwise
        """
        overall_pass = True

        if meas_list is not None and em_target is not None and not meas_list.is_empty():
            measurement_list = meas_list
            self.__tested_one_time = True
            for target_element in em_target:
                if isinstance(target_element, dict) and \
                    "scheduled_time" not in target_element and\
                        "name" in target_element:
                    attr_name = str(target_element["name"])
                        # init param
                    if measurement_list.contain(attr_name):
                        # if it is for test purpose do not remove the element on the list
                        if not do_not_store_result:
                            measurement = measurement_list.pop(attr_name)
                        else:
                            measurement = measurement_list.get(attr_name)

                        one_numeric_found = self.__is_numeric(target_element["value"]) or\
                            self.__is_numeric(target_element["lo_lim"]) or\
                            self.__is_numeric(target_element["hi_lim"])

                        if one_numeric_found:
                            # case where target is pure numeric
                            potential_result = self.__compare_numeric_value(measurement,
                                                                        target_element, do_not_store_result) and overall_pass
                        else:
                            # case where target is str
                            potential_result = self.__compare_string_value(measurement,
                                                                       target_element, do_not_store_result) and overall_pass
                        # consider only tcd that participate to verdict for this function return value
                        if self.__tcd_that_participate_to_verdict is not None:
                            if len(self.__tcd_that_participate_to_verdict) > 0 and attr_name in self.__tcd_that_participate_to_verdict:
                                overall_pass = potential_result
                        else:
                            overall_pass = potential_result
            if not do_not_store_result:
                self.save_data_report_file()
        else:
            overall_pass = False

        # empty list measurement if option set to true
        if clean_meas_list and isinstance(meas_list, MeasurementList):
            meas_list.clean()
        return overall_pass

    def save_data_report_file(self):
        """
        Save the xml document that contains all data reports in a xml file and return the file name
        """
        # save local verdict file
        # Create xml structure in tempfile to avoid corruption
        tmp_path = self.__data_report_filename + ".tmp"
        try:
            with open(tmp_path, "w") as xml_file:
                self.__xml_document.write(xml_file, pretty_print=True)
            time.sleep(1)
            if os.path.getsize(tmp_path) > 0:
                move(tmp_path, self.__data_report_filename)
        # add workarround against popen filesystem lock
        except Exception as e:
            LOGGER_TEST_SCRIPT.error(self.__LOG_TAG + "XMLMeasurementFile: failed to write file '%s'" % str(e))
            LOGGER_TEST_SCRIPT.error(self.__LOG_TAG + "temporary file path : %s" % str(tmp_path))
            LOGGER_TEST_SCRIPT.error(self.__LOG_TAG + "final file path : %s" % str(self.__data_report_filename))
        # delete temp file if it exist
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception as e:  # pylint: disable=W0703
                LOGGER_TEST_SCRIPT.debug(self.__LOG_TAG + "XMLMeasurementFile: failed to remove tempfile '%s'" % str(e))

        return self.__data_report_filename

    # sub function to avoid long line code
    def __is_numeric(self, text):
        """
        internal function to check if a value is numeric

        :type text: str
        :param text: text
        """
        try:
            float(str(text))
            return True
        except ValueError:
            return False

    def get_local_result_v2(self):
        """
        return a dictonary used to fill acs secondary reports.
        To be used with SecondaryTestReport.py
        if there was specified tcd to evaluate settled by the user, they will
        be the only one to be shown.

        :rtype: dict
        :return: a dict with the following shape:
                        result = {key_tag: {
                                'status': Verdict.FAIL,
                                'comment': "your text"}
        """
        result = {}
        for key_tag in self.__target_from_conf_file.keys():
            tcd = self.__target_from_conf_file[key_tag]

            # if vital key is not found switch to next one
            if not set(["iteration", "pass", "fail", "pass_rate", "testcase_id", "comment"]).issubset(tcd.keys()):
                continue

            # for all tcd that are not wanted and fail, log a block
            if self.__tcd_that_participate_to_verdict is not None:
                    if key_tag in self.__tcd_that_participate_to_verdict:
                        result[key_tag] = {}
                        result[key_tag]["status"] = tcd["status"]
                        result[key_tag]['comment'] = tcd["comment"]
            else:
                result[key_tag] = {}
                result[key_tag]["status"] = tcd["status"]
                result[key_tag]['comment'] = tcd["comment"]

        return result

    def get_local_result(self):
        """
        return a dictonary used to fill acs secondary reports.
        To be used with SecondaryTestReport.py

        :rtype: dict
        :return: a dict with the following shape:
                        result = {key_tag: {
                                'status': Verdict.FAIL,
                                'comment': "your text"}
        """
        result = {}
        for key_tag in self.__local_target_file.keys():
            tcd = self.__local_target_file[key_tag]

            # if vital key is not found switch to next one
            if not set(["iteration", "pass", "fail", "pass_rate", "testcase_id"]).issubset(tcd.keys()):
                continue

            # create TCD entry on dict
            tcd_ids = tcd["testcase_id"]
            # split by | in case or 2 tcd for the same checking
            for tcd_id in tcd_ids.split("|"):
                tcd_id = tcd_id.strip()
                # filter forbidden tag
                if tcd_id.lower() not in self.__FORBIDDEN_TAG:
                    # check that tcd is not already existing
                    if tcd_id not in result:
                        result[tcd_id] = {}
                        result[tcd_id]["status"] = tcd["status"]
                        comment = ""
                        for text in ["iteration", "pass", "fail", "pass_rate"]:
                            comment += text + ":" + str(tcd[text]) + " \t "
                        result[tcd_id]['comment'] = comment

        return result

    def __get_verdict_depending_of_choosen_tcd(self):
        """
        compute a verdict depending of all measurements read and
        what user want to check on these measurement.

        :rtype: String
        :return: test verdict between PASS, FAIL, BLOCKED
        """
        last_verdict = None
        message = ""

        # parse all generated result and construct a list of what user want to test and their result
        for tcd_wanted_by_user in self.__tcd_that_participate_to_verdict:
            status = Verdict.BLOCKED
            reason = "test failed to be measured"
            if  tcd_wanted_by_user in self.__target_from_conf_file.keys():
                tcd_status = self.__target_from_conf_file[tcd_wanted_by_user].get("status")
                if tcd_status is not None:
                    status = tcd_status

                tcd_reason = self.__target_from_conf_file[tcd_wanted_by_user].get("comment")
                if tcd_reason is not None:
                    reason = tcd_reason

            message += "test of %s is %s, reason : %s; " % (tcd_wanted_by_user, status, reason)
            # if it has never been initialized, set to status
            if last_verdict is None:
                last_verdict = status
            # if we was pass and a we meet a block or fail , set to fail
            elif last_verdict == Verdict.PASS and status in [Verdict.BLOCKED, Verdict.FAIL]:
                last_verdict = Verdict.FAIL
            # if we was block or we meet a fail or pass, set to fail
            elif last_verdict == Verdict.BLOCKED and status in [Verdict.FAIL, Verdict.PASS]:
                last_verdict = Verdict.FAIL

        # convert verdict in uc verdict
        if last_verdict is None:
            last_verdict = Global.BLOCKED
            message = "not test evaluated"
        elif last_verdict == Verdict.FAIL:
            last_verdict = Global.FAILURE
        elif last_verdict == Verdict.BLOCKED:
            last_verdict = Global.BLOCKED
        elif last_verdict == Verdict.PASS:
            last_verdict = Global.SUCCESS

        return last_verdict, message


#------------------------------------------------------------------------------


class XMLMeasurementFile:

    """
    Energy Management xml file that generate a list of measurement.
    """
    __ROOT = "EnergyManagementMeasurement"
    __MAIN_NODE = "Measurements"
    __SUB_NODE = "Measure"

    def __init__(self, file_name, root_directory="."):
        """
        Init class
        open a new line for measurement.
        """
        self.__data_report_filename = os.path.join(root_directory, file_name)
        # init xml object
        self.__root_node = etree.Element(self.__ROOT)
        self.__xml_document = etree.ElementTree(self.__root_node)
        # __MAIN_NODE --> __ROOT
        self.__main_node = etree.SubElement(self.__root_node, self.__MAIN_NODE)
        # _SUB_NODE --> __MAIN_NODE
        self.__current_node = etree.Element(self.__SUB_NODE)
        # list of dict that represent your current measurement
        self.__measurement_object = []
        self.__current_dict = {}
        self.__global_meas_object = None

    def get_report_path(self):
        """
        return the path of xml file
        """
        return self.__data_report_filename

    def enable_global_meas(self, file_path, test_name):
        """
        Activate global measurement stocking for current test.

        :type file_path: str
        :param param: file path to store global measurement

        :type test_name: str
        :param test_name: name of the test to add
        """
        self.__global_meas_object = XMLGlobalMeasurementFile(file_path)
        self.__global_meas_object.add_usecase_node(test_name)

    def add_dict_measurement(self, dictionary):
        """
        Update current measurement line with informations from a dict.
        Each element of the dict must finished with a tuple (value, unit) or a single element if type of int, float or str

        :type dictionary: dict
        :param dictionary: formated dict, dict depth can vary as long as it finished by tuple
        """
        def __recursive(dico, name):
            """
            local function
            """
            for element in sorted(dico.keys()):
                obj = dico[element]
                if name != "":
                    sub_name = name + "_" + str(element)
                else:
                    sub_name = element
                if isinstance(obj, tuple) and len(obj) >= 2:
                    # get the unit
                    if str(obj[1]).upper() not in ["NONE", ""]:
                        sub_name += "__" + str(obj[1])

                    # add robustness to avoid space in atribute name
                    sub_name = sub_name.replace(" ", "_")
                    # set attribute
                    self.__current_node.set(sub_name, str(obj[0]))
                    self.__current_dict.update({sub_name: obj})
                elif isinstance(obj, dict):
                    __recursive(obj, sub_name)
                # add case where dico does not have a tuple but only unit
                elif type(obj) in [str, int, float]:
                    # add robustness to avoid space in atribute name
                    sub_name = sub_name.replace(" ", "_")
                    # set attribute
                    self.__current_node.set(sub_name, str(obj))
                    self.__current_dict.update({sub_name: (obj, "")})

        __recursive(dictionary, "")

    def add_computed_meas(self, element_list):
        """
        add computed measurement to current meas object.
        the appending is started after the last found measurement with same key name.
        if th elemnt does not exist then a new entry will be create.

        :type element_list: tuple (str , int/str/float, optional unit)
        :param element_list: list of measurement to add  ("key", value_of_key)
        """
        if not self.__measurement_object:
            self.__measurement_object.append({})

        for sub_element in element_list:
            added = False
            if len(sub_element) >= 2:
                name = sub_element[0]
                value = sub_element[1]
                unit = ""
                # get unit from the third value
                if len(sub_element) >= 3:
                    unit = sub_element[2]
                    if unit.upper() not in ["NONE", ""]:
                        name += "__" + unit
                # else if __ is present on the name it meas the element at is right is the unit
                elif name.find("__") != -1:
                    unit = name.split("__")[-1]
                else:
                    # test if value is a real numeric value
                    try:
                        float(str(value))
                        unit = EMConstant.NUMERIC
                        name += "__" + unit
                    except ValueError:
                        pass
                obj = (value, unit)
            else:
                obj = None

            # case where new element are added before the end of this list
            if obj is not None:
                for element in self.__measurement_object:
                    if name not in element:
                        element.update({name: obj})
                        added = True
                        break

                if not added:
                    # case where a new entry must be created
                    self.__measurement_object.append({name: obj})

    def remove_meas(self, name):
        """
        Remove all computed measurement with given "name" to current meas object.
        If name is not found, it will try to find "name" with unit attach on it.
        name can be an str or a list of str.

        Useful if you want to compute measurement over B2B result and
        overwrite their values since the beginning

        :type name: str or list of str
        :param name: measurement name to remove
        """
        for element in self.__measurement_object:
            if isinstance(element, dict):
                # case where name is at least an str
                if not isinstance(name, list):
                # try to get the full name as key
                    if name in element:
                        element.pop(name)
                    else:
                        # if full name is not found try to find out if there an unit attach to this name
                        for key in element.keys():
                            if name in key.split("__"):
                                element.pop(key)
                                break
                else:
                # case where name is a list of str
                    for sub_name in name:
                        if sub_name in element:
                        # try to get the full name as key
                            element.pop(sub_name)
                        else:
                            # if full name is not found try to find out if there is an unit attach to this name
                            for key in element.keys():
                                if sub_name in key.split("__"):
                                    element.pop(key)
                                    break

    def add_measurement(self, element_list):
        """
        Add new measurements to current line.

        :type element_list: tuple (str , object, optional unit)
        :param element_list: list of measurement to add  ("key", value_of_key)
        """
        for element in element_list:
            if len(element) >= 2:
                name = element[0]
                value = element[1]
                unit = ""
                if len(element) >= 3:
                    unit = element[2]
                    name += "__" + unit
                # else if __ is present on the name it mean the element at is right is the unit
                elif name.find("__") != -1:
                    unit = name.split("__")[-1]
                else:
                    # test if value is a real numeric value
                    try:
                        # convert to str to avoid casting boolean as Numeric
                        float(str(value))
                        unit = EMConstant.NUMERIC
                        name += "__" + unit
                    except ValueError:
                        pass

                obj = (value, unit)
                self.__current_node.set(name, str(value))
                self.__current_dict.update({name: obj})

    def save_data_report_file(self):
        """
        Save the xml document that contains all data reports in a xml file and return the file name

        :rtype: str
        :return: report name
        """
        # Create xml structure in tempfile to avoid corruption
        tmp_path = self.__data_report_filename + ".tmp"
        try:
            with open(tmp_path, "w") as xml_file:
                self.__xml_document.write(xml_file, pretty_print=True)
            time.sleep(1)
            if os.path.getsize(tmp_path) > 0:
                move(tmp_path, self.__data_report_filename)
        except Exception as e:
            LOGGER_TEST_SCRIPT.error("XMLMeasurementFile: failed to write file '%s'" % str(e))
        # delete temp file if it exist
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception as e:  # pylint: disable=W0703
                LOGGER_TEST_SCRIPT.debug("XMLMeasurementFile: failed to remove tempfile '%s'" % str(e))

        return self.__data_report_filename

    def get_overall_measurement(self):
        """
        return all the measurement for current test as a list.
        each list element contains a dictionary where  dict[name_unit] = (value, unit)
        note that you need to sink measurements through function switch_to_next_meas first
        otherwise it will return a empty list of dict

        :rtype: list[dict]
        :return: list of dictionary
        """
        return self.__measurement_object

    def switch_to_next_meas(self):
        """
        Save the xml document and switch to next measurement line if the current line is not empty.
        """
        # sink measurement only if there is something to sink.
        if len(self.__current_node.keys()) > 0:
            self.__main_node.append(self.__current_node)
            # create a new node only if current one is not empty
            self.__current_node = etree.Element(self.__SUB_NODE)
            self.__measurement_object.append(self.__current_dict)
            self.__current_dict = {}

            # save xml structure as file
            with open(self.__data_report_filename, "w") as xml_file:
                self.__xml_document.write(xml_file, pretty_print=True)

    def generate_global_file(self):
        """
        compute all value on xml measurement and added them to the global measurment File

        :rtype: str
        :return: path of generated file or ""
        """
        path = ""
        if self.__global_meas_object is not None:
            self.__global_meas_object.add_measurement(self.__measurement_object)
            path = self.__global_meas_object.save_data_report_file()

        return path

    def reset_global_file(self):
        """
        reset global file to it init state
        """
        if self.__global_meas_object is not None:
            self.__global_meas_object.reset_document()

#------------------------------------------------------------------------------


class XMLGlobalMeasurementFile:

    """
    Energy Management xml file that generate a list of measurements for AWR.
    This class is designed to be use for graph generation.
    """

    __REPORT_TAG = "EMResults"
    __USECASE_TAG = "Testcase"
    __MEAS_TAG = "Measure"

    def __init__(self, file_name, root_directory="."):
        """
        Init class.

        :type file_name: str
        :param file_name: the file where to store generated info

        :type root_directory: str
        :param root_directory: root directory where to store the report
        """
        self.__data_report_filename = os.path.join(root_directory, file_name)
        # create a temporary file path
        new_file = True
        self.__order = 0

        # data report file already exist, load it
        if os.path.exists(self.__data_report_filename):
            # load file and get main NODE
            try:
                parser = etree.XMLParser(remove_blank_text=True)
                self.__xml_document = etree.parse(self.__data_report_filename, parser)
                self.__root_node = self.__xml_document.getroot()
                if self.__root_node is not None:
                    # get the order to sort TC
                    for elt in self.__root_node.findall(self.__USECASE_TAG):
                        if 'order' in elt.attrib:
                            order = int(elt.get('order'))
                            if self.__order < order:
                                self.__order = order
                    new_file = False
            # if file is corrupted, copy it somewhere and generate new one
            except etree.XMLSyntaxError:
                LOGGER_TEST_SCRIPT.error("XMLGlobalMeasurementFile:Corrupted file %s, reset the file" %
                                               self.__data_report_filename)
                copyfile(self.__data_report_filename, self.__data_report_filename + ".corrupted")

        if new_file:
        # else create it
            # init xml document
            self.__root_node = etree.Element(self.__REPORT_TAG)
            self.__xml_document = etree.ElementTree(self.__root_node)
            # then save it
            with open(self.__data_report_filename, "w") as xml_file:
                self.__xml_document.write(xml_file, pretty_print=True)

        # init var
        self.__retrieved_measurement = [{}]
        self.__current_node = None
        self.__main_node = None
        self.__usecase_name = ""

    def add_usecase_node(self, usecase_name):
        """
        Add new entry for a usecase.
        Opening and initializing xml node.

        :type usecase_name: str
        :param usecase_name: usecase name
        """
        self.__usecase_name = usecase_name
        self.__init_node()

    def __init_node(self):
        """
        init node values
        """
        # create xml connection
        # __USECASE_TAG --> __REPORT_TAG
        self.__main_node = etree.SubElement(self.__root_node, self.__USECASE_TAG,
                                            id=self.__usecase_name,
                                            order=str(self.__order + 1))
        # __MEAS_TAG --> __USECASE_TAG
        self.__current_node = etree.Element(self.__MEAS_TAG)

    def add_measurement(self, meas_list):
        """
        Add new measurements to current line.
        only numeric value will be kept.

        :type meas_list: list
        :param meas_list: measurement list generated by XMLMeasurementFile Class
        """
        measurement_added = False
        for dico in meas_list:
            if isinstance(dico, dict):
                for key in sorted(dico.keys()):
                    (value, unit) = dico[key]
                    # test if value is a real numeric value
                    try:
                        float(value)
                        if unit != "":
                            self.__current_node.set(key, str(value))
                            measurement_added = True
                    except (ValueError, TypeError):
                        continue

            if measurement_added:
                self.__main_node.append(self.__current_node)
                self.__current_node = etree.Element(self.__MEAS_TAG)

        if measurement_added:
            self.__retrieved_measurement = list

    def save_data_report_file(self):
        """
        Save the xml document that contains all data
        reports in a xml file and return the file name

        :rtype: str
        :return: report name
        """
        # Create xml structure in tempfile to avoid corruption
        tmp_path = self.__data_report_filename + ".tmp"
        try:
            with open(tmp_path, "w") as xml_file:
                self.__xml_document.write(xml_file, pretty_print=True)
            time.sleep(1)
            if os.path.getsize(tmp_path) > 0:
                move(tmp_path, self.__data_report_filename)
        except Exception as e:
            LOGGER_TEST_SCRIPT.error("XMLMeasurementFile: failed to write file '%s'" % str(e))
        # delete temp file if it exist
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception as e:  # pylint: disable=W0703
                LOGGER_TEST_SCRIPT.debug("XMLMeasurementFile: failed to remove tempfile '%s'" % str(e))

        return self.__data_report_filename

    def reset_document(self):
        """
        move the document pointer back to this doc __init__ state
        in order to overwrite doc in B2B test
        """
        if self.__main_node is not None:
            self.__main_node.clear()
            self.__main_node.set("id", self.__usecase_name)
            self.__main_node.set("order", str(self.__order + 1))
            # __MEAS_TAG --> __USECASE_TAG
            self.__current_node = etree.Element(self.__MEAS_TAG)

#------------------------------------------------------------------------------


class MeasurementList:

    """
    This class is used to store measurement.
    each measurment is a tuple like following : (unit , value , timestamp)
    associate to a tag.
    each tag can appear only 1 time in the measurement list, if
    you inject twice a tag , this one will overide the existing one
    """

    def __init__(self):
        """
        init the list
        """
        self.__meas_list = []

    def add(self, tag, *args):
        """
        Add measurement to your list.

        :type tag: str
        :param tag: tag related to your measurement
                    it will help to find it in energy_management.xml file

        :type args: tuple
        :param args: args can be a tuple (value, unit ) or equal to ,value, unit) together
                     an extra element can be add to set a timestamp for this measurement
                     like (value, unit, time_stamp ) or equal to ,value, unit)
                     see below for details

        :type value: int, str or float
        :param value: value to store

        :type unit: str
        :param unit: unit of your measurement

        :type time_stamp: int or str
        :param time_stamp: time stamp for your measurement
        """
        value_to_add = None
        if isinstance(args, tuple):
            # case   value, unit, timestamp)
            if len(args) >= 2:
                # check if time stamp is used
                if len(args) == 3:
                    time_stamp = args[2]
                else:
                    # else auto set a timestamp
                    time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S")
                value_to_add = (args[0], args[1], time_stamp)

            # case   (value, unit), timestamp)
            elif len(args) >= 1 and \
                    isinstance(args[0], tuple) and len(args[0]) == 2:
                # check if time stamp is used
                if len(args) == 3:
                    time_stamp = args[1]
                else:
                    # else auto set a timestamp in formated way
                    time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S")

                value_to_add = (args[0][0], args[0][1], time_stamp)
        else:
            error_msg = "Error in your measurement format for TAG '%s' = %s" % (tag, str(args))
            raise ValueError(error_msg)
        # check that the info is not already here
        if self.contain(tag):
            self.remove(tag)
        self.__meas_list.append({tag: value_to_add})

    def add_dict(self, tag, dictionary, time_stamp=None):
        """
        Add measurement from a dict to your list.
        the dict must have at most 2 levels ([x][y]) and finish by a tuple
        Non tuple elements will be ignored.
        the tag generated for each element will be determinated dEpending of the dict depth , e.i "tag.x.y"

        :type tag: str
        :param tag: tag related to your measurement
                    it will help to find it in energy_management.xml file

        :type dictionary: dict
        :param dictionary: dict where the last element finish with a tuple
                            e.i [BATTERY][CURRENT_NOW] = (value, unit)

        :type: str or int
        :param time_stamp: time stamp for this dict
        """
        if isinstance(dictionary, dict):
            # iter on level 1
            for category in dictionary.keys():
                full_tag_name = tag + "." + category.upper()
                dict_info = dictionary[category]

                # iter on level 2 if level 1 is not measurement
                if isinstance(dict_info, dict):
                    for element in dict_info:
                        info = dict_info[element]
                        # parse level 2
                        if isinstance(info, tuple) and len(info) == 2:
                            final_tag = full_tag_name + "." + element.upper()
                            if time_stamp is None:
                                time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S")
                            info_to_store = (info[0], info[1], time_stamp)
                            # check that the info is not already here
                            if self.contain(final_tag):
                                self.remove(final_tag)
                            self.__meas_list.append({final_tag: info_to_store})

                # parse level 1
                elif isinstance(dict_info, tuple) and len(dict_info) == 2:
                    if time_stamp is None:
                        time_stamp = time.strftime("%Y-%m-%d_%Hh%M.%S")
                    info_to_store = (dict_info[0], dict_info[1], time_stamp)
                    # check that the info is not already here
                    if self.contain(full_tag_name):
                        self.remove(full_tag_name)
                    self.__meas_list.append({full_tag_name: info_to_store})

    def remove(self, tag):
        """
        Remove a measurement from the list

        :type tag: str
        :param tag: tag related to your measurement
        """
        for element in self.__meas_list:
            if isinstance(element, dict) and tag in element:
                self.__meas_list.remove(element)
                break

    def pop(self, tag):
        """
        Get the measurement value and remove it from the list.

        :type tag: str
        :param tag: tag related to your measurement

        :rtype: tuple
        :return:  (value depending the storing format, unit as str)
                if tag not found ("value not found", "none") is returned
        """
        result = ("value not found", "none")
        for element in self.__meas_list:
            if isinstance(element, dict) and tag in element:
                result = element[tag]
                self.__meas_list.remove(element)
                break

        return result

    def get(self, tag):
        """
        Get the measurement value

        :type tag: str
        :param tag: tag related to your measurement

        :rtype: tuple
        :return:  (value depending the storing format, unit as str)
                if tag not found ("value not found", "none") is returned
        """
        result = ("value not found", "none")
        for element in self.__meas_list:
            if isinstance(element, dict) and tag in element:
                result = element[tag]
                break

        return result

    def contain(self, tag):
        """
        Check if measurement list contains given tag

        :type tag: str
        :param tag: tag related to your measurement
                    it will help to find it in energy_management.xml file

        :rtype: boolean
        :return: True if found , False otherwise
        """
        for element in self.__meas_list:
            if isinstance(element, dict) and tag in element:
                return True

        return False

    def to_string(self):
        """
        return a str containing the measurement list elements.

        :rtype: str
        :return: a str containing the list element.
        """
        output = ""
        for element in self.__meas_list:
            if isinstance(element, dict):
                for sub_element in element:
                    if len(element[sub_element]) == 2:
                        output += sub_element + " = " + \
                            str(element[sub_element][0]) + " " + \
                            str(element[sub_element][1]) + "\n"

        return output

    def is_empty(self):
        """
        test if measure list is empty.

        :rtype: boolean
        :return: True if empty, False Otherwise
        """
        result = True
        if len(self.__meas_list) != 0:
            result = False

        return result

    def clean(self):
        """
        Clean the list
        """
        self.__meas_list = []

    def pretty_print(self):
        """
        return a printable message that contains this list element
        """
        msg = "your measurement list contains %s element(s)\n" % str(len(self.__meas_list))
        msg += "TAG\t=\tVALUE\n"
        for element in self.__meas_list:
            if type(element) is dict:
                for key in element.keys():
                    msg += "%s\t=\t%s\n" % (str(key), str(element.get(key)))

        return msg


#------------------------OBSOLET TOOL----------------------------------------------


class OcvComputingTool:

    """
    Open Circuit Voltage computing tools
    This class allow you to calculate OCV for your usecase
    .. warning:: this class does not manage the unit conversion, by default unit are in V.
    """
    OCV_AVG = "OCV_AVG"
    OCV_STD = "OCV_STD"
    OCV_LOW_LIM = "OCV_AVG_LOW_LIM"
    OCV_HIGH_LIM = "OCV_AVG_HIGH_LIM"
    OCV_IN_LIMIT = "OCV_AVG_LIMIT_TEST"
    SOC = "SOC"
    UNIT = "V"

    def __init__(self, dico_target):
        """
        init class

        :type dico_target: dict
        :param dico_target: ocv target limits get from
                    UtilitiesFWK.Utilities.ConfigsParser.parse_em_ocv_targets
        """
        # init ocv list
        self.__ocv_dict = {}
        if isinstance(dico_target, dict) and len(dico_target) > 0:
            self.__target_dict = dico_target
        else:
            self.__target_dict = None
            LOGGER_TEST_SCRIPT.error("OcvComputingTool: failed to load OCV target limits")

    def add(self, soc, ocv):
        """
        add a value to ocv dictionary

        :type soc: int
        :param soc: state of charge

        :type ocv: float
        :param ocv: open circuit voltage

        :rtype: boolean
        :return: True if ocv is good, false otherwise
        """
        # if soc is present, add it to the current list
        if soc in self.__ocv_dict:
            self.__ocv_dict[soc].append(ocv)
        # create a new entry for this soc
        else:
            self.__ocv_dict.update({soc: [ocv]})

        return self.evaluate_ocv(soc, ocv)

    def compute_as_meas(self):
        """
        Compute the ocv, computing its average for each soc.
        Result will be returned as a list of tuple list to match with the XMLMeasurementFile
        requirement in term of measurement storage.

        :rtype: list[list[(tuple),(tuple)...]]
        :return: list of tuple list
        .. todo:: put code to also consider simple tuple
        """
        result = []
        # sort the dict from higher to lower value
        for soc in reversed(sorted(self.__ocv_dict)):
            ocv_list = self.__ocv_dict[soc]
            list_to_add = []
            if len(ocv_list) > 0:
                ocv_avg = numpy.average(ocv_list)
                ocv_std = numpy.std(ocv_list)
                list_to_add = [(self.OCV_AVG, ocv_avg, self.UNIT),
                               (self.SOC, soc, "none"),
                               (self.OCV_STD, ocv_std, self.UNIT)]

                # compute meas
                if isinstance(self.__target_dict, dict):
                    for lo_soc, hi_soc in self.__target_dict.keys():
                        if lo_soc <= soc <= hi_soc:
                            dico = self.__target_dict[(lo_soc, hi_soc)]
                            # get low ocv limit
                            lo_intercept = dico["lo_intercept"]
                            lo_slope = dico["lo_slope"]
                            if None not in [lo_intercept, lo_slope]:
                                lo_ocv = float(soc) * lo_slope + lo_intercept
                                list_to_add.append((self.OCV_LOW_LIM, lo_ocv, self.UNIT))

                            # get high ocv limit
                            hi_intercept = dico["hi_intercept"]
                            hi_slope = dico["hi_slope"]
                            if None not in [hi_intercept, hi_slope]:
                                hi_ocv = float(soc) * hi_slope + hi_intercept
                                list_to_add.append((self.OCV_HIGH_LIM, hi_ocv, self.UNIT))
                            ocv_ok = 0
                            if lo_ocv <= ocv_avg <= hi_ocv:
                                ocv_ok = 1
                            list_to_add.append((self.OCV_IN_LIMIT, ocv_ok, self.UNIT))
                            break

                # append element in result
                result.append(list_to_add)

        return result

    def evaluate_ocv(self, soc, ocv):
        """
        Evaluate that the given ocv
        is in the ocv target limits.

        :rtype: boolean
        :return: True if ocv is good, false otherwise
        """
        result = False
        if isinstance(self.__target_dict, dict):
            for lo_soc, hi_soc in self.__target_dict.keys():
                if lo_soc <= soc <= hi_soc:
                    dico = self.__target_dict[(lo_soc, hi_soc)]
                    # get low ocv limit
                    lo_intercept = dico["lo_intercept"]
                    lo_slope = dico["lo_slope"]
                    if None not in [lo_intercept, lo_slope]:
                        lo_ocv = float(soc) * lo_slope + lo_intercept

                    # get high ocv limit
                    hi_intercept = dico["hi_intercept"]
                    hi_slope = dico["hi_slope"]
                    if None not in [hi_intercept, hi_slope]:
                        hi_ocv = float(soc) * hi_slope + hi_intercept

                    if lo_ocv <= ocv <= hi_ocv:
                        result = True
                    break

        return result

    def get_ocv_limit(self, soc):
        """
        Return OCV low and high limits for given SOC.

        :rtype: tuple(float, float)
        :return: returning (ocv low limits, ocv high limits)
        """
        result = (None, None)
        if isinstance(self.__target_dict, dict):
            for lo_soc, hi_soc in self.__target_dict.keys():
                if lo_soc <= soc <= hi_soc:
                    dico = self.__target_dict[(lo_soc, hi_soc)]
                    # get low ocv limit
                    lo_intercept = dico["lo_intercept"]
                    lo_slope = dico["lo_slope"]
                    if None not in [lo_intercept, lo_slope]:
                        lo_ocv = float(soc) * lo_slope + lo_intercept

                    # get high ocv limit
                    hi_intercept = dico["hi_intercept"]
                    hi_slope = dico["hi_slope"]
                    if None not in [hi_intercept, hi_slope]:
                        hi_ocv = float(soc) * hi_slope + hi_intercept

                    result = (lo_ocv, hi_ocv)

        return result

    def get_constant_list(self):
        """
        Return a list containing all the constants used by this object

        :rtype: list
        :return: list of this class constant
        """
        return [self.OCV_AVG, self.OCV_STD, self.OCV_LOW_LIM,
                self.OCV_HIGH_LIM, self.SOC, self.OCV_IN_LIMIT]
