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
:summary: Implements power measurement methods used over the whole ACS solution
:since: 20/03/2010
:author: cbresoli
"""

import os
import numpy
import scipy
import shutil
import tempfile
from datetime import datetime
from lxml import etree
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import Verdict
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.PathManager import Paths
from Core.Report.ACSLogging import LOGGER_FWK
from Core.Report.Live.LiveReporting import LiveReporting
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from Device.DeviceManager import DeviceManager
from ErrorHandling.TestEquipmentException import TestEquipmentException


class PnPResults:
    """
    Generation of the pnpresults.xml file
    """
    __logger = LOGGER_FWK
    def __init__(self, report_tree, device_model,
                 failure_file, target_file,
                 tcattributes,iteration=False):
        """
        Constructor
        """
        self.iteration = iteration
        self.__device_model = device_model
        self.__pnptargets = PnPTargets(failure_file, target_file)

        stylesheet = "power.xsl"
        self.__filename = os.path.join(report_tree.get_report_path(),
                                       "pnpresults.xml")

        # Load file if exists
        if os.path.exists(self.__filename):
            parser = etree.XMLParser(ns_clean=True, remove_blank_text=True)
            tree = etree.parse(self.__filename, parser)
            self.__xmlroot = tree.getroot()
        else:
            self.__xmlroot = etree.Element("pnp")
            self.__xmlroot.addprevious(etree.PI("xml-stylesheet",
                                                "type='text/xsl' href='%s'" %
                                                stylesheet))

            # copy file acs_test_suites/FT/pnp/REPORT/power.xsl" in report path
            srcname = os.path.join(Paths.TEST_SUITES, "FT", "pnp", "REPORT", stylesheet)
            shutil.copy(srcname, report_tree.get_report_path())

        self.__xmltc = etree.Element("TestCase")
        self.__xmlroot.append(self.__xmltc)

        self.update(tcattributes)

        self._tc_name = tcattributes["id"]
        self.__failure = None
        self.__target = None
        self.__target_msg = None
        self.__failure_msg = None
        self.average_measures = []
        self.global_unit = ""

    def __update_targets(self, sformat, extend=None, test_name=None):
        """
        Read targets and failure
        """
        self.__target_msg = "NOT SET"
        self.__failure_msg = "NOT SET"

        if extend is not None:
            failure = extend + "_failure"
            target = extend + "_target"
        else:
            failure = "failure"
            target = "target"

        test_case = self._tc_name
        if test_name is not None and test_name != "SCORE":
            test_case += "_%s" % test_name

        self.__failure = self.__pnptargets.get(device_model=self.__device_model,
                                               test_case=test_case,
                                               key=failure)

        self.__target = self.__pnptargets.get(device_model=self.__device_model,
                                              test_case=test_case,
                                              key=target)

        if self.__failure:
            self.update({"failure": str(self.__failure)})
            self.__failure_msg = ":".join(sformat % x for x in self.__failure)

        if self.__target:
            self.update({"target": str(self.__target)})
            self.__target_msg = sformat % self.__target[0]

    def update(self, attributes):
        """
        Update __xmltc attributes
        """
        for attname, attval in attributes.items():
            self.__xmltc.attrib[attname] = attval

    def get_value(self, attr):
        """
        Get attribute (if exists)

        :param attr: the attribute to get
        :type sleep_mode: str

        :rtype: string
        :return: The value of the attribute

        """
        return self.__xmltc.attrib.get(attr, "")

    def append(self, xmltree):
        """
        Append xml element to structure

        :type xmltree: etree.Element
        :param xmltree: Tree element to append
        """
        self.__xmltc.append(xmltree)

    def write(self, calcul_method="MEDIAN"):
        """
        Write to file
        :rtype: string
        :return: The path to the pnp report
        """
        xmlfile = open(self.__filename, "w")
        xmlstr = etree.tostring(self.__xmlroot.getroottree(),
                                pretty_print=True,
                                encoding='utf-8',
                                xml_declaration=True)
        xmlfile.write(xmlstr)
        xmlfile.close()

        ctnreport = PnPCTNReport(self.__xmltc, self.iteration)
        if not self.average_measures:
            ctnreport.send()
        else:
            self.__logger.info("compute %s for test case measurments" % calcul_method.lower())
            calcul_funct = get_math_function(calcul_method)
            value = calcul_funct(numpy.array(self.average_measures).astype(numpy.float))
            ctnreport.send("%.2f" % value, self.global_unit)

        return self.__filename

    def get_residency_verdict(self, sleep_mode, sleep_duration, test_meas_duration):
        """
        Get the residency message and verdict

        :param sleep_mode: sleep mode.
        :type sleep_mode: str

        :param sleep_duration: Sleep duration
        :type sleep_duration: int

        :rtype: tuple
        :return: The verdict and the associated message for residencies
        """
        tc_date = self.__xmltc.attrib.get("date", "")
        self.__update_targets("%.2f%%", "residency")

        # If residencies verdict is not required
        if self.__failure is None:
            return Global.SUCCESS, ""

        xpath = "/pnp/TestCase[@id=\"%s\"]/SysDebug/"\
                "Residencies" % self._tc_name

        residencies_enabled = self.__xmlroot.xpath(xpath)

        if not residencies_enabled:
            residency_msg = "Residencies Sysdebug module not enabled"
            return Global.FAILURE, residency_msg

        verdict = Global.FAILURE
        xpath = "/pnp/TestCase[@id=\"%s\" and @date=\"%s\"]/SysDebug/"\
                "Residencies/Residency[@mode=\"%s\"]" % \
                (self._tc_name, tc_date, str(sleep_mode))

        residencies = self.__xmlroot.xpath(xpath)
        residency_msg = "No residency found in XML file"

        if residencies:
            spent = float(residencies[0].attrib["residency"])
            residency_msg = "Residency: %0.2f%%; "\
                            "FailureResidency: %s; "\
                            "TargetResidency: %s; "\
                            "ResidencyPeriod: %ds " % \
                            (spent, self.__failure_msg, self.__target_msg,
                             sleep_duration + test_meas_duration)

            verdict = self.check_residency_verdict(spent)

        return verdict, residency_msg

    def check_residency_verdict(self, sleep_mode_value):
        """
        Get the residency verdict

        :param sleep_mode_value: sleep mode value.
        :type sleep_mode_value: float

        :rtype: boolean
        :return: The verdict
        """
        verdict = Global.FAILURE
        if self.__failure is not None and len(self.__failure) > 0 and \
           sleep_mode_value >= self.__failure[0]:
                verdict = Global.SUCCESS

        return verdict

    def retrieve_residency_measures(self, sleep_mode_target):
        """
        Retrieve the residency measures

        :param sleep_mode_target: sleep mode.
        :type sleep_mode_target: str

        :rtype: dict
        :return: List of residency measure as dict
        """
        measures = None
        tc_date = self.__xmltc.attrib.get("date", "")
        self.__update_targets("%.2f%%", "residency")
        xpath = "/pnp/TestCase[@id=\"%s\"]/SysDebug/" \
                "Residencies" % self._tc_name

        residencies_enabled = self.__xmlroot.xpath(xpath)
        if not residencies_enabled:
            return None

        xpath = "/pnp/TestCase[@id=\"%s\" and @date=\"%s\"]/SysDebug/Residencies/Residency" % (self._tc_name, tc_date)

        residencies = self.__xmlroot.xpath(xpath)
        if residencies:
            measures = dict()
            for residency_node in residencies:
                mode = residency_node.attrib["mode"]
                spent = float(residency_node.attrib["residency"])
                measure = dict(residency=float(spent))
                if self.__failure_msg and sleep_mode_target == mode:
                    measure["failure"] = self.__failure_msg
                if self.__target_msg and sleep_mode_target == mode:
                    measure["target"] = self.__target_msg
                if mode in measures:
                    measures[mode].append(measure)
                else:
                    measures[mode] = [measure]

        return measures

    def get_power_verdict(self, verdict_rail):
        """
        Compute verdict for power measurement

        :param verdict_rail: verdict rail.
        :type verdict_rail: float.

        :rtype: tuple
        :return: The verdict and the associated message for power measurement
        """
        tc_date = self.__xmltc.attrib.get("date", "")

        verdict = Global.FAILURE
        xpath = "/pnp/TestCase[@id=\"%s\" and @date=\"%s\"]/PowerAnalyzerTool/"\
                "/PwrRail[@name=\"%s\"]" % \
                (self._tc_name, tc_date, verdict_rail)

        pwrmeas = self.__xmlroot.xpath(xpath)
        pwrmeas_msg = "No or missing PwrRail measurement"

        if pwrmeas:
            measnode = pwrmeas[0]
            self.__update_targets("%.3f")

            if self.__failure:
                measnode.attrib["failure"] = ":".join("%.3f" % x for x in self.__failure)

                average = float(measnode.attrib["average"])
                verdict = self.check_rail_verdict(average)

            if self.__target:
                measnode.attrib["target"] = str(self.__target)

            pwrmeas_msg = "PwrRail: %s; AverageMeasure: %s; "\
                          "FailAverageMeasure: %s; TargetAverageMeasure: %s " % \
                          (verdict_rail, measnode.attrib["average"],
                           self.__failure_msg, self.__target_msg)

        return verdict, pwrmeas_msg

    def check_rail_verdict(self, average_measure):
        """
        Get the pwr verdict

        :param average_measure: average power rail measure.
        :type average_measure: str

        :rtype: boolean
        :return: The verdict
        """
        self.__update_targets("%.3f")
        verdict = Global.FAILURE
        if self.__failure and (len(self.__failure) == 1 and average_measure <= self.__failure[0] \
            or len(self.__failure) >= 2 and self.__failure[0] <= average_measure <= self.__failure[1]):
            verdict = Global.SUCCESS
        return verdict

    def retrieve_power_measures(self, verdict_rail):
        """
        Retrive power measures information

        :param verdict_rail: verdict rail.
        :type verdict_rail: str.

        :rtype: dict
        :return: power measure as dict
        """
        measures = None
        tc_date = self.__xmltc.attrib.get("date", "")
        self.__update_targets("%.3f")

        xpath = "/pnp/TestCase[@id=\"%s\" and @date=\"%s\"]/PowerAnalyzerTool/PwrRail" % (self._tc_name, tc_date)
        pwrmeas = self.__xmlroot.xpath(xpath)
        if pwrmeas:
            measures = dict()
            for measnode in pwrmeas:
                if not "name" in measnode.attrib:
                    continue
                rail_name = measnode.attrib["name"]
                measure = dict(average=float(measnode.attrib["average"]),
                               min=float(measnode.attrib["min"]),
                               max=float(measnode.attrib["max"]),
                               target="No target set",
                               failure="No failure set")
                if self.__failure and rail_name == verdict_rail:
                    measure["failure"] = measnode.attrib["failure"] = ":".join("%.3f" % x for x in self.__failure)
                if self.__target and rail_name == verdict_rail:
                    measure["target"] = str(self.__target)
                if rail_name == verdict_rail:
                    self.average_measures.append(measnode.attrib["average"])
                    self.global_unit = measnode.get("unit", "mA")
                if not rail_name in measures:
                    measures[rail_name] = [measure]
                else:
                    measures[rail_name].append(measure)
        return measures

    def get_performance_verdict(self, scores, lower_is_better=True,
                                secondary_report=None, tc_order=None):
        """
        Get the verdict for performance tests

        :type scores: etree.Element
        :param scores: xml structure where to find value to determine the
                        verdict for performance.

        :type lower_is_better: bool
        :param lower_is_better: Check when lower score are better

        :return: str
        :return: formatted str of scores and additional data

        """
        global_verdict = []
        vtbl_msg = [Verdict.FAIL, Verdict.PASS]
        vtbl = [Global.FAILURE, Global.SUCCESS]
        all_output = ""
        score_output = ""
        is_failure_set = False

        compare_op = ">="
        if lower_is_better:
            compare_op = "<="

        subscores = []
        for item in scores.iter('item'):
            name = item.attrib.get("name").upper()
            score = float(item.attrib.get("value"))
            targets = []

            subscore = {"name": name}
            self.__update_targets("%.3f", test_name=name)

            verdict = 0
            if len(self.__failure) == 1:
                item.attrib["failure"] = "%.3f" % self.__failure[0]
                verdict = eval("%s%s%s" % (score, compare_op, self.__failure[0]))
                targets.append("failure: %s" % self.__failure_msg)
                global_verdict.append(verdict)
                subscore["verdict"] = vtbl_msg[verdict]
                is_failure_set = True
            elif len(self.__failure) == 2:
                item.attrib["failure"] = ":".join("%.3f" % x for x in self.__failure)
                verdict = self.__failure[0] <= score <= self.__failure[1]
                targets.append("failure: %s" % str(self.__failure_msg))
                global_verdict.append(verdict)
                subscore["verdict"] = vtbl_msg[verdict]
                is_failure_set = True

            if self.__target:
                item.attrib["target"] = str(self.__target)
                targets.append("target: %s" % str(self.__target_msg))

            target_msg = ""
            if len(targets) > 0:
                target_msg = " (" + ", ".join(targets) + ")"

            test_output = "%s: %s%s; " % (name.lower(), score, target_msg)
            subscore["msg"] = test_output
            subscores.append(subscore)
            if name.lower() == "score":
                score_output = test_output

            all_output += test_output

        verdict = 0
        if is_failure_set:
            verdict = min(global_verdict)

        if secondary_report is not None:
            for subscore in subscores:
                tc_verdict = None
                if "verdict" in subscore:
                    tc_verdict = subscore["verdict"]
                elif not is_failure_set:
                    tc_verdict = vtbl_msg[0]

                if tc_verdict is not None:
                    secondary_report.add_result(subscore["name"],
                                                tc_verdict,
                                                subscore["msg"],
                                                self._tc_name,
                                                tc_order)

        prefix_msg = ""
        if not is_failure_set:
            prefix_msg = "No Targets defined - "

        self.update({"verdict": vtbl_msg[verdict]})
        if score_output:
            return vtbl[verdict], prefix_msg + score_output
        else:
            return vtbl[verdict], prefix_msg + all_output


class PnPTargets:
    """
    Retrieve targets from CSV file.
    """
    __targets = {}

    def __init__(self, failure_file=None, target_file=None):
        """
        Constructor

        :type target_file: String
        :param target_file: Name of the file used to retrieve targets.

        :type failure_file: String
        :param failure_file: Name of the file used to retrieve targets.
        """

        # Targets have already been loaded
        if self.__targets:
            return

        if failure_file is not None and os.path.isfile(failure_file):
            failures = self.__read_file(failure_file)
            if(failures is not None):
                self.__merge_to_targets(failures, "failure")

        if target_file is not None and os.path.isfile(target_file):
            targets = self.__read_file(target_file)
            if(targets is not None):
                self.__merge_to_targets(targets, "target")

    def __merge_to_targets(self, config, key):
        """
        Merge config to targets structure

        :type config: dictionnary
        :param config: Structure to merge to targets

        :type key: tuple
        :param key: (device model, test case) tuple used to get targets
        """

        for dmtc, item in config.items():
            if dmtc not in self.__targets:
                self.__targets[dmtc] = {}

            self.__targets[dmtc][key] = item[0]
            self.__targets[dmtc]["residency_" + key] = item[1]

    def __read_conf(self, tcconf):
        """
        Read configuration datum
        """
        tcconf = tcconf.replace(",", ".").replace("%", "")
        if tcconf == "":
            return ()

        values = tcconf.split(":")
        if len(values) == 1:
            return [float(values[0])]
        else:
            return sorted([float(values[0]), float(values[1])])

    def __read_file(self, filename):
        """
        Read the file and build a dictionnary for targets, failures and residencies

        :type filename: String
        :param filename: Name of csv file to read

        :rtype: dictionnary
        :return: Dictionnary for targets, failure, residency target, residency
                 failure of each (devicemodel, tests) tuple:

            {
                ("device_model", "test_case_name") : ["failure",
                                                      "residency_failure"]
            }
        """
        csv = open(filename, "r")
        lines = csv.read().splitlines()
        if(len(lines) == 0):
            return
        ret = {}

        # First line is header (test cases name)
        header = lines.pop(0)
        testcases = header.split(";")

        # First column is device model. Ignore it in header
        testcases.pop(0)

        for line in lines:
            config = line.strip(";").split(";")
            device_model = config.pop(0)

            tc_num = 0
            for tcconf in config:
                if testcases[tc_num][-10:] == "_RESIDENCY":
                    rank = 1
                    tcname = testcases[tc_num][0:-10]
                else:
                    rank = 0
                    tcname = testcases[tc_num]
                tc_num += 1

                if (device_model, tcname) not in ret.keys():
                    ret[(device_model, tcname)] = [None] * 2

                ret[(device_model, tcname)][rank] = self.__read_conf(tcconf)

        return ret

    def get(self, device_model, test_case, key):
        """
        Return the targets for the tuple (device_model, test_case)

        :type device_model: str
        :param device_model: device model

        :type test_case: str
        :param test_case: Name of the test case

        :type key: str
        :param key: The key we wants to read (residency/target/failure)
        """
        if (device_model, test_case) in self.__targets and\
           key in self.__targets[(device_model, test_case)]:
            return self.__targets[(device_model, test_case)][key]

        if ("DEFAULT", test_case) in self.__targets and\
           key in self.__targets[("DEFAULT", test_case)]:
            return self.__targets[("DEFAULT", test_case)][key]

        return []


class PnPConfiguration:

    """
    Read PnP configuration file
    """

    def __init__(self, device_model, tcname):
        """
        Constuctor

        :type device_model: str
        :param device_model: device model

        :type tcname: str
        :param tcname: Name of the test case
        """
        self.__tcname = tcname
        self.__device_model = device_model
        self.__parameters = {}

        pnp_config_file = os.path.join(Paths.CONFIGS, "Pnp_Config.xml")

        if not self.__device_model or not self.__tcname:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unable to retrieve device model or test case name")

        if not os.path.exists(pnp_config_file):
            self.__xmlroot = etree.Element("pnpconfig")
        else:
            parser = etree.XMLParser(ns_clean=True, remove_blank_text=True)
            tree = etree.parse(pnp_config_file, parser)
            self.__xmlroot = tree.getroot()

    def get(self, name):
        """
        Get the parameter from name.
        Compute the weight of parameters found with name and return the last eaviest
        one parameter :
            * test_case weight is 2
            * device_model weight is 1
        """
        if name in self.__parameters:
            return self.__parameters[name]

        self.__parameters[name] = {}
        for parameter in self.__xmlroot.xpath("Parameter[@name = '%s']" % name):
            prmtc = parameter.attrib.get("test_case")
            prmdm = parameter.attrib.get("device_model")
            curtc = self.__parameters[name].get("test_case")
            curdm = self.__parameters[name].get("device_model")

            # Ignore mismatch configuration
            if prmtc and prmtc != self.__tcname or \
                    prmdm and prmdm != self.__device_model:
                continue

            weight_attr = 2 * bool(prmtc) + bool(prmdm)
            weight_ret = 2 * bool(curtc) + bool(curdm)

            if weight_attr >= weight_ret:
                self.__parameters[name] = parameter.attrib

        return self.__parameters[name]


def grouped_regex_from_list(groups):
    """
    Get a regular expression from a dictionary formatted as followed:
        [
            [groupname,
             before_pattern,
             group_pattern,
             after_pattern],
            [...]
        ]

        where
          - groupname is the name of the group in regular expression,
          - before_pattern is the pattern before the group to match,
          - group_pattern is the pattern of the group to match,
          - after_pattern is the pattern after the group to match

        The regular expression will be:
            "before_pattern(?P<groupname>group_pattern)after_pattern"

        Many groups can be defined as so
    """
    retpattern = ""
    for group in groups:
        grouppattern = "%s(?P<%s>%s)%s" % (group[1], group[0], group[2], group[3])
        retpattern += grouppattern

    return retpattern


class PatLibUtil(object):
    """
    Util for PatLib
    """

    def __init__(self, pat_config, dut_name, tcname, logger):
        """
        Constructor
        """
        self.__patconf = pat_config
        self.__pnp_config = PnPConfiguration(dut_name, tcname)
        self.__logger = logger
        self.__patlib = None
        self.__verdict_rail = None
        self.__patconf_file = None
        self.__get_config_file()
        self.__retrieve_power_rail()
        self.__tcname = tcname

    @property
    def conf_file(self):
        return self.__patconf_file

    @property
    def power_rail(self):
        return self.__verdict_rail

    def __create_config_file(self):
        """
        """
        # Check mandatory parameters
        daqname_value = self.__patconf.get_param_value("DaQName", "")
        channel_value = self.__patconf.get_param_value("Channel", "")
        resistor_value = self.__patconf.get_param_value("Resistor", "")

        if "" in (daqname_value, channel_value, resistor_value):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "DaQName, Channel and Resisor must be set")

        xmlroot = etree.Element("PowerProject")
        xmlroot.attrib["name"] = "ACSProject"
        xmlroot.attrib["ekey_file_version"] = ""
        xmlroot.attrib["ekey_file_hash"] = ""
        xmlroot.attrib["patlib_version"] = "2.7"
        xmlroot.attrib["edition_date"] = datetime.strftime(datetime.now(), "%B %d %a %I:%M:%S %Y")
        xmlroot.attrib["saving_directory"] = tempfile.gettempdir()

        properties = etree.Element("Acquisition_Properties")

        rates = etree.Element("Samplerate")
        rates.text = self.__patconf.get_param_value("Samplerate", "50000")

        duration = etree.Element("Duration")
        duration.text = "180"
        properties.append(rates)
        properties.append(duration)

        xmlroot.append(properties)
        xmlroot.append(etree.Element("Comment"))

        channels = etree.Element("Channels")
        daq = etree.Element("DAQ")
        daq.attrib["hw_lowpass_filter_value"] = "0"
        daq.attrib["name"] = daqname_value
        daq.attrib["sw_lowpass_filter_value"] = "0"

        channel = etree.Element("Channel")
        channel.attrib["name"] = self.__patconf.get_param_value("VerdictRail")
        channel.attrib["color"] = self.__patconf.get_param_value("Color", "(197, 102, 110, 255)")
        channel.attrib["max"] = self.__patconf.get_param_value("Max", "0.1")
        channel.attrib["hw_channel"] = channel_value
        channel.attrib["type"] = self.__patconf.get_param_value("Type", "diffI")
        channel.attrib["resistor"] = resistor_value
        daq.append(channel)

        channels.append(daq)
        xmlroot.append(channels)

        patconf = etree.tostring(xmlroot.getroottree(),
                                 pretty_print=True,
                                 encoding='utf-8',
                                 xml_declaration=True)

        tmpfile = os.path.join(tempfile.gettempdir(), "pat_conf.xml")

        if os.path.isfile(tmpfile):
            os.remove(tmpfile)

        tmpfd = open(tmpfile, "w")
        tmpfd.write(patconf)
        tmpfd.close()

        return tmpfile

    def __get_config_file(self):
        """
        Get the PAT configuration file

        Try to read the PAT configuration file from Bench_Config.
        If it's not found, the configuration file will be read in the
        UseCase configuration.

        @rtype: String
        @return: PAT configuration file
        """
        conf_file = self.__patconf.get_param_value("ConfFile", "")
        self.__logger.debug("PAT configuration file from BenchConfig is %s" % (conf_file))

        patlib = self.__pnp_config.get("patlib")
        patconf_file = None
        if patlib:
            patconf_file = patlib.get("config_file")

        if patconf_file:
            conf_file = patconf_file
            self.__logger.info("PAT configuration file from Pnp_Config is %s" % (conf_file))

        if not conf_file:
            self.__patconf_file = self.__create_config_file()
        else:
            self.__patconf_file = os.path.join(Paths.EXECUTION_CONFIG, conf_file)

    def __retrieve_power_rail(self):
        """
        Get the rail to compute verdict.

        Try to read the verdict from Bench_Config.
        If it's not found, the verdict will be read in the UseCase configuration.

        @rtype: String
        @return: Verdict rail
        """
        verdict_rail = self.__patconf.get_param_value("VerdictRail")
        self.__logger.debug("PAT Verdict rail from BenchConfig is %s" % (verdict_rail))

        patlib = self.__pnp_config.get("patlib")
        verdict_rail_pnp = patlib.get("verdict_rail")
        if patlib and verdict_rail_pnp:
            verdict_rail = verdict_rail_pnp
            self.__logger.info("PAT Verdict rail from Pnp_Config is %s" % verdict_rail)

        if not verdict_rail:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "verdict rail is not set")
        self.__verdict_rail = verdict_rail

    def export(self, patconf_obj, report_tree, tcdate):
        """
        Save power measurement data in campaign report tree

        @rtype: Tuple
        @return: Export verdict & export location
        """
        raw_pat_file = os.path.join(report_tree.get_report_path(),
                                    "PowerMeasurementData_%s_%s.xml" %
                                    (tcdate.replace(" ", "_").replace(":", ""), self.__tcname))

        res = patconf_obj.export(raw_pat_file)

        csv_file = os.path.join(report_tree.get_report_path(),
                                "PowerMeasurementData_%s_%s" %
                                (tcdate.replace(" ", "_").replace(":", ""), self.__tcname),
                                "export.csv")
        patconf_obj.export_csv(csv_file)

        return res, raw_pat_file

    def export_plot(self, patconf_obj, report_tree, tcdate):
        """
        Save power graph data in campaign report tree
        """
        raw_pat_file = os.path.join(report_tree.get_report_path(),
                                    "PowerMeasurementData_%s_%s.xml" %
                                    (tcdate.replace(" ", "_").replace(":", ""), self.__tcname))
        dat_pat_file = os.path.join(report_tree.get_report_path(),
                                    "PowerMeasurementData_%s_%s" %
                                    (tcdate.replace(" ", "_").replace(":", ""), self.__tcname),
                                    self.__verdict_rail + ".dat")
        js_out_file = os.path.join(report_tree.get_report_path(),
                                   "PowerMeasurementData_%s_%s" %
                                   (tcdate.replace(" ", "_").replace(":", ""), self.__tcname),
                                   "graph_values.js")

        res = patconf_obj.export_plot(raw_pat_file, dat_pat_file, js_out_file)

        # Copy power_graph.html
        srcname = os.path.join(Paths.TEST_SUITES, "FT", "pnp", "REPORT", "power_graph.html")
        destname = os.path.join(report_tree.get_report_path(),
                                "PowerMeasurementData_%s_%s" %
                                (tcdate.replace(" ", "_").replace(":", ""), self.__tcname))
        shutil.copy(srcname, destname)

        # Compute plots for other rails than verdict rail (if any)
        export_rail = self.__patconf.get_param_value("ExportedRails", "")
        export_rail_testcase = self.__patconf.get_param_value("ExportedRails_%s" % self.__tcname, "")

        dat_pat_folder = os.path.join(report_tree.get_report_path(),
                                      "PowerMeasurementData_%s_%s" %
                                      (tcdate.replace(" ", "_").replace(":", ""),
                                       self.__tcname))

        if export_rail:
            self.__logger.debug("Rails to export : %s" % export_rail_testcase)
            filterlist = export_rail.split(';')
            patconf_obj.export_all_plots(raw_pat_file, dat_pat_folder, filterlist)

        if export_rail_testcase:
            self.__logger.debug("Additional rails to export (test case specific) : %s" % export_rail_testcase)
            filterlist = export_rail_testcase.split(';')
            patconf_obj.export_all_plots(raw_pat_file, dat_pat_folder, filterlist)

        return res

    def export_indicators(self, patconf_obj, report_tree, tcdate):
        """
        Return power indicators for pnp result
        """
        # Compute indicators for verdict rail
        raw_pat_file = os.path.join(report_tree.get_report_path(),
                                    "PowerMeasurementData_%s_%s.xml" %
                                    (tcdate.replace(" ", "_").replace(":", ""), self.__tcname))
        dat_pat_folder = os.path.join(report_tree.get_report_path(),
                                      "PowerMeasurementData_%s_%s" %
                                      (tcdate.replace(" ", "_").replace(":", ""), self.__tcname))
        dat_pat_file = os.path.join(dat_pat_folder, self.__verdict_rail + ".dat")
        res = patconf_obj.export_indicators(raw_pat_file, dat_pat_file, self.__verdict_rail)

        # Compute indicators for other rails than verdict rail (if any is required)
        export_rail = self.__patconf.get_param_value("ExportedRails", "")
        export_rail_testcase = self.__patconf.get_param_value("ExportedRails_%s" % self.__tcname, "")

        if export_rail:
            self.__logger.debug("Rails to export : %s" % export_rail)
            filterlist = export_rail.split(';')
            res = patconf_obj.export_all_indicators(raw_pat_file, dat_pat_folder, filterlist)

        if export_rail_testcase:
            self.__logger.debug("Additional rails to export (test case specific) : %s" % export_rail_testcase)
            filterlist = export_rail_testcase.split(';')
            res = patconf_obj.export_all_indicators(raw_pat_file, dat_pat_folder, filterlist)

        return res

    def clear_data(self, patconf_obj):
        """
        Clear power data from previous captures (if any)
        """
        patconf_obj.clear_data()

def get_math_function(operation):
    """
    Get the PAT configuration file

    Try to read the PAT configuration file from Bench_Config.
    If it's not found, the configuration file will be read in the
    UseCase configuration.

    :type operation: str
    :param operation: operation to do

    @rtype: function
    @return: function to use
    """
    if operation == "ARITHMETIC_MEAN":
        functor = numpy.mean
    elif operation == "GEOMETRIC_MEAN":
        functor = scipy.stats.mstats.gmean
    elif operation == "MEDIAN":
        functor = numpy.median
    elif operation == "RAW":
        functor = str
    else:
        raise AcsConfigException(AcsConfigException.FEATURE_NOT_IMPLEMENTED,
                                 "Unknown operation {0}".format(operation))
    return functor


class PnPCTNReport:
    __logger = LOGGER_FWK
    __pi = None
    __pi_residency = None

    def __init__(self, xmltree, iteration=False):

        self.iteration = iteration
        self._test_verdict = Verdict.PASS

        # Create metrics node #
        self._em = EquipmentManager()
        self._pat = self._em.get_global_config().benchConfig.get_parameters("POWER_ANALYZER_TOOL")
        self._verdict_rail = self._pat.get_param_value("VerdictRail")
        self._supply = float(DeviceManager().get_device_config("PHONE1").get("PowerSupplyValue", 0.0))

        self.__pi = {}
        self.__pi["name"] = xmltree.attrib.get("id", None)

        self.__report = dict((x, self.format_param(xmltree.attrib.get(x))) for x in xmltree.attrib if x != "failure")

        if "residency_mode" in xmltree.attrib:
            self._residency_mode = xmltree.attrib["residency_mode"]
            self.__logger.debug("residency mode catched: %s" % self._residency_mode)
        else:
            self._residency_mode = ""

        if "verdict" in xmltree.attrib:
            self.__report.update([("verdict", self.format_verdict(xmltree.attrib.get("verdict")))])

        if "duration" in xmltree.attrib:
            duration_value = xmltree.attrib.get("duration")
            self._get_duration_dict(duration_value)

        self._get_power_dict(xmltree)
        self._get_perf_dict(xmltree)

        self._get_sysdbg_dict(xmltree)

    def __format_value(self, value, split=None):
        """
        """
        if split is not None:
            return [self.format_param(i) for i in value.split(str(split))]
        else:
            return self.format_param(value)

    def __get_generic_dict(self, xmltree, key, split=None):
        """
        """
        report = {}

        nodes = xmltree.xpath("child::node()")
        for item in nodes:
            if key not in item.attrib:
                self.__logger.error("Module %s does not have attribute %s" %
                                    (item.tag, key))
                continue

            keyvalue = item.attrib.get(key)
            report[keyvalue] = {}

            for name, value in item.attrib.items():
                report[keyvalue][name] = self.__format_value(value, split)

        return [(xmltree.tag.lower(), report)]

    def __get_generic_list(self, xmltree, keylist, split=None):
        """
        """
        report = {}

        nodes = xmltree.xpath("child::node()")
        report["raws"] = []
        report["columns"] = keylist

        for item in nodes:
            element = []
            for key in keylist:
                if key not in item.attrib:
                    element.append("")
                else:
                    value = item.attrib.get(key)
                    element.append(self.__format_value(value, split))

            report["raws"].append(element)

        return [(xmltree.tag.lower(), report)]

    def _get_sysdbg_dict(self, xmltree):
        """
        """
        modules = xmltree.xpath("SysDebug/child::node()")
        for modnode in modules:
            modname = modnode.tag.lower()
            funcname = "_get_%s_dict" % modname
            functor = None

            if hasattr(self, funcname):
                self.__logger.debug("Fill CTN report for %s" % modname)
                functor = getattr(self, funcname)
                self.__report.update(functor(modnode))
            else:
                self.__logger.warning("There is no methods to export data "
                                      "for sysdebug module %s. No export "
                                      "will be done to CTN" % modname)

    def _get_duration_dict(self, duration_value):
        """
            Extract duration with CTN format from xml
        """
        duration = {}
        duration["value"] = duration_value
        duration["unit"] = "s"

        self.__report.update([("duration", duration)])

    def _get_perf_dict(self, xmltree):
        """
        """
        node = xmltree.xpath("scores")
        if len(node) == 0:
            return

        self.__report.update(self.__get_generic_dict(node[0], "name", ";"))

    def _get_power_dict(self, xmltree):
        """
        """
        node = xmltree.xpath("PowerAnalyzerTool/PwrRail")
        if len(node) == 0:
            return

        if self._supply:
            self.__logger.debug("Supply voltage set: %s" % self._supply)
        else:
            self.__logger.info("You have to set voltage value in bench config to return mW Power values")

        current = {}
        current["data"] = {}
        power = {}
        power["data"] = {}
        voltage = {}
        voltage["data"] = {}

        for rail in node:
            attribs = rail.attrib
            name = attribs.get("name", "")
            name = name.replace(".", "_") # no '.' allowed in json keys
            unit = attribs.get("unit", "mA")

            if "A" in unit:
                toreport = current
            elif "W" in unit:
                toreport = power
            elif "V" in unit:
                toreport = voltage
            else:
                toreport = current

            toreport["unit"] = unit
            toreport["data"][name] = dict([(x, self.format_param(attribs[x])) for x in attribs if x in ["average", "failure", "target"]])

            if self._verdict_rail is not None and name == self._verdict_rail:
                if "Batt" in self._verdict_rail and self._supply and "mA" in unit:
                    self.__pi["value"] = self.format_param("%.2f" % (float(attribs.get("average"))*float(self._supply)))
                    self.__pi["unit"] = "mW"
                else:
                    self.__pi["value"] = self.format_param(attribs.get("average"))
                    self.__pi["unit"] = unit

        if "value" not in self.__pi or self.__pi["value"] <= 0:
            self._test_verdict = Verdict.FAIL

        # If no power rail for battery, get the best matching rail to report
        if self._supply and not [x for x in power["data"].keys() if "Batt" in x]:
            for currentrail in current["data"].keys():
                if "Batt" in currentrail:
                    power["unit"] = "mW"
                    attr = current["data"][currentrail]
                    power["data"][currentrail] = dict([(x, self.format_param("%.2f" % (float(attr[x])* self._supply))) for x in attr])

        if current["data"]:
            self.__report.update([("CurrentRail", current)])
        if power["data"]:
            self.__report.update([("PwrRail", power)])
        if voltage["data"]:
            self.__report.update([("VoltageRail", voltage)])

        self._get_powerplot_dict()
        self._get_power_indicators_dict(xmltree)

    def _get_power_indicators_dict(self, xmltree):
        """
        """
        node = xmltree.xpath("PowerAnalyzerTool/PwrIndicators/Rail")
        if len(node) == 0:
            node = xmltree.xpath("PwrIndicators/Rail")
            if len(node) == 0:
                return

        pwrindic = {}
        pwrindic["data"] = {}
        pwrindic["unit"] = {}
        currindic = {}
        currindic["data"] = {}
        currindic["unit"] = {}
        voltindic = {}
        voltindic["data"] = {}
        voltindic["unit"] = {}
        supplyindic = {} # computed from currindic for V_Batt if no V_Batt power rail existing
        supplyindic["data"] = {}
        supplyindic["unit"] = {}

        for indicator in node:
            attribs = indicator.attrib
            name = attribs.pop("name", "")
            name = name.replace(".", "_") # no '.' allowed in json keys
            unit = attribs.pop("unit", "")

            if "A" in unit:
                toreport = currindic
            elif "W" in unit:
                toreport = pwrindic
            elif "V" in unit:
                toreport = voltindic
            else:
                toreport = currindic

            toreport["data"][name] = dict([(x, self.format_param(attribs[x])) for x in attribs])
            for indic in attribs:
                if "residency" in indic:
                    toreport["unit"][indic] = "%"
                elif "impact" in indic or "steady_state" in indic or "floor" in indic:
                    toreport["unit"][indic] = unit

            if self._supply and "mA" in unit and "Batt" in name:
                supplyindic["data"][name] = dict([(x, self.format_param(attribs[x])) for x in attribs])
                for indic in attribs:
                    if "residency" in indic:
                        supplyindic["unit"][indic] = "%"
                    elif "impact" in indic or "steady_state" in indic or "floor" in indic:
                        supplyindic["unit"][indic] = "mW"
                        if type(supplyindic["data"][name][indic]) in (int, float):
                            supplyindic["data"][name][indic] *= self._supply

        if pwrindic["data"]:
            self.__report.update([("PwrIndicators", pwrindic)])
        elif self._supply and supplyindic["data"]:
            self.__report.update([("PwrIndicators", supplyindic)])

        if currindic["data"]:
            self.__report.update([("CurrentIndicators", currindic)])
        if voltindic["data"]:
            self.__report.update([("VoltageIndicators", voltindic)])

    def _get_powerplot_dict(self):
        """
        """

        self._em = EquipmentManager()
        try:
            self._pat = self._em.get_power_analyzer_tool("POWER_ANALYZER_TOOL")
            raw = self._pat.get_plot()
            if raw:
                cols = ["Time (s)", "Consumption (mA)"]
                conso = {}
                conso["columns"] = cols
                conso["raw"] = raw
                power = {}
                power["consumption"] = conso
                self.__report.update([("power", power)])

            # Upload one graph per rail in the correct graph (according to unit)
            power_plot = {}
            power_plot["title"] = {"text": "Power graph (all power rails measured are displayed)"}
            power_plot["xAxis"] = {"title" : "time (s)"}
            power_plot["yAxis"] = {"title" : "consumption (mW)"}
            power_plot["tooltip"] = {"valueSuffix" : "mW"}
            power_plot["series"] = []

            current_plot = {}
            current_plot["title"] = {"text": "Current rails graph (all current rails measured are displayed)"}
            current_plot["xAxis"] = {"title" : "time (s)"}
            current_plot["yAxis"] = {"title" : "consumption (mA)"}
            current_plot["tooltip"] = {"valueSuffix" : "mA"}
            current_plot["series"] = []

            voltage_plot = {}
            voltage_plot["title"] = {"text": "Voltage rails graph (all voltage rails measured are displayed)"}
            voltage_plot["xAxis"] = {"title" : "time (s)"}
            voltage_plot["yAxis"] = {"title" : "voltage (V)"}
            voltage_plot["tooltip"] = {"valueSuffix" : "V"}
            voltage_plot["series"] = []

            allplots = self._pat.get_all_rails_plots()
            for rail in allplots.keys():
                unit = self._pat.get_rail_unit(rail)
                if "A" in unit:
                    toreport = current_plot
                elif "W" in unit:
                    toreport = power_plot
                elif "V" in unit:
                    toreport = voltage_plot
                else:
                    toreport = current_plot

                graph = {"name" : rail, "data" : allplots[rail]}
                toreport["series"].append(graph)

            if power_plot["series"]:
                LiveReporting.instance().send_test_case_chart(power_plot)
            if current_plot["series"]:
                LiveReporting.instance().send_test_case_chart(current_plot)
            if voltage_plot["series"]:
                LiveReporting.instance().send_test_case_chart(voltage_plot)

        except Exception as ex:
            error_msg = str(ex)
            self.__logger.error(error_msg)

    def _get_thermals_dict(self, xmltree):
        """
        """
        nodes = xmltree.xpath("thermal")

        report = {}
        report["data"] = {}
        for thermal in nodes:
            report["unit"] = "Celsius degrees"
            report["data"][thermal.attrib["name"]] = [self.format_param(f) for f in thermal.attrib["value"].split(";")]

        return [("thermals", report)]

    def _get_residencies_dict(self, xmltree):
        """
        """
        residencies = self.__get_generic_dict(xmltree, "mode")

        for key, value in residencies[0][1].items():
                if self._residency_mode and key == self._residency_mode:
                    self.__logger.debug("Residency mode pushed to TCR: %s" % str(key))
                    self.__pi_residency = {}
                    self.__pi_residency["value"] = value["residency"]
                    self.__pi_residency["unit"] = "%"

        return [(residencies[0][0], {"data": residencies[0][1], "unit": {"residency": "%"}})]

    def _get_wakelocks_dict(self, xmltree):
        """
        """
        return self.__get_generic_list(xmltree, ["name", "acquired", "released", "spent"])

    def _get_alarms_dict(self, xmltree):
        """
        """
        return self.__get_generic_list(xmltree, ["date", "intent", "pkg"])

    def _get_s3failures_dict(self, xmltree):
        """
        """
        return self.__get_generic_list(xmltree, ["date", "device", "reason"])

    def _get_activewakeupsource_dict(self, xmltree):
        """
        """
        return self.__get_generic_list(xmltree, ["date", "wakeupsource"])

    def _get_irqwakeups_dict(self, xmltree):
        """
        """
        return self.__get_generic_list(xmltree, ["number", "name", "occurences"])

    def _get_interrupts_dict(self, xmltree):
        """
        """
        return self.__get_generic_list(xmltree, ["irq", "number", "info"])

    def _get_dstates_dict(self, xmltree):
        """
        """
        nodes = xmltree.xpath("device")

        report = ""
        for device in nodes:
            report += device.attrib["line"]

        return [("dstates", report)]

    def format_param(self, param):
        """
        Formats parameters in order to put them in the correct format
        """
        try:
            ret = eval(param)
            return ret
        except (SyntaxError, NameError):
            pass

        return param

    def send(self, global_measure=None, global_unit="mA"):
        """
        send the report
        """
        # moving residency failure & targer to a new node (if they exists)
        if "residency_mode" in self.__report and "failure" in self.__report:
            res = self.__report["residency_mode"]
            res_node = {}
            res_node["failure"] = self.__report["failure"]
            del self.__report["failure"]
            if "target" in self.__report:
                res_node["target"] = self.__report["target"]
                del self.__report["target"]
            self.__report.update([(res, res_node)])

        self._format_pi()
        if self.iteration:
            LiveReporting.instance().send_start_tc_info(tc_name=self.__pi["name"], iteration=self.iteration)

        self.__logger.debug(str(self.__report))
        if self._residency_mode and self._residency_mode != "s0" and self.__pi_residency:
            metrics = [self.__pi, self.__pi_residency]
        else:
            metrics = [self.__pi]

        LiveReporting.instance().update_running_tc_info(test_info={"PNP": self.__report, "metrics": metrics}, iteration=self.iteration)

        # if global_measure is not None:
        #     if "Batt" in self._verdict_rail and self._supply and "mA" in global_unit:
        #         metrics[0]["value"] = self.format_param("%.2f" % (float(global_measure)*float(self._supply)))
        #         metrics[0]["unit"] = "mW"
        #     else:
        #         metrics[0]["value"] = self.format_param(global_measure)
        #         metrics[0]["unit"] = global_unit
        #     self.__logger.debug("metrics sent: %s" % metrics)
        #     LiveReporting.instance().update_running_tc_info(test_info={"metrics": metrics})

        if self.iteration:
            LiveReporting.instance().send_stop_tc_info(verdict=self._test_verdict,
                                                       execution_nb=1,
                                                       success_counter=1,
                                                       max_attempt=1,
                                                       acceptance_nb=1,
                                                       tc_comments="",
                                                       iteration=self.iteration)
        if Verdict.FAIL in self._test_verdict:
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, "Wrong, or not present, power measure")

    def _format_pi(self):
        """
        extract pi name from test case name
        """
        self.__logger.debug("pi name: %s" % self.__pi["name"])
        if "ST" in self.__pi["name"]:
            self.__pi["name"] = self.__pi["name"].split("_", 1)[1]
        elif "PWRMEAS" in self.__pi["name"]:
            self.__pi["name"] = self.__pi["name"].split("_", 2)[2]
        elif not self.__pi["name"]:
            self.__pi["name"] = "Undefined"

        if self._residency_mode and self._residency_mode != "s0" and self.__pi_residency:
                self.__pi_residency["name"] = self.__pi["name"] + "_residency"

    def format_verdict(self, verdict):
        """
        Format verdict to match with CTN needs
        """
        if verdict == "FAIL":
            return Verdict.FAIL
        elif verdict == "PASS":
            return Verdict.PASS
        else:
            return verdict
