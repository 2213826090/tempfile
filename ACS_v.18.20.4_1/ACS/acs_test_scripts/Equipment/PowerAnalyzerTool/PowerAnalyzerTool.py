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
:summary: implementation of PatLib equipment for power measurement
:author: pbluniex
"""

import os
from lxml import etree
import numpy
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from patlib.power_measurements import PowerMeasurements  # @UnresolvedImport


class PowerAnalyzerTool(EquipmentBase):

    """
    patlib library interface
    """
    __instance = None

    def __init__(self, name, model, params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type params: dict
        :param params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        EquipmentBase.__init__(self, name, model, params)

        # Instantiate Pat library (by default NOT stubbed mode)
        # - True = STUB mode (NiDAQ DLL is not loaded)
        # - False or no argument = REAL mode (NiDAQ DLL is loaded)
        # Configure Pat tool
        if model == "STUB":
            self.__instance = PowerMeasurements(True)
        else:
            self.__instance = PowerMeasurements(False)

        self._plot = None
        self._all_plots = {}
        self._all_indicators = {}

        self.detect_percent = 0.5
        self.fact_min = 1 - self.detect_percent
        self.fact_max = 1 + self.detect_percent
        self.__pat_file = None
        self.__rails_conf = None

    def __getattr__(self, attr):
        """
        Call patlib methods
        """
        return getattr(self.__instance, attr)

    def init(self, pat_file):
        """
        Configure patlib with file
        """
        if pat_file is None:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Unable to retrieve Pat configuration file !")

        if not os.path.isfile(pat_file):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Pat configuration file '%s' not found !" % str(pat_file))

        self.__instance.configure(pat_file)
        self.__pat_file = pat_file

    def __fetch_rail_conf(self):
        """
        Retrieve all rails information from pat conf file

        :rtype:     dict
        :return:    dict of rails information
        """
        if self.__rails_conf or not self.__pat_file:
            return self.__rails_conf

        self.__rails_conf = dict()
        try:
            rootconf = etree.parse(self.__pat_file).getroot()
            all_channels = list(rootconf.iter("Virtual_Channel")) + list(rootconf.iter("Channel"))
            for rail in all_channels:
                if "name" in rail.attrib:
                    name = rail.attrib.pop("name", "")
                    properties = dict(rail.attrib)
                    if "unit" not in properties and "type" in properties:
                        if "RSE" in properties["type"]:
                            properties["unit"] = "V"
                        elif "diffI" in properties["type"]:
                            properties["unit"] = "mA"
                        elif name.endswith("I"):
                            properties["unit"] = "mA"
                        elif name.endswith("V"):
                            properties["unit"] = "V"
                        elif name.endswith("P"):
                            properties["unit"] = "mW"
                    self.__rails_conf[name] = properties
        finally:
            return self.__rails_conf

    def get_rail_unit(self, railname):
        """
        Get the rail unit

        :rtype:     dict
        :return:    dict of rails information
        """
        rails = self.__fetch_rail_conf()
        if railname in rails and "unit" in rails[railname]:
            return rails[railname]["unit"]
        return ""

    def get_measurements(self):
        """
        Retrieve all measurements and store it as dict

        :rtype:     list
        :return:    list of measurements
        """
        res_measure = []
        measurements = self.__instance.get_measurements()
        if not measurements:
            measurements = []

        for measure in measurements:
            measurement = {}
            measurement["name"] = measure[0]
            measurement["average"] = "%0.3f" % measure[1]
            measurement["min"] = "%0.3f" % measure[2]
            measurement["max"] = "%0.3f" % measure[3]
            res_measure.append(measurement)
        return res_measure

    def report(self,
               isPowerCalculationNeeded=False,
               rawDataFile=None,
               datDataFile=None,
               jsOutFile=None):
        """
        Return lxml.etree.Element node with measure

        :type isPowerCalculationNeeded: boolean
        :param isPowerCalculationNeeded: Switch to power calculation mode

        :type rawDataFile: str
        :param rawDataFile: Filename to write raw data
        :type datDataFile: str
        :param datDataFile: Filename to read raw data (.dat file)
        """
        xmlpat = etree.Element("PowerAnalyzerTool")
        rail = None

        if isPowerCalculationNeeded:
            self.__instance.power_calculation()

        measurements = self.get_measurements()

        for measure in measurements:
            pwrrail = etree.Element("PwrRail")
            name = measure.get("name")
            pwrrail.attrib["name"] = name
            pwrrail.attrib["average"] = measure.get("average")
            pwrrail.attrib["min"] = measure.get("min")
            pwrrail.attrib["max"] = measure.get("max")
            unit = self.get_rail_unit(name)
            if unit:
                pwrrail.attrib["unit"] = unit
            xmlpat.append(pwrrail)
            if datDataFile and "%s.dat" % name in datDataFile:
                rail = name

        # Append Raw data to the PowerAnalyzerTool xml node
        if rawDataFile:
            self.export(rawDataFile)

        self.clear_data()

        if rawDataFile and datDataFile and jsOutFile:
            xmlplot = self.export_plot(rawDataFile, datDataFile, jsOutFile)
            xmlpat.append(xmlplot)

        if rawDataFile and datDataFile and rail:
            xmlindic = self.export_indicators(rawDataFile, datDataFile, rail)
            xmlpat.append(xmlindic)

        return xmlpat

    def export_csv(self, filename):
        """
        Generates a csv file with average/min/max values for each rail
        """
        self.__instance.export_measurements_to_csv(filename)

    def compute_indicators(self, values):
        """
        Returns a dictionnary with indicator name & value

        :type values: list
        :param values: List of measurement used to compute the indicators
        """
        TRIGGER_FACTOR = 2     # trigger is 2 * floor estimation
        TRIGGER_OFFSET = 6     # trigger is floor estimation + 6mA
        BIG_WU_DURATION = 20   # a big wakeup is 20 consecutive high measures

        indicators = []
        nvalues = len(values)

        # get the median value
        median_val = numpy.median(values)
        if median_val < 0: # can happen if there is noise
            median_val = 0

        # compute the trigger for high value detection from the median
        TRIGGER_HIGH = min(TRIGGER_FACTOR * median_val, median_val + TRIGGER_OFFSET)

        # the floor is the median of low-power measures
        condition_low = numpy.less(values, TRIGGER_HIGH)
        low_values = numpy.extract(condition_low, values)
        low_val_median = numpy.median(low_values)

        # impacted values are used to get the impact of wakeups (value - floor)
        impacted_values = numpy.subtract(values, low_val_median)

        # We re-compute the trigger for high value detection
        TRIGGER_HIGH = min(TRIGGER_FACTOR * low_val_median, low_val_median + TRIGGER_OFFSET)

        # wakeup condition : greater than the trigger
        condition_wu = numpy.greater(values, TRIGGER_HIGH)
        impacted_wu_values = numpy.where(condition_wu, impacted_values, 0)

        # get wakeups boundaries by making a diff on the condition list and then the wakeup durations
        diff_array = numpy.append(numpy.insert(numpy.diff(condition_wu), 0, True), True)
        sizes = numpy.diff(numpy.nonzero(diff_array))[0]

        # we have all wakeups in odd or even lists depending if we are beginning with a wakeup or not
        wu_durations = sizes[::2] if condition_wu[0] else sizes[1::2]
        big_wu_durations = numpy.extract(numpy.greater(wu_durations, BIG_WU_DURATION), wu_durations)

        # to filter big wakeups, we generate a list with ones for long serie of wakeup/non-wakeup
        # and make a logical and on the wakeup condition
        condition_big_wu_lists = [numpy.ones(x, dtype=bool) if x >= BIG_WU_DURATION
                                  else numpy.zeros(x, dtype=bool) for x in sizes]
        condition_big_wu = numpy.logical_and(numpy.concatenate(condition_big_wu_lists), condition_wu)
        impacted_big_wu_values = numpy.where(condition_big_wu, impacted_values, 0)
        steady_state_values = numpy.extract(numpy.logical_not(condition_big_wu), values)

        # Append all indicators to the list
        indicators.append(("floor_indicator", low_val_median))
        indicators.append(("nb_wakes_indicator", numpy.size(wu_durations)))
        indicators.append(("wakes_impact_indicator", numpy.average(impacted_wu_values)))
        indicators.append(("steady_state_indicator", numpy.average(steady_state_values)))
        indicators.append(("nb_big_wakes_indicator", numpy.size(big_wu_durations)))
        indicators.append(("big_wakes_impact_indicator", numpy.average(impacted_big_wu_values)))
        big_wu_residency = 100. * sum(big_wu_durations) / nvalues
        indicators.append(("big_wakes_residency", big_wu_residency))
        residency = 100. * sum(wu_durations) / nvalues
        indicators.append(("residency_indicator", 100-residency))

        return indicators

    def filter_plot(self, plot):
        """
        Filter the data from plot to keep only relevant points

        :type plot: list
        :param plot: plot to filter
        :rtype:     list
        :return:    filtered plot
        """
        ret = []
        power_values = [i[1] for i in plot]
        time_values = [i[0] for i in plot]

        # use deviation to know if there is noise or not
        dev = numpy.std(power_values)
        avg = numpy.average(power_values)

        # compute the diff of the sliding average of 3 values
        conv = numpy.convolve(power_values, [0.33, 0.33, 0.33], mode='valid')
        diff_conv = numpy.ediff1d(conv)
        diff_avg = numpy.average(numpy.absolute(diff_conv))

        # detect if the signal is noisy
        if 2 * diff_avg > dev or avg < 0:
            ret.append((round(time_values[0], 2), round(avg, 3)))
            ret.append((round(time_values[-1], 2), round(avg, 3)))
            self._logger.info("Rail is noisy or has negative average, do not compute the graph")
            return ret
        elif dev < 1.:
            trig = self.detect_percent
        elif dev < 1000.0:
            trig = dev * self.detect_percent
        else:
            trig = 1000.0 * self.detect_percent

        # detect the need to add some points if the diff is > to the trigger
        cond_great = numpy.greater(diff_conv, trig)
        cond_less = numpy.less(diff_conv, -trig)
        cond_or = numpy.logical_or(cond_great, cond_less)
        cond_or = numpy.insert(numpy.append(cond_or, [False]), 0, False)

        # increase the trigger to have less points
        while numpy.count_nonzero(cond_or) > 500:
            self._logger.info("Too many points (%d > 500), increase the trigger" % numpy.count_nonzero(cond_or))
            trig = trig * 2.
            cond_great = numpy.greater(diff_conv, trig)
            cond_less = numpy.less(diff_conv, -trig)
            cond_or = numpy.logical_or(cond_great, cond_less)
            cond_or = numpy.insert(numpy.append(cond_or, [False]), 0, False)

        # get the point before and the point after and add them all in the extract condition
        # also take the first point and the last one anyway
        pt_before = numpy.append(cond_or, [False])
        pt = numpy.insert(cond_or, 0, False)
        cond_all = numpy.logical_or(pt_before, pt)
        cond = cond_all[0:len(plot)]
        cond[-1] = True
        cond[0] = True

        # get the indices of elements to keep
        true_indices = numpy.nonzero(cond)

        # iterate over the indices (take (n, n+1) for n in [0..len(true_indices)])
        indices_it = numpy.nditer([numpy.delete(true_indices, -1), numpy.delete(true_indices, 0)])
        ret.append((round(time_values[0], 2), round(power_values[0], 3))) # take the first point
        for (n_pt, n1_pt) in indices_it:
            # if consecutive, just append
            if n_pt+1 == n1_pt:
                ret.append((round(time_values[n1_pt], 2), round(power_values[n1_pt], 3)))
            # if not consecutive, replace the last point (n) by the average value between n and n+1
            else:
                del ret[-1]
                average = numpy.average(power_values[n_pt:n1_pt])
                ret.append((round(time_values[n_pt], 2), round(average, 3)))
                ret.append((round(time_values[n1_pt], 2), round(average, 3)))
        return ret

    def fill_js_data(self, jsOutFile, plot):
        """
        Prints the plot stored in javascript file
        """
        # filter and then fill the file
        outfile = open(jsOutFile, "w")
        try:
            outfile.write("function getRaws() {\n")
            outfile.write("raws = [\n")
            outfile.write("['Time (s)', 'Consumption (mA)'],\n")

            for value in plot:
                js_obj = "[" + str(value[0]) + "," + str(value[1]) + "],\n"
                outfile.write(js_obj)

            outfile.write("]\n")
            outfile.write("return raws\n")
            outfile.write("}\n")
        finally:
            outfile.close()

    def get_plot(self):
        """
        Return the plot points for last measurement (verdict rail)
        """
        return self._plot

    def get_all_rails_plots(self):
        """
        Return the plot points for rail passed as param

        :rtype:     list
        :return:    list representation of the plot
        """
        return self._all_plots

    def export(self, rawDataFile):
        """
        Export to rawDataFile
        """
        filename = os.path.basename(rawDataFile)
        dirname = os.path.dirname(rawDataFile)
        self.__instance.save(dirname, filename)

    def get_plot_from_patfile(self, patDataFile, echPerSecond, valuesForMean):
        """
        Get a plot list from pat binary file

        :type patDataFile: str
        :param patDataFile: Filename to read raw data (.dat file)
        :type echPerSecond: int
        :param echPerSecond: Filename to read raw data (.dat file)
        :type valuesForMean: int
        :param valuesForMean: Filename to read raw data (.dat file)
        :rtype:     list
        :return:    list representation of the plot
        """
        time = 0
        plot = []

        try:
            fvalues = numpy.fromfile(patDataFile, dtype=numpy.float32)
            listnumber = len(fvalues) / valuesForMean
            flen = len(fvalues)
            flen /= int(valuesForMean)
            flen *= int(valuesForMean)

            if listnumber > 0 and valuesForMean > 0:
                resh = numpy.reshape(fvalues[0:flen], (listnumber, valuesForMean))
                average_values = list(numpy.dot(resh, numpy.dot(numpy.ones(valuesForMean), 1./valuesForMean)))
                time_interval = float(valuesForMean) / echPerSecond
            else:
                average_values = list(fvalues)
                time_interval = 1. / echPerSecond

            for val in average_values:
                # If we got enough values, we add a new point to the graph
                time += time_interval
                plot.append([time, val])

            del fvalues, average_values
        except Exception as e:
            self._logger.error("Exception catched while generating power graph : %s" % str(e))

        return plot

    def export_plot(self, rawDataFile, patDataFile, jsOutFile=None):
        """
        Create a plot file

        :type rawDataFile: str
        :param rawDataFile: Filename to write raw data
        :type patDataFile: str
        :param patDataFile: Filename to read raw data (.dat file)
        :type jsOutFile: str
        :param jsOutFile: Filename to write the javascript output
        """
        pwrgraph = etree.Element("PwrGraph")

        # get from the file the Samplerate :
        root_pmdata = etree.parse(rawDataFile).getroot()
        ech_per_second = int(root_pmdata.find("Acquisition_Properties").find("Samplerate").text)

        # Retrieve the data from the file to create a new xml node
        if ech_per_second > 10:
            values_for_mean = ech_per_second / 10 # create 10 points per second
        else: # should be a mistake, but we handle the case
            values_for_mean = 10

        self._logger.info("Export power graph for file : %s" % patDataFile)
        plot = self.get_plot_from_patfile(patDataFile, ech_per_second, values_for_mean)

        if plot:
            self._plot = self.filter_plot(plot)
            if jsOutFile:
                self.fill_js_data(jsOutFile, self._plot)
                graph_dir = os.path.basename(os.path.dirname(jsOutFile))
                pwrgraph.attrib["dir"] = graph_dir
        return pwrgraph

    def export_all_plots(self, rawDataFile, patDataFolder, filterlist):
        """
        Generate a plot for each rail

        :type filterlist: list
        :param filterlist: list of filters (rails which name does not contain any of them will not be exported)
        :type rawDataFile: str
        :param rawDataFile: File to retrieve raw data settings
        :type patDataFolder: str
        :param patDataFolder: Folder to find raw data (.dat files)
        :type railName: str
        :param railName: Name of the rail to compute (compute everything if None)
        """

        # get from the file the Samplerate :
        root_pmdata = etree.parse(rawDataFile).getroot()
        ech_per_second = int(root_pmdata.find("Acquisition_Properties").find("Samplerate").text)

        # Retrieve the data from the file to create a new xml node
        if ech_per_second > 10:
            values_for_mean = ech_per_second / 10 # create 10 points per second
        else: # should be a mistake, but we handle the case
            values_for_mean = 10

        measurements = self.get_measurements()

        for measure in measurements:
            rail = measure.get("name")
            if rail in self._all_plots:
                continue

            # if the rail (or part of its name) is in the list, parse it
            if [x for x in filterlist if x in rail]:
                self._logger.info("Export power graph for rail : %s" % rail)
                patDataFile = os.path.join(patDataFolder, rail + ".dat")
                railPlot = self.get_plot_from_patfile(patDataFile, ech_per_second, values_for_mean)
                if railPlot:
                    plot = self.filter_plot(railPlot)
                    self._all_plots[rail] = plot

    def export_indicators_from_patfile(self, patDataFile, valuesForMean):
        """
        Get the power indicators for the current capture

        :type patDataFile: str
        :param patDataFile: Filename of the .dat file
        :type valuesForMean: int
        :param valuesForMean: number of values to compute local averages
        """
        indic = None
        average_values = []

        try:
            fvalues = numpy.fromfile(patDataFile, dtype=numpy.float32)
            listnumber = len(fvalues) / valuesForMean
            flen = len(fvalues)
            flen /= int(valuesForMean)
            flen *= int(valuesForMean)

            if listnumber > 0 and valuesForMean > 0:
                resh = numpy.reshape(fvalues[0:flen], (listnumber, valuesForMean))
                average_values = numpy.dot(resh, numpy.dot(numpy.ones(valuesForMean), 1./valuesForMean))
            else:
                average_values = list(fvalues)
            indic = self.compute_indicators(average_values)
        except Exception as e:
            self._logger.error("Exception catched while parsing indicators : %s" % str(e))

        return indic

    def get_xml_indicators(self):
        """
        Get an etree element with all the power indicators
        """
        pwrindicators = etree.Element("PwrIndicators")
        for rname in self._all_indicators.keys():
            pwrrail = etree.Element("Rail")
            pwrrail.attrib["name"] = rname
            pwrrail.attrib["unit"] = self.get_rail_unit(rname)
            for name, power in self._all_indicators[rname]:
                pwrrail.attrib[name] = "%0.3f" % power
            pwrindicators.append(pwrrail)
        return pwrindicators

    def export_indicators(self, rawDataFile, patDataFile, rail):
        """
        Get the power indicators for the current capture

        :type rawDataFile: str
        :param rawDataFile: Filename to write raw data
        :type patDataFile: str
        :param patDataFile: .dat file
        :type rail: str
        :param rail: Rail to export
        """
        if rail not in self._all_indicators.keys():
            # get from the file the Samplerate :
            root_pmdata = etree.parse(rawDataFile).getroot()
            ech_per_second = int(root_pmdata.find("Acquisition_Properties").find("Samplerate").text)
            values_for_avg = ech_per_second / 100 # average value for 10ms

            self._logger.info("Compute indicators for rail : %s" % rail)
            indic = self.export_indicators_from_patfile(patDataFile, values_for_avg)
            if indic:
                self._all_indicators[rail] = indic

        return self.get_xml_indicators()

    def export_all_indicators(self, rawDataFile, patDataFolder, filterlist=None):
        """
        Get the power indicators for the current capture

        :type rawDataFile: str
        :param rawDataFile: Filename to write raw data
        :type patDataFolder: str
        :param patDataFolder: Folder to find raw data (.dat files)
        """
        # get from the file the Samplerate :
        root_pmdata = etree.parse(rawDataFile).getroot()
        ech_per_second = int(root_pmdata.find("Acquisition_Properties").find("Samplerate").text)
        values_for_avg = ech_per_second / 100 # average value for 10ms

        measurements = self.get_measurements()

        for measure in measurements:
            indic = None
            railname = measure.get("name")
            if not railname or railname in self._all_indicators.keys():
                continue

            # if the rail (or part of its name) is in the list, parse it
            if not filterlist or [x for x in filterlist if x in railname]:
                self._logger.info("Compute indicators for rail : %s" % railname)
                patDataFile = os.path.join(patDataFolder, railname + ".dat")
                indic = self.export_indicators_from_patfile(patDataFile, values_for_avg)
                if indic:
                    self._all_indicators[railname] = indic

        return self.get_xml_indicators()

    def clear_data(self):
        """
        Clear all the plot and indicators data computed previously
        """
        self._plot = None
        self._all_plots = {}
        self._all_indicators = {}
