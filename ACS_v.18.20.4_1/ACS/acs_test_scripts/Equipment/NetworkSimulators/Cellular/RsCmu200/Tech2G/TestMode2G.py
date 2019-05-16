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
:summary: test mode 2G implementation for RS CMU200
simulators
:since: 05/04/2011
:author: ymorel
"""

from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ITestMode2G import ITestMode2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech2G import WTestMode2G as W


class TestMode2G(ITestMode2G):

    """
    Test mode 2G implementation for RS CMU200
    """

    RSCMU_NAN = 9.91e+37
    RSCMU_NAN_VI_INT32 = 0x80000000

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        ITestMode2G.__init__(self)
        self.__root = root

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def __convert_to_nan(self, value):
        """
        Converts value to float("NAN") if value is equivalent to NAN.
        """
        is_nan = False

        if isinstance(value, int):
            is_nan = (value == self.RSCMU_NAN_VI_INT32)
        elif isinstance(value, float):
            is_nan = (value == self.RSCMU_NAN)

        if is_nan:
            return float("NAN")
        else:
            return value

    def get_root(self):
        """
        Get the root object of the equipment
        :rtype: RsCmu200
        :return: the root object of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def configure_ber_measurement(self, nb_frames, ref_level):
        """
        Configures bit error rate (BER) measurement
        :type nb_frames: integer
        :param nb_frames: number of frames to use for the BER measurement.
        :type ref_level: double
        :param ref_level: the cell power reference to use for the measurement
        """
        (err, msg) = W.ConfigureBERMeasurement(
            self.get_root(),
            nb_frames,
            ref_level)
        self.__error_check(err, msg)

    def get_ber(self):
        """
        Wraps to GetBER 2G driver function
        :raise TestEquipmentException: failed to call GetBER driver function
        :rtype: double
        :return: the measured BER (Bit Error Rate)
        """
        (err, ber, msg) = W.GetBER(self.get_root())
        self.__error_check(err, msg)
        self.get_logger().info("Measured bit error rate: %f", ber)
        return ber

    def get_rssi(self):
        """
        Wraps to GetRSSI 2G driver function
        :raise TestEquipmentException: failed to call GetRSSI driver function
        :rtype: long
        :return: the measured RSSI (Received Signal Strength Indicator)
        """
        (err, rssi, msg) = W.GetRSSI(self.get_root())
        self.__error_check(err, msg)
        self.get_logger().info("Measured RX level: %f", rssi)
        return rssi

    def check_rssi_connectivity(self, target, margin):
        """
        Checks DUT to network simulator connectivity through RSSI measurement.
        The RSSI value retrieved is compared to target +/- margin.
        :type target: integer
        :param target: value of the expected RSSI (in dBm).
        :type margin: integer
        :param margin: acceptable margin.
        :raise TestEquipmentException: An internal problem occurs.
        """
        positive_margin = margin
        if margin < 0:
            positive_margin = -margin
        retrieved = self.get_rssi()
        rssi_min = target - positive_margin
        rssi_max = target + positive_margin
        if (retrieved < rssi_min) or (retrieved > rssi_max):
            msg = "Connectivity is too bad : " + str(retrieved) + "dBm"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTIVITY_ERROR, msg)

    def set_modulation_spectrum_limits(self, modulation, limits):
        """
        Sets limits for modulation spectrum measurements.
        :type modulation: str
        :param modulation: the modulation type to use
            - "GSM"
            - "EDGE"
            - "ALL": sets limits for all available modulation
        :type limits: dict
        :param limits: a dictionary containing the limits for each following keys:
            - "0,10MHZ", "0,20MHZ", "0,25MHZ", "0,60MHZ", "0,80MHZ", "1,00MHZ",
              "1,20MHZ", "1,40MHZ", "1,60MHZ","1,80MHZ"
            - values: tuple containing 3 float in that order:
                - reference power in dBm (--> value parameter)
                - low limit in dB (--> minPower parameter)
                - high limit in dB (--> maxPower parameter)
        """
        if modulation == "ALL":
            modulations = ["GSM", "EDGE"]
        else:
            modulations = [modulation]
        measure_points = {
            "0,10MHZ": 1, "0,20MHZ": 2, "0,25MHZ": 3, "0,40MHZ": 4,
            "0,60MHZ": 5, "0,80MHZ": 6, "1,00MHZ": 7, "1,20MHZ": 8,
            "1,40MHZ": 9, "1,60MHZ": 10, "1,80MHZ": 11}
        for mod in modulations:
            for key in limits.keys():
                (ref_power, min_power, max_power) = limits[key]
                (err, msg) = W.SetSpectrumModulationLimitsLine(
                    self.get_root(),
                    mod,
                    measure_points[key],
                    "ON",
                    min_power,
                    max_power,
                    ref_power)
                self.__error_check(err, msg)

    def set_switching_spectrum_limits(self, modulation, limits):
        """
        Sets the limits for spectrum switching measurements
        :type modulation: str
        :param modulation: the modulation type to use
            - "GSM"
            - "EDGE"
            - "ALL" : sets the limits for all available modulations
        :type limits: dict
        :param limits: a dictionary containing the limits to set for each power level number
            - keys are tuple containing 2 integers: (power level number, power level).
            - values: list of 5 values:
                - the first is a str corresponding to 'enable' parameter
                    - "OFF"
                    - "ON"
            - the four other values are integers representing limit for 0.40MHz, 0.60MHz,
              1.20MHz and 1.80MHz.
        """
        if modulation == "ALL":
            modulations = ["GSM", "EDGE"]
        else:
            modulations = [modulation]
        keys = sorted(limits.keys())
        keys.reverse()
        number_of_keys = len(keys)
        # Check the number of keys in dictionary (expected: 10)
        if number_of_keys != 10:
            msg = "Unexpected number of keys in dictionary, " \
                "got %d expected 10." % number_of_keys
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
        # Check dictionary values
        self._check_power_levels(limits)
        # Reset limits to default values
        self._reset_switching_spectrum_limits(modulation)
        # Set the limits
        for mod in modulations:
            for key in keys:
                (pow_lvl_num, pow_lvl) = key
                (enable, l_040MHz, l_060MHz, l_120MHz, l_180MHz) = limits[key]
                (err, msg) = W.SetSpectrumSwitchingLimitsLine(
                    self.get_root(),
                    mod,
                    pow_lvl_num,
                    pow_lvl,
                    enable,
                    l_040MHz,
                    l_060MHz,
                    l_120MHz,
                    l_180MHz)
                self.__error_check(err, msg)

    def _check_power_levels(self, limits):
        """
        Checks that the power levels are given in descending order (when enumerating
        the power level number in an ascending order).
        Each value shall also fulfill the following condition:
            - value(n) <= value(n - 1) - 1
        :raise TestEquipmentException: if at least one value does not met the above requirements.
        """
        # Sort the keys
        keys = sorted(limits.keys())
        previous = None
        lower = True
        diff = None
        # Check power values
        for key in keys:
            (pow_lvl_num, pow_lvl) = key
            if previous is not None:
                lower = pow_lvl < previous
                if lower:
                    diff = previous - pow_lvl
            else:
                previous = pow_lvl
            if not lower or diff < 1 and diff is not None:
                msg = "Power level %s (power level number: %s) " \
                    "should be <= %s - 1" \
                    % (str(pow_lvl), str(pow_lvl_num), str(previous))
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)
            else:
                previous = pow_lvl

    def _reset_switching_spectrum_limits(self, modulation="ALL"):
        """
        Sets the limits for spectrum switching measurements to default values.
        """
        if modulation == "ALL":
            modulations = ["GSM", "EDGE"]
        else:
            modulations = [modulation]
        limits = {
            (1, 99.9): ["ON", 0, 0, 0, 0],
            (2, 98.9): ["ON", 0, 0, 0, 0],
            (3, 97.9): ["ON", 0, 0, 0, 0],
            (4, 96.9): ["ON", 0, 0, 0, 0],
            (5, 95.9): ["ON", 0, 0, 0, 0],
            (6, 94.9): ["ON", 0, 0, 0, 0],
            (7, 93.9): ["ON", 0, 0, 0, 0],
            (8, 92.9): ["ON", 0, 0, 0, 0],
            (9, 91.9): ["ON", 0, 0, 0, 0],
            (10, 90.9): ["ON", 0, 0, 0, 0]}
        keys = sorted(limits.keys())
        for mod in modulations:
            for key in keys:
                (pow_lvl_num, pow_lvl) = key
                (enable, l_040MHz, l_060MHz, l_120MHz, l_180MHz) = limits[key]
                (err, msg) = W.SetSpectrumSwitchingLimitsLine(
                    self.get_root(),
                    mod,
                    pow_lvl_num,
                    pow_lvl,
                    enable,
                    l_040MHz,
                    l_060MHz,
                    l_120MHz,
                    l_180MHz)
                self.__error_check(err, msg)

    def get_gsm_power(self):
        """
        This function starts a single shot measurement and returns the results
        :rtype: dict
        :return: a dictionary containing the following measurements:
            - key : "AVG_BURST_POWER", value type: float
            - key : "PEAK_BURST_POWER", value type: float
            - key : "PCL", value type: integer
            - key : "TIMING_ADV_ERROR", value type: float
            - key : "BURST_OUT_OF_TOLERANCE", value type: float
            - key : "BURST_MATCHING", value type: Global.SUCCESS or Global.FAiLURE
            - key : "AVG_BURST_POWER", value type: float
        """
        (err, powers, msg) = W.ReadPower(self.get_root(), "GSM")
        self.__error_check(err, msg)

        verdict = powers["BURST_MATCHING"]
        if powers["BURST_MATCHING"] == "OK":
            powers["BURST_MATCHING"] = Global.SUCCESS
        else:
            powers["BURST_MATCHING"] = Global.FAILURE

        # Convert invalid results to NAN
        for key in powers.keys():
            powers[key] = self.__convert_to_nan(powers[key])

        # Log results in debug mode
        self.get_logger().debug("GSM powers:")
        for key in powers.keys():
            if key == "BURST_MATCHING":
                self.get_logger().debug(
                    "\t%s: %s (%s)",
                    key,
                    str(powers[key]),
                    str(verdict))
            else:
                self.get_logger().debug("\t%s: %s", key, str(powers[key]))

        return powers

    def get_edge_power(self):
        """
        This function starts a single shot measurement and returns the results
        :rtype: dict
        :return: a dictionary containing the following measurements:
            - key : "AVG_BURST_POWER", value type: float
            - key : "PEAK_BURST_POWER", value type: float
            - key : "PCL", value type: integer
            - key : "TIMING_ADV_ERROR", value type: float
            - key : "BURST_OUT_OF_TOLERANCE", value type: float
            - key : "BURST_MATCHING", value type: Global.SUCCESS or Global.FAiLURE
            - key : "AVG_BURST_POWER", value type: float
        """
        (err, powers, msg) = W.ReadPower(self.get_root(), "EDGE")
        self.__error_check(err, msg)

        verdict = powers["BURST_MATCHING"]
        if powers["BURST_MATCHING"] == "OK":
            powers["BURST_MATCHING"] = Global.SUCCESS
        else:
            powers["BURST_MATCHING"] = Global.FAILURE

        # Convert invalid results to NAN
        for key in powers.keys():
            powers[key] = self.__convert_to_nan(powers[key])

        # Log results in debug mode
        self.get_logger().debug("EDGE powers:")
        for key in powers.keys():
            if key == "BURST_MATCHING":
                self.get_logger().debug(
                    "\t%s: %s (%s)",
                    key,
                    str(powers[key]),
                    str(verdict))
            else:
                self.get_logger().debug("\t%s: %s", key, str(powers[key]))

        return powers

    def get_gsm_avg_modulation_meas(self):
        """
        This function starts a single shot measurement and returns the results.
        :rtype: dict
        :return: a dictionary containing the following keys
            - "PHASE_ERR_PEAK": float
            - "PHASE_ERR_RMS": float
            - "OFFSET": float
            - "IQ_IMBALANCE": float
            - "FREQUENCY_ERR": float
            - "AVG_BURST_POWER": float
            - "BURST_OUT_OF_TOL": float
        """
        (err, results, msg) = W.ReadModXper(self.get_root(), "AVERAGE")
        self.__error_check(err, msg)

        # Convert invalid results to NAN
        for key in results.keys():
            results[key] = self.__convert_to_nan(results[key])

        # Log results in debug mode
        self.get_logger().debug("GSM average modulation results (phase error):")
        for key in results.keys():
            self.get_logger().debug("\t%s: %s", key, str(results[key]))

        return results

    def get_gsm_modulation_spectrum_verdict(self):
        """
        Reads spectrum modulation measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        (err, ref_pow, match, msg) = W.FetchSpectrumModulationVerdict(
            self.get_root(), "GSM")
        self.__error_check(err, msg)
        if match == "OK":
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        # Convert invalid results to NAN
        ref_pow = self.__convert_to_nan(ref_pow)

        # Log results in debug mode
        self.get_logger().debug("\tVerdict: %s (%s)", str(verdict), str(match))
        self.get_logger().debug("\tReference power: %f", ref_pow)

        return verdict, ref_pow

    def get_gsm_modulation_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
            a dictionary containing the power level as float values for each following key:
                - "0MHZ"     | "-0,10MHZ" | "-0,20MHZ" | "-0,25MHZ" | "-0,40MHZ" |
                  "-0,60MHZ" | "-0,80MHZ" | "-1,00MHZ" | "-1,20MHZ" | "-1,40MHZ" |
                  "-1,60MHZ" | "-1,80MHZ" | "0,10MHZ"  | "0,20MHZ"  | "0,25MHZ"  |
                  "0,40MHZ"  | "0,60MHZ"  | "0,80MHZ"  | "1,00MHZ"  | "1,20MHZ"  |
                  "1,40MHZ"  | "1,60MHZ"  | "1,80MHZ"
        """
        (err, power_levels, msg) = W.ReadSpectrumModulation(
            self.get_root(), "GSM")
        self.__error_check(err, msg)

        # Convert invalid results to NAN
        for key in power_levels.keys():
            power_levels[key] = self.__convert_to_nan(power_levels[key])

        # Log results in debug mode
        for key in power_levels.keys():
            self.get_logger().debug("\t%s: %s", key, str(power_levels[key]))

        return power_levels

    def get_gsm_switching_spectrum_verdict(self):
        """
        Reads spectrum switching measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        (err, ref_pow, match, msg) = W.FetchSpectrumSwitchingVerdict(
            self.get_root(), "GSM")
        self.__error_check(err, msg)
        if match == "OK":
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        # Convert invalid results to NAN
        ref_pow = self.__convert_to_nan(ref_pow)

        # Log results in debug mode
        self.get_logger().debug("\tVerdict: %s (%s)", str(verdict), str(match))
        self.get_logger().debug("\tReference power: %f", ref_pow)

        return verdict, ref_pow

    def get_gsm_switching_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
        - dict: a dictionary containing the power as float values for each following key:
            - "-1800KHZ", "-1200KHZ", "-600KHZ", "-400KHZ", "0KHZ", "400KHZ",
              "600KHZ", "1200KHZ", "1800KHZ"
            - "NAN" is returned at the disabled points
        """
        (err, powers, msg) = W.ReadSpectrumSwitching(
            self.get_root(), "GSM")
        self.__error_check(err, msg)

        # Convert invalid results to NAN
        for key in powers.keys():
            powers[key] = self.__convert_to_nan(powers[key])

        # Log results in debug mode
        for key in powers.keys():
            self.get_logger().debug("\t%s: %s", key, str(powers[key]))

        return powers

    def get_edge_avg_modulation_meas(self):
        """
        Starts single shot measurements and return results
        :rtype: dict
        :return: a dictionary containing the following keys:
            - key: "95EVM"            => double: error vector magnitude
            - key: "EVM_PEAK"         => double: error vector magnitude peak
            - key: "EVM_RMS"          => double: error vector magnitude RMS
            - key: "OFFSET"           => double: offset
            - key: "IQ_IMBALANCE"     => double: IQ imbalance
            - key: "FREQUENCY_ERR"    => double: frequency error
            - key: "AVG_BURST_POWER"  => double: average burst power current
            - key: "BURST_OUT_OF_TOL" => double: burst out of tolerance
        """
        (err, results, msg) = W.ReadEdgeModulationEvm(
            self.get_root(), "AVERAGE")
        self.__error_check(err, msg)

        # Convert invalid results to NAN
        for key in results.keys():
            results[key] = self.__convert_to_nan(results[key])

        # Log results in debug mode
        for key in results.keys():
            self.get_logger().debug("\t%s: %s", key, str(results[key]))

        return results

    def get_edge_modulation_spectrum_verdict(self):
        """
        Reads spectrum modulation measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        (err, ref_pow, match, msg) = W.FetchSpectrumModulationVerdict(
            self.get_root(), "EDGE")
        self.__error_check(err, msg)
        if match == "OK":
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        # Convert invalid results to NAN
        ref_pow = self.__convert_to_nan(ref_pow)

        # Log results in debug mode
        self.get_logger().debug("\tVerdict: %s (%s)", str(verdict), str(match))
        self.get_logger().debug("\tReference power: %f", ref_pow)

        return verdict, ref_pow

    def get_edge_modulation_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
            a dictionary containing the power level as float values for each following key:
                - "0MHZ"     | "-0,10MHZ" | "-0,20MHZ" | "-0,25MHZ" | "-0,40MHZ" |
                  "-0,60MHZ" | "-0,80MHZ" | "-1,00MHZ" | "-1,20MHZ" | "-1,40MHZ" |
                  "-1,60MHZ" | "-1,80MHZ" | "0,10MHZ"  | "0,20MHZ"  | "0,25MHZ"  |
                  "0,40MHZ"  | "0,60MHZ"  | "0,80MHZ"  | "1,00MHZ"  | "1,20MHZ"  |
                  "1,40MHZ"  | "1,60MHZ"  | "1,80MHZ"
        """
        (err, power_levels, msg) = W.ReadSpectrumModulation(
            self.get_root(), "EDGE")
        self.__error_check(err, msg)

        # Convert invalid results to NAN
        for key in power_levels.keys():
            power_levels[key] = self.__convert_to_nan(power_levels[key])

        # Log results in debug mode
        for key in power_levels.keys():
            self.get_logger().debug("\t%s: %s", key, str(power_levels[key]))

        return power_levels

    def get_edge_switching_spectrum_verdict(self):
        """
        Reads spectrum switching measurement results and compares them with tolerance values
        :rtype: tuple
        :return:
            - global spectrum modulation verdict:
                - Global.SUCCESS
                - Global.FAILURE
            - reference power as a float
        """
        (err, ref_pow, match, msg) = W.FetchSpectrumSwitchingVerdict(
            self.get_root(), "EDGE")
        self.__error_check(err, msg)
        if match == "OK":
            verdict = Global.SUCCESS
        else:
            verdict = Global.FAILURE

        # Convert invalid results to NAN
        ref_pow = self.__convert_to_nan(ref_pow)

        # Log results in debug mode
        self.get_logger().debug("\tVerdict: %s (%s)", str(verdict), str(match))
        self.get_logger().debug("\tReference power: %f", ref_pow)

        return verdict, ref_pow

    def get_edge_switching_spectrum(self):
        """
        Starts single shot measurement and return results
        :return:
        - dict: a dictionary containing the power as float values for each following key:
            - "-1800KHZ", "-1200KHZ", "-600KHZ", "-400KHZ", "0KHZ", "400KHZ",
              "600KHZ", "1200KHZ", "1800KHZ"
            - "NAN" is returned at the disabled points
        """
        (err, powers, msg) = W.ReadSpectrumSwitching(
            self.get_root(), "EDGE")
        self.__error_check(err, msg)

        # Convert invalid results to NAN
        for key in powers.keys():
            powers[key] = self.__convert_to_nan(powers[key])

        # Log results in debug mode
        for key in powers.keys():
            self.get_logger().debug("\t%s: %s", key, str(powers[key]))

        return powers

    def get_pdat_ber_reference_level(self):
        """
        Returns the I{BER} TCH reference level.

        :rtype: float
        :return: the reference level for I{BER} measurements.
        """
        (err, level, msg) = W.GetBerTchReferenceLevel(self.get_root())
        self.__error_check(err, msg)
        return level

    def configure_pdat_ber_slots(self, slot_level_array):
        """
        Configures the I{BER} packet data slot levels with the given values.

        :type slot_level_array: list
        :param slot_level_array: a list of 8 slot levels

        :rtype: None
        """
        for slot in range(len(slot_level_array)):
            slot_level = slot_level_array[slot]
            (err, msg) = W.ConfigurePdatBerTchSlotPowerLevel(
                self.get_root(),
                "TS1",
                "SINGLE_SHOT",
                slot,
                slot_level)
            self.__error_check(err, msg)
