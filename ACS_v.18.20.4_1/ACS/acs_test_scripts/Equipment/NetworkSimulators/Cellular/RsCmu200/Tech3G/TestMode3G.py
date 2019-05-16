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
:summary: test mode 3G implementation for RS CMU200
simulators
:since: 06/04/2011
:author: ymorel
"""

from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ITestMode3G import ITestMode3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech3G import WTestMode3G as W


class TestMode3G(ITestMode3G):

    """
    Test mode 3G implementation for RS CMU200
    """

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        ITestMode3G.__init__(self)
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

    def set_ber_block_number(self, block_number):
        """
        Sets bit error rate block number for BER measurement
        :type block_number: integer
        :param block_number: the number of blocks to set: an integer from
            1 to 50000
        """
        (err, msg) = W.SetBERBlockNumber(self.get_root(), block_number)
        self.__error_check(err, msg)

    def get_ber(self):
        """
        Gets bit error rate
        :rtype: double
        :return: the measured BER (Bit Error Rate)
        """
        (err, ber, msg) = W.GetBER(self.get_root())
        self.__error_check(err, msg)
        return ber

    def configure_rssi_measurement(self):
        """
        Configures RSSI measurement
        """
        self.get_logger().info("Configure equipment for RSSI measurement")
        (err, msg) = W.ConfigureRSSIMeasurement(self.get_root())
        self.__error_check(err, msg)

    def get_rssi(self):
        """
        Gets RSSI
        :rtype: double
        :return: the measured RSSI (Received Signal Strength Indicator)
        """
        (err, rssi, msg) = W.GetRSSI(self.get_root())
        self.__error_check(err, msg)
        return rssi

    def check_rssi_connectivity(self, target, margin):
        """
        Checks DUT to network simulator connectivity thanks to RSSI measurement.
        The RSSI value retrieved is compared to target +/- margin.
        :type target: integer
        :param target: value of the RSSI expected (in dBm).
        :type margin: integer
        :param margin: acceptable margin.
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

    def set_avg_max_pow_limits(
            self,
            peak_limit,
            rms_limit,
            peak_tolerance,
            rms_tolerance):
        """
        Sets maximum power limits for the average measurements.
        :type peak_limit: float
        :param peak_limit: the peak limit to set
        :type rms_limit: float
        :param rms_limit: the RMS limit to set
        :type peak_tolerance: tuple
        :param peak_tolerance:
            - float: peak lower tolerance
            - float: peak upper tolerance
        :type rms_tolerance: tuple
        :param rms_tolerance:
            - float: RMS lower tolerance
            - float: RMS upper tolerance
        """
        # Disable use of default limits
        (err, msg) = W.SetDefaultPowerMaximumLimits3G(self.get_root(), "ON")
        self.__error_check(err, msg)
        # Configure limits, peak and RMS
        (err, msg) = W.ConfigurePowerMaximumLimitsRated3G(
            self.get_root(),
            "AVERAGE",
            peak_limit,
            rms_limit)
        self.__error_check(err, msg)
        # Configure upper tolerance
        (err, msg) = W.ConfigurePowerMaximumLimitsUpper3G(
            self.get_root(),
            "AVERAGE",
            peak_tolerance[1],
            rms_tolerance[1])
        self.__error_check(err, msg)
        # Configure lower tolerance
        (err, msg) = W.ConfigurePowerMaximumLimitsLower3G(
            self.get_root(),
            "AVERAGE",
            peak_tolerance[0],
            rms_tolerance[0])
        self.__error_check(err, msg)

    def set_avg_min_pow_limits(self, peak_limit, rms_limit):
        """
        Sets minimum power limits for the average measurements.
        :type peak_limit: float
        :param peak_limit: the peak limlit to set
        :type rms_limit: float
        :param rms_limit: the RMS limit to set
        """
        # Disable use of default limits
        (err, msg) = W.SetDefaultPowerMinimumLimits3G(self.get_root(), "ON")
        self.__error_check(err, msg)
        # Configure limits, peak and RMS
        (err, msg) = W.ConfigurePowerMinimumLimitsUpper3G(
            self.get_root(),
            "AVERAGE",
            peak_limit,
            rms_limit)
        self.__error_check(err, msg)

    def set_modulation_limits(
            self,
            peak_evm,
            rms_evm,
            offset,
            frequency_error,
            dom_error):
        """
        Set EVM (Error Vector Modulation) limits and frequency error limit.
        :type peak_evm: float
        :param peak_evm:
        :type rms_evm: float
        :param rms_evm:
        :type offset: float
        :param offset:
        :type frequency_error: float
        :param frequency_error:
        :type dom_error: float
        :param dom_error:
        """
        # Disable default value
        (err, msg) = W.SetDefaultEvmWcdmaLimits3G(self.get_root(), "ON")
        self.__error_check(err, msg)
        # Configure average modulation limits
        (err, msg) = W.ConfigureEvmWcdmaLimitsUpper3G(
            self.get_root(),
            "AVERAGE",
            peak_evm,
            rms_evm,
            17.5,
            13.5,
            10,
            10,
            offset,
            - 10,
            frequency_error,
            0.9440,
            dom_error,
            0.06)
        self.__error_check(err, msg)
        # Configure minimum and maximum modulation limits
        (err, msg) = W.ConfigureEvmWcdmaLimitsUpper3G(
            self.get_root(),
            "CURRENT",
            17.5,
            13.5,
            17.5,
            13.5,
            10,
            10,
            - 30,
            - 10,
            frequency_error,
            0.95,
            - 15,
            0.06)
        self.__error_check(err, msg)

    def set_aclr_avg_limits(self, aclr_5_rel, aclr_10_rel, aclr_abs, obw_lim):
        """
        Set ACLR and OBW (Occupied bandwidth) limits.
        :type aclr_5_rel: float
        :param aclr_5_rel: relative limit for ACLR +5MHz and ACLR -5 MHz
        :type aclr_10_rel: float
        :param aclr_10_rel: relative limit for ACLR +10MHz and ACLR -10 MHz
        :type aclr_abs: float
        :param aclr_abs: absolute limit for ACLR
        :type obw_lim: float
        :param obw_lim: occupied bandwidth limit
        """
        # Disable default values
        (err, msg) = W.SetDefaultSpectrumMfftLimits3G(self.get_root(), "ON")
        self.__error_check(err, msg)
        # Configure ACLR -10MHz (relative):
        (err, msg) = W.ConfigureSpectrumMfftLimitsRelative3G(
            self.get_root(),
            "AVERAGE",
            - 2,
            aclr_10_rel,
            "ON")
        self.__error_check(err, msg)
        # Configure ACLR -5MHz (relative):
        (err, msg) = W.ConfigureSpectrumMfftLimitsRelative3G(
            self.get_root(),
            "AVERAGE",
            - 1,
            aclr_5_rel,
            "ON")
        self.__error_check(err, msg)
        # Configure ACLR +5MHz (relative):
        (err, msg) = W.ConfigureSpectrumMfftLimitsRelative3G(
            self.get_root(),
            "AVERAGE",
            1,
            aclr_5_rel,
            "ON")
        self.__error_check(err, msg)
        # Configure ACLR +10MHz (relative):
        (err, msg) = W.ConfigureSpectrumMfftLimitsRelative3G(
            self.get_root(),
            "AVERAGE",
            2,
            aclr_10_rel,
            "ON")
        self.__error_check(err, msg)
        # Configure ACLR (absolute mode):
        (err, msg) = W.ConfigureSpectrumMfftLimitsAbsolute3G(
            self.get_root(),
            "AVERAGE",
            aclr_abs,
            "ON")
        self.__error_check(err, msg)
        # Configure OBW (occupied BandWidth):
        (err, msg) = W.ConfigureSpectrumMfftLimitsObw3G(
            self.get_root(),
            "AVERAGE",
            obw_lim,
            "ON")
        self.__error_check(err, msg)

    def set_sem_avg_limits(self, limits):
        """
        Sets SEM (Spectrum Emission Mask) limits.
        :type limits: list of doubles
        :param limits: list of 6 values defining 8 sections:
            - 1st & 2nd values: define section 12, 5-8, 5MHz and its symmetric
            - 2nd & 3rd values: define section 8,  5-7, 5MHz and its symmetric
            - 3rd & 4th values: define section 7,  5-3, 5MHz and its symmetric
            - 5th & 6th values: define section 3,  5-2, 5MHz and its symmetric
        """
        # Configure SEM limits:
        (err, msg) = W.ConfigureSpectrumEmaskLimitsRelative3G(
            self.get_root(),
            "AVERAGE",
            limits[0],
            limits[1],
            limits[2],
            limits[3],
            limits[4],
            limits[5])
        self.__error_check(err, msg)
        # Enable the fourth section:
        (err, msg) = W.EnableSpectrumEmaskLimitsRelative3G(
            self.get_root(),
            "AVERAGE",
            "ON",
            "ON")
        self.__error_check(err, msg)
        # Enable absolute limits (conform to 3GPP):
        (err, msg) = W.EnableSpectrumEmaskLimitsAsolute3G(
            self.get_root(),
            "AVERAGE",
            "ON")
        self.__error_check(err, msg)

    def get_avg_max_power(self):
        """
        Performs maximum power measurement in order to retrieve the average
        maximum power measured (in dBm).
        :rtype: double
        :return: the average maximum power measured
        """
        (err, powers, msg) = W.ReadPower3G(self.get_root(), "MAX")
        self.__error_check(err, msg)
        powers_keys = powers.keys()
        if "RMS_POW_AVG" in powers_keys:
            return powers["RMS_POW_AVG"]
        else:
            msg = "Failed to measure maximum average power"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

    def get_avg_min_power(self):
        """
        Performs minimum power measurement in order to retrieve the average
        minimum power measured (in dBm).
        :rtype: double
        :return: the average minimum power measured
        """
        (err, powers, msg) = W.ReadPower3G(self.get_root(), "MIN")
        self.__error_check(err, msg)
        powers_keys = powers.keys()
        if "RMS_POW_AVG" in powers_keys:
            return powers["RMS_POW_AVG"]
        else:
            msg = "Failed to measure minimum average power"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

    def get_dut_avg_power(self):
        """
        Performs power measurement in order to retrieve the average
        power measured (in dBm).
        """
        (err, powers, msg) = W.ReadSpectrumEmask3G(self.get_root())
        self.__error_check(err, msg)
        powers_keys = powers.keys()
        if "REF_POW_AVG" in powers_keys:
            return powers["REF_POW_AVG"]
        else:
            msg = "Failed to measure DUT average power"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

    def get_modulation_meas(self):
        """
        Performs EVM (Error Vector Modulation) measurement and retrieve
        the RMS EVM and the frequency error values.
        :rtype: dict
        :return: a dictionnary containing the following values:
            - key: "PEAK_EVM": average peak EVM
            - key: "RMS_EVM": average RMS EVM
            - key: "OFFSET": average I/Q origin offset
            - key: "FREQUENCY_ERROR": minimum/maximum frequency error
            - key: "DOM_ERROR": average peak code DOM error
        """
        # Get average measurements
        (err, results, msg) = W.ReadEvmWcdma3G(self.get_root(), 0, "AVERAGE")
        self.__error_check(err, msg)
        results_keys = results.keys()
        modulation_meas = {}
        avg_keys = ["PEAK_EVM", "RMS_EVM", "OFFSET", "DOM_ERROR"]
        for key in avg_keys:
            if key in results_keys:
                modulation_meas[key] = results[key]
            else:
                msg = "Failed to get modulation measurements"
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)
        # Get Min/Max measurements for frequency error only
        (err, results, msg) = W.ReadEvmWcdma3G(self.get_root(), 0, "MAXMIN")
        self.__error_check(err, msg)
        results_keys = results.keys()
        if "FREQUENCY_ERROR" in results_keys:
            modulation_meas["FREQUENCY_ERROR"] = results["FREQUENCY_ERROR"]
        else:
            msg = "Failed to get modulation measurements"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

        return modulation_meas

    def get_sem_avg_meas(self):
        """
        Performs SEM average measurements
        :rtype: dict
        :return: a dictionary containg the following values:
            - key: "-12.5_-8.5MHz" : margin value of -12.5_-8.5MHz interval
            - key: "-8.5_-7.5MHz"  : margin value of -8.5_-7.5MHz interval
            - key: "-7.5_-3.5MHz"  : margin value of -7.5_-3.5MHz interval
            - key: "-3.5_-2.5MHz"  : margin value of -3.5_-2.5MHz interval
            - key: "2.5_3.5MHz"    : margin value of 2.5_3.5MHz interval
            - key: "3.5_7.5MHz"    : margin value of 3.5_7.5MHz interval
            - key: "7.5_8.5MHz"    : margin value of 7.5_8.5MHz interval
            - key: "8.5_12.5MHz"   : margin value of 8.5_12.5MHz interval
        """
        (err, results, msg) = W.ReadSpectrumEmaskMargin3G(
            self.get_root(),
            "AVERAGE")
        self.__error_check(err, msg)
        return results

    def get_aclr_avg_meas(self):
        """
        Performs ACLR and OBW power measurements
        :rtype: dict
        :return: a dictionary containing the following keys:
            - "RELATIVE_ACLR-10MHz"
            - "RELATIVE_ACLR-5MHz"
            - "RELATIVE_ACLR+5MHz"
            - "RELATIVE_ACLR+10MHz"
            - "OBW"
            - "ABSOLUTE_ACLR-10MHz"
            - "ABSOLUTE_ACLR-5MHz"
            - "ABSOLUTE_ACLR+5MHz"
            - "ABSOLUTE_ACLR+10MHz"
        """
        # Perform relative measurements
        (err, rel_powers, msg) = W.ReadSpectrumMfft3G(self.get_root(), "REL")
        self.__error_check(err, msg)
        # Performs absolute measurements
        (err, abs_powers, msg) = W.ReadSpectrumMfft3G(self.get_root(), "ABS")
        self.__error_check(err, msg)
        results = {}
        # Check and get needed relative measurements
        if  rel_powers[6] != "NAN" and rel_powers[7] != "NAN" and \
            rel_powers[8] != "NAN" and rel_powers[9] != "NAN" and \
                rel_powers[15] != "NAN":
            results["RELATIVE_ACLR-10MHz"] = float(rel_powers[6])
            results["RELATIVE_ACLR-5MHz"] = float(rel_powers[7])
            results["RELATIVE_ACLR+5MHz"] = float(rel_powers[8])
            results["RELATIVE_ACLR+10MHz"] = float(rel_powers[9])
            results["OBW"] = float(rel_powers[15]) / 1e+006
        else:
            msg = "Invalid relative ACLR measure"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

        # Check and get needed absolute measurements
        if  abs_powers[6] != "NAN" and abs_powers[7] != "NAN" and \
                abs_powers[8] != "NAN" and abs_powers[9] != "NAN":
            results["ABSOLUTE_ACLR-10MHz"] = float(abs_powers[6])
            results["ABSOLUTE_ACLR-5MHz"] = float(abs_powers[7])
            results["ABSOLUTE_ACLR+5MHz"] = float(abs_powers[8])
            results["ABSOLUTE_ACLR+10MHz"] = float(abs_powers[9])
        else:
            msg = "Invalid absolute ACLR measure"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, msg)

        return results

    def get_avg_max_power_verdict(self):
        """
        Retrieve maximum power verdict from the network simulator
        :rtype: object
        :return: the verdict:
            - Global.SUCCESS
            - Global.FAILURE
        """
        (err, verdicts, msg) = W.CalculatePowerMatching3G(
            self.get_root(),
            "MAX")
        self.__error_check(err, msg)
        if "RMS_POW_AVG" not in verdicts.keys():
            msg = "Verdict is missing"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)
        if verdicts["RMS_POW_AVG"] == "OK":
            return Global.SUCCESS
        else:
            return Global.FAILURE

    def get_avg_min_power_verdict(self):
        """
        Retrieve minimum power verdict from the network simulator
        :rtype: object
        :return: the verdict:
            - Global.SUCCESS
            - Global.FAILURE
        """
        (err, verdicts, msg) = W.CalculatePowerMatching3G(
            self.get_root(),
            "MIN")
        self.__error_check(err, msg)
        if "RMS_POW_AVG" not in verdicts.keys():
            msg = "Verdict is missing"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)
        if verdicts["RMS_POW_AVG"] == "OK":
            return Global.SUCCESS
        else:
            return Global.FAILURE

    def get_modulation_verdicts(self):
        """
        Retrieve RMS EVM and frequency error verdicts
        :rtype: dict
        :return: a dictionary containing the following verdicts
            - key: "PEAK_EVM": average peak error vector magnitude verdict
            - key: "RMS_EVM": average RMS error vector magnitude verdict
            - key: "OFFEST": average I/Q origin offset verdict
            - key: "FREQUENCY_ERROR": min/max frequency error verdict
            - key: "DOM_ERROR": average peak code domain error verdict
        """
        # Calculate verdicts for average values
        (err, avg_verdicts, msg) = W.CalculateEvmWcdmaMatching3G(
            self.get_root(),
            0,
            "AVERAGE")
        self.__error_check(err, msg)
        # Calculate verdicts for max/min values
        (err, minmax_verdicts, msg) = W.CalculateEvmWcdmaMatching3G(
            self.get_root(),
            0,
            "MAXMIN")
        self.__error_check(err, msg)

        # Check and get needed verdicts for average values
        avg_verdicts_keys = ["PEAK_EVM", "RMS_EVM", "OFFSET", "DOM_ERROR"]
        for key in avg_verdicts_keys:
            if key not in avg_verdicts.keys():
                msg = "Missing verdict"
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)

        # Check and get needed verdicts for min/max values
        if "FREQUENCY_ERROR" not in minmax_verdicts.keys():
            msg = "Missing verdict"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, msg)

        avg_verdicts["FREQUENCY_ERROR"] = minmax_verdicts["FREQUENCY_ERROR"]

        for key in avg_verdicts.keys():
            if str(avg_verdicts[key]).upper() == "OK":
                avg_verdicts[key] = Global.SUCCESS
            else:
                avg_verdicts[key] = Global.FAILURE

        return avg_verdicts

    def get_sem_avg_verdict(self):
        """
        Retrieve SEM verdicts from the network simulator
        :rtype: object
        :return: the verdict:
            - Global.SUCCESS
            - Global.FAILURE
        """
        (err, verdicts, msg) = W.CalculateSpectrumEmaskMatching3G(
            self.get_root(),
            "AVERAGE")
        self.__error_check(err, msg)
        verdict_failure = False
        for key in verdicts.keys():
            self.get_logger().info("\t%s: %s", key, verdicts[key])
            if verdicts[key] != "OK":
                verdict_failure = True

        if verdict_failure:
            msg = "At least one SEM section verdict returned FAIL"
            self.get_logger().info(msg)
            return Global.FAILURE

        self.get_logger().info("All SEM section verdict returned SUCCESS")
        return Global.SUCCESS

    def get_aclr_avg_verdicts(self):
        """
        Retrieve ACLR and OBW verdicts from the network simulator
        :rtype: dict
        :return: a dictionary containing the following verdicts:
            .. note:: each verdict value can be:
                - Global.FAILURE
                - Global.SUCCCESS
            - key: "ACLR-10MHZ": average ACLR -10MHz verdict
            - key: "ACLR-5MHZ": average ACLR -5MHz verdict
            - key: "ACLR+5MHZ": average ACLR +5MHz verdict
            - key: "ACLR+10MHZ": average ACLR +10MHz verdict
            - key: "OBW": average OBW verdict
        """
        aclr_verdicts = {}
        miss_verdict = "Missing verdict"

        # Get ACLR -10MHz verdicts
        (err, verdicts, msg) = W.CalculateSpectrumMfftMatching3G(
            self.get_root(),
            - 2)
        self.__error_check(err, msg)
        if "ACLR_RMS_AVG" not in verdicts.keys():
            self.get_logger().error(miss_verdict)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, miss_verdict)
        if "OBW_AVG" not in verdicts.keys():
            self.get_logger().error(miss_verdict)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, miss_verdict)

        aclr_verdicts["ACLR-10MHZ"] = verdicts["ACLR_RMS_AVG"]
        aclr_verdicts["OBW"] = verdicts["OBW_AVG"]

        # Get ACLR -5MHz verdicts
        (err, verdicts, msg) = W.CalculateSpectrumMfftMatching3G(
            self.get_root(),
            - 1)
        self.__error_check(err, msg)
        if "ACLR_RMS_AVG" not in verdicts.keys():
            self.get_logger().error(miss_verdict)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, miss_verdict)

        aclr_verdicts["ACLR-5MHZ"] = verdicts["ACLR_RMS_AVG"]

        # Get ACLR +5MHz verdicts
        (err, verdicts, msg) = W.CalculateSpectrumMfftMatching3G(
            self.get_root(),
            1)
        self.__error_check(err, msg)
        if "ACLR_RMS_AVG" not in verdicts.keys():
            self.get_logger().error(miss_verdict)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, miss_verdict)

        aclr_verdicts["ACLR+5MHZ"] = verdicts["ACLR_RMS_AVG"]

        # Get ACLR +10MHz verdicts
        (err, verdicts, msg) = W.CalculateSpectrumMfftMatching3G(
            self.get_root(),
            2)
        self.__error_check(err, msg)
        if "ACLR_RMS_AVG" not in verdicts.keys():
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.VERDICT_ERROR, miss_verdict)

        aclr_verdicts["ACLR+10MHZ"] = verdicts["ACLR_RMS_AVG"]

        for key in aclr_verdicts.keys():
            if str(aclr_verdicts[key]).upper() == "OK":
                aclr_verdicts[key] = Global.SUCCESS
            else:
                aclr_verdicts[key] = Global.FAILURE

        return aclr_verdicts
