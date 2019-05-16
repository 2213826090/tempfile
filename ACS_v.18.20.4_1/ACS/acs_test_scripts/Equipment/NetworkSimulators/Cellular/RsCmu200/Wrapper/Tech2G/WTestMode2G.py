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

:summary: wrapper for RS CMU200 2G test mode functions

:organization: INTEL MCG PSI
:author: ymorel
:since: 05/04/2011
"""

import ctypes

# RS CMU 200 verdicts, the returned verdict (integer) is the index of
# the verdict in the VERDICTS list
RS_CMU200_VERDICTS = [
    "OK",
    "INVALID",
    "KO",
    "NTSC",
    "OUT",
    "NO_TRIGGER",
    "BNF",
    "NOT_RAMPING",
    "OFF",
    "OVERFLOW",
    "UNDERFLOW"]


def ConfigureBERMeasurement(eqt, nb_frames, ref_level):
    """
    Wraps to ConfigureBERMeasurement2G driver function
    :raise TestEquipmentException: failed to call ConfigureBERMeasurement driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type nb_frames: integer
    :param nb_frames: number of frames to use for the BER measurement.
    :type ref_level: double
    :param ref_level: the cell power reference to use for the measurement
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Configure bit error rate measurement")
    eqt.get_logger().debug("\tNumber of frames: %.3f", nb_frames)
    eqt.get_logger().debug("\tTransmit power: %.3f", ref_level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureBERMeasurement2G(handle, ctypes.c_int(nb_frames),
                                        ctypes.c_double(ref_level), ctypes.byref(err_msg))
    return err, err_msg.value


def GetBER(eqt):
    """
    Wraps to GetBER2G driver function
    :raise TestEquipmentException: failed to call GetBER driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - double: the measured BER (Bit Error Rate)
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get bit error rate")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ber = ctypes.c_double()
    err = dll.GetBER2G(handle, ctypes.byref(ber), ctypes.byref(err_msg))
    return err, ber.value, err_msg.value


def GetRSSI(eqt):
    """
    Wraps to GetRSSI2G driver function
    :raise TestEquipmentException: failed to call GetRSSI driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - long: the measured RSSI (Received Signal Strength Indicator)
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get RX level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    rssi = ctypes.c_long()
    err = dll.GetRSSI2G(handle, ctypes.byref(rssi), ctypes.byref(err_msg))
    return err, rssi.value, err_msg.value


def SetSpectrumModulationLimitsLine(
    eqt,
    modulation,
    measure_point,
    enable,
    min_power,
    max_power,
        ref_power):
    """
    Wraps to SetSpectrumModulationLimitsLine2G driver function
    This function activates and defines limits line for the spectrum due to modulation measurement.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: The modulation type to use:
        - "GSM"
        - "EDGE"
    :type measure_point: long
    :param measure_point: Measurement point (frequency), an integer from 1 to 11.
    :type enable: str
    :param enable: Enable or disable limit check for frequency point:
        - "OFF"
        - "ON"
    :type min_power: double
    :param min_power: Limit for relative power below the interpolation range (-99.9 dB to +99.9dB)
    :type max_power: double
    :param max_power: Limit for relative power above the interpolation range (-99.9 dB to +99.9dB)
    :type ref_power: double
    :param ref_power: Alternative absolute power limit (-99.9 dBm to +99.9dBm)
    :rtype: tuple
    :return:
        - integer: the error code of the driver function
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call SetSpectrumModulationLimitsLine2G driver function
    """
    eqt.get_logger().info("Set %s spectrum modulation limits", modulation)
    eqt.get_logger().debug("\tMeasurement point: %d", measure_point)
    eqt.get_logger().debug("\tEnable: %s", enable)
    eqt.get_logger().debug("\tMinimum power: %f", min_power)
    eqt.get_logger().debug("\tMaximum power: %f", max_power)
    eqt.get_logger().debug("\tReference power: %f", ref_power)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSpectrumModulationLimitsLine2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.c_long(measure_point),
        ctypes.c_char_p(enable),
        ctypes.c_double(min_power),
        ctypes.c_double(max_power),
        ctypes.c_double(ref_power),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSpectrumSwitchingLimitsLine(
    eqt,
    modulation,
    pow_lvl_num,
    pow_lvl,
    enable,
    limit_040MHz,
    limit_060MHz,
    limit_120MHz,
        limit_180MHz):
    """
    Wraps to SetSpectrumSwitchingLimitsLine2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: The modulation type to use:
        - "GSM"
        - "EDGE"
    :type pow_lvl_num: long
    :param pow_lvl_num: Power level number, an integer from 1 to 10.
    :type pow_lvl: double
    :param pow_lvl: MS output power for power level number (-100dBm to 30dBm)
    :type enable: str
    :param enable: Enable or disable limit check for power level:
        - "OFF"
        - "ON"
    :type limit_040MHz: double
    :param limit_040MHz: Limit for the measurement point at 0.4 MHz (-100dBm to 30dBm)
    :type limit_060MHz: double
    :param limit_060MHz: Limit for the measurement point at 0.6 MHz (-100dBm to 30dBm)
    :type limit_120MHz: double
    :param limit_120MHz: Limit for the measurement point at 1.2 MHz (-100dBm to 30dBm)
    :type limit_180MHz: double
    :param limit_180MHz: Limit for the measurement point at 1.8 MHz (-100dBm to 30dBm)
    :rtype: tuple
    :return:
        - integer: the error code of the driver function
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call SetSpectrumSwitchingLimitsLine2G driver function
    """
    eqt.get_logger().info("Set %s spectrum switching limits", modulation)
    eqt.get_logger().debug("\tPower level number: %d", pow_lvl_num)
    eqt.get_logger().debug("\tPower level: %f", pow_lvl)
    eqt.get_logger().debug("\tEnable: %s", enable)
    eqt.get_logger().debug("\t0.40MHz limit: %f", limit_040MHz)
    eqt.get_logger().debug("\t0.60MHz limit: %f", limit_060MHz)
    eqt.get_logger().debug("\t1.20MHz limit: %f", limit_120MHz)
    eqt.get_logger().debug("\t1.80MHz limit: %f", limit_180MHz)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSpectrumSwitchingLimitsLine2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.c_long(pow_lvl_num),
        ctypes.c_double(pow_lvl),
        ctypes.c_char_p(enable),
        ctypes.c_double(limit_040MHz),
        ctypes.c_double(limit_060MHz),
        ctypes.c_double(limit_120MHz),
        ctypes.c_double(limit_180MHz),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ReadPower(eqt, modulation):
    """
    Wraps to ReadPower2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: The modulation type to use:
        - "GSM"
        - "EDGE"
    :rtype: tuple
    :return:
        - integer: the error code of the driver function
        - dict: a dictionary containing:
            - AVG_BURST_POWER: float
            - PEAK_BURST_POWER: float
            - PCL: integer
            - TIMING_ADV_ERROR: float
            - BURST_OUT_OF_TOLERANCE: float
            - BURST_MATCHING: str:
                - "OK": matching
                - "INVALID": invalid
                - "KO": not matching
                - "NTSC": no training sequence code
                - "OUT": out of range
                - "NO_TRIGGER": no trigger
                - "BNF": burst not found
                - "NOT_RAMPING": not ramping
                - "OFF": off
                - "OVERFLOW": overflow
                - "UNDERFLOW": underflow
            - AVG_BURST_POWER: float
        - str: the error message
    :raise TestEquipmentException: failed to call ReadPower2G driver function
    """
    eqt.get_logger().info("Read %s power", modulation)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    avg_burst_pow = ctypes.c_double()
    peak_burst_pow = ctypes.c_double()
    pow_cont_lvl = ctypes.c_long()
    timing_advance_error_bit = ctypes.c_double()
    burst_out_of_tol = ctypes.c_double()
    burst_template_matching = ctypes.c_long()
    avg_burst_avg_pow = ctypes.c_double()
    err = dll.ReadPower2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.byref(avg_burst_pow),
        ctypes.byref(peak_burst_pow),
        ctypes.byref(pow_cont_lvl),
        ctypes.byref(timing_advance_error_bit),
        ctypes.byref(burst_out_of_tol),
        ctypes.byref(burst_template_matching),
        ctypes.byref(avg_burst_avg_pow),
        ctypes.byref(err_msg))
    powers = {}
    powers["AVG_BURST_POWER"] = avg_burst_pow.value
    powers["PEAK_BURST_POWER"] = peak_burst_pow.value
    powers["PCL"] = pow_cont_lvl.value
    powers["TIMING_ADV_ERROR"] = timing_advance_error_bit.value
    powers["BURST_OUT_OF_TOLERANCE"] = burst_out_of_tol.value
    powers["BURST_MATCHING"] = RS_CMU200_VERDICTS[burst_template_matching.value]
    powers["AVG_BURST_POWER"] = avg_burst_avg_pow.value
    return err, powers, err_msg.value


def ReadModXper(eqt, stats_mode):
    """
    Wraps to ReadModXper2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type stats_mode: str
    :param stats_mode: Defines how the measurement is calculated if the measurement
    extends over several bursts. Possible values:
        - "CURRENT": current value of the burst measured
        - "AVERAGE": average value of all bursts measured
        - "MAXMIN" : extreme value of all bursts measured
    :rtype: tuple
    :return:
        - integer: the error code of the driver function
        - dict: a dictionary containing:
            - "PHASE_ERR_PEAK": integer
            - "PHASE_ERR_RMS": integer
            - "OFFSET": float
            - "IQ_IMBALANCE": float
            - "FREQUENCY_ERR": float
            - "AVG_BURST_POWER": float
            - "BURST_OUT_OF_TOL": float
        - str: the error message
    :raise TestEquipmentException: failed to call ReadModXper2G driver function
    """
    eqt.get_logger().info("Read %s phase error", stats_mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    per_peak = ctypes.c_double()
    per_rms = ctypes.c_double()
    offset = ctypes.c_double()
    iq_imbalance = ctypes.c_double()
    freq_err = ctypes.c_double()
    avg_burst_pow_curr = ctypes.c_double()
    burst_out_of_tol = ctypes.c_double()
    err = dll.ReadModXper2G(
        handle,
        ctypes.c_char_p(stats_mode),
        ctypes.byref(per_peak),
        ctypes.byref(per_rms),
        ctypes.byref(offset),
        ctypes.byref(iq_imbalance),
        ctypes.byref(freq_err),
        ctypes.byref(avg_burst_pow_curr),
        ctypes.byref(burst_out_of_tol),
        ctypes.byref(err_msg))
    modulations = {}
    modulations["PHASE_ERR_PEAK"] = per_peak.value
    modulations["PHASE_ERR_RMS"] = per_rms.value
    modulations["OFFSET"] = offset.value
    modulations["IQ_IMBALANCE"] = iq_imbalance.value
    modulations["FREQUENCY_ERR"] = freq_err.value
    modulations["AVG_BURST_POWER"] = avg_burst_pow_curr.value
    modulations["BURST_OUT_OF_TOL"] = burst_out_of_tol.value
    return err, modulations, err_msg.value


def FetchSpectrumModulationVerdict(eqt, modulation):
    """
    Wraps to FetchSpectrumModulationVerdict2G 2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: The modulation type to use:
        - "GSM"
        - "EDGE"
    :rtype: tuple
    :return:
        - integer: the error code of the driver function
        - integer: the absolute carrier power measured
        - str: matching state. Possible values:
            - "OK": matching
            - "INVALID": invalid
            - "KO": not matching
            - "NTSC": no training sequence code
            - "OUT": out of range
            - "NO_TRIGGER": no trigger
            - "BNF": burst not found
            - "NOT_RAMPING": not ramping
            - "OFF": off
            - "OVERFLOW": overflow
            - "UNDERFLOW": underflow
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call FetchSpectrumModulationVerdict2G driver function
    """
    eqt.get_logger().info("Fetch %s spectrum modulation verdict", modulation)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ref_power = ctypes.c_double()
    match = ctypes.c_long()
    err = dll.FetchSpectrumModulationVerdict2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.byref(ref_power),
        ctypes.byref(match),
        ctypes.byref(err_msg))
    return err, ref_power.value, RS_CMU200_VERDICTS[match.value], err_msg.value


def ReadSpectrumModulation(eqt, modulation):
    """
    Wraps to ReadSpectrumModulation2G 2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: the modulation type to use:
        - "GSM"
        - "EDGE"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - dict: a dictionary containing the power level as float values for each
                following key:
            - "0MHZ"     | "-0,10MHZ" | "-0,20MHZ" | "-0,25MHZ" | "-0,40MHZ" |
              "-0,60MHZ" | "-0,80MHZ" | "-1,00MHZ" | "-1,20MHZ" | "-1,40MHZ" |
              "-1,60MHZ" | "-1,80MHZ" | "0,10MHZ"  | "0,20MHZ"  | "0,25MHZ"  |
              "0,40MHZ"  | "0,60MHZ"  | "0,80MHZ"  | "1,00MHZ"  | "1,20MHZ"  |
              "1,40MHZ"  | "1,60MHZ"  | "1,80MHZ"
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call ReadSpectrumModulation2G driver function
    """
    eqt.get_logger().info("Read %s spectrum modulation", modulation)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    size = 32768
    err_msg = ctypes.c_char_p('\x00' * 1024)
    powers = ctypes.c_char_p('\x00' * size)
    err = dll.ReadSpectrumModulation2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.c_int(size),
        powers,
        ctypes.byref(err_msg))
    # Split resulting powers str to identify each power
    splitted_powers = powers.value.split(',')  # pylint: disable=E1101
    # List of power keys in the order they appear in the resulting str
    power_keys = ["-1,80MHZ", "-1,60MHZ", "-1,40MHZ", "-1,20MHZ", "-1,00MHZ",
                  "-0,80MHZ", "-0,60MHZ", "-0,40MHZ", "-0,25MHZ", "-0,20MHZ",
                  "-0,10MHZ", "0MHZ", "0,10MHZ", "0,20MHZ", "0,25MHZ",
                  "0,40MHZ", "0,60MHZ", "0,80MHZ", "1,00MHZ", "1,20MHZ",
                  "1,40MHZ", "1,60MHZ", "1,80MHZ"]
    # Fill the resulting dictionary
    powers_dict = {}
    power_idx = 0
    for power_key in power_keys:
        powers_dict[power_key] = float(splitted_powers[power_idx])
        power_idx += 1

    return err, powers_dict, err_msg.value


def FetchSpectrumSwitchingVerdict(eqt, modulation):
    """
    Wraps to FetchSpectrumSwitchingVerdict2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: the modulation type to use:
        - "GSM"
        - "EDGE"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - double: reference power
        - str: matching state
            - "OK": matching
            - "INVALID": invalid
            - "KO": not matching
            - "NTSC": no training sequence code
            - "OUT": out of range
            - "NO_TRIGGER": no trigger
            - "BNF": burst not found
            - "NOT_RAMPING": not ramping
            - "OFF": off
            - "OVERFLOW": overflow
            - "UNDERFLOW": underflow
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call FetchSpectrumSwitchingVerdict2G driver function
    """
    eqt.get_logger().info("Fetch %s spectrum switching verdict", modulation)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ref_power = ctypes.c_double()
    match = ctypes.c_long()
    err = dll.FetchSpectrumSwitchingVerdict2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.byref(ref_power),
        ctypes.byref(match),
        ctypes.byref(err_msg))

    return err, ref_power.value, RS_CMU200_VERDICTS[match.value], err_msg.value


def ReadSpectrumSwitching(eqt, modulation):
    """
    Wraps to ReadSpectrumSwitching2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type modulation: str
    :param modulation: the modulation type to use:
        - "GSM"
        - "EDGE"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - dict: a dictionary containing the power as float values for each following key:
            - "-1800KHZ", "-1200KHZ", "-600KHZ", "-400KHZ", "0KHZ", "400KHZ",
              "600KHZ", "1200KHZ", "1800KHZ"
            - "NAN" is returned at the disabled points
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call ReadSpectrumSwitching2G driver function
    """
    eqt.get_logger().info("Read %s spectrum switching", modulation)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    powers = ctypes.c_char_p('\x00' * 32768)
    err = dll.ReadSpectrumSwitching2G(
        handle,
        ctypes.c_char_p(modulation),
        ctypes.c_int(32768),
        powers,
        ctypes.byref(err_msg))
    # Split resulting powers str to identify each power
    splitted_powers = powers.value.split(',')  # pylint: disable=E1101
    # List of power keys in the order they appear in the resulting str
    power_keys = ["-1800KHZ", "-1200KHZ", "-600KHZ", "-400KHZ", "0KHZ", "400KHZ",
                  "600KHZ", "1200KHZ", "1800KHZ"]
    # Fill the resulting dictionary
    powers_dict = {}
    power_idx = 0
    for power_key in power_keys:
        powers_dict[power_key] = float(splitted_powers[power_idx])
        power_idx += 1

    return err, powers_dict, err_msg.value


def ReadEdgeModulationEvm(eqt, stats_mode):
    """
    Wraps to ReadEdgeModulationEvm2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type stats_mode: str
    :param stats_mode: defines how the measurement is calculated if the measurement
        is extends over several bursts. Possible values:
            - "CURRENT": current value of the burst measured
            - "AVERAGE": average value of all bursts measured
            - "MAXMIN" : extreme value of all bursts measured
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - dict: a dictionary containing the following keys:
            - key: "95EVM"            => double: error vector magnitude
            - key: "EVM_PEAK"         => double: error vector magnitude peak
            - key: "EVM_RMS"          => double: error vector magnitude RMS
            - key: "OFFSET"           => double: offset
            - key: "IQ_IMBALANCE"     => double: IQ imbalance
            - key: "FREQUENCY_ERR"    => double: frequency error
            - key: "AVG_BURST_POWER"  => double: average burst power current
            - key: "BURST_OUT_OF_TOL" => double: burst out of tolerance
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call ReadEdgeModulationEvm2G driver function
    """
    eqt.get_logger().info("Read EDGE %s modulation error vector magnitude", stats_mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    evm = ctypes.c_double()
    evm_peak = ctypes.c_double()
    evm_rms = ctypes.c_double()
    offset = ctypes.c_double()
    iq_imbalance = ctypes.c_double()
    freq_err = ctypes.c_double()
    avg_burst_pow_curr = ctypes.c_double()
    burst_out_of_tol = ctypes.c_double()
    err = dll.ReadEdgeModulationEvm2G(
        handle,
        ctypes.c_char_p(stats_mode),
        ctypes.byref(evm),
        ctypes.byref(evm_peak),
        ctypes.byref(evm_rms),
        ctypes.byref(offset),
        ctypes.byref(iq_imbalance),
        ctypes.byref(freq_err),
        ctypes.byref(avg_burst_pow_curr),
        ctypes.byref(burst_out_of_tol),
        ctypes.byref(err_msg))
    results = {}
    results["95EVM"] = evm.value
    results["EVM_PEAK"] = evm_peak.value
    results["EVM_RMS"] = evm_rms.value
    results["OFFSET"] = offset.value
    results["IQ_IMBALANCE"] = iq_imbalance.value
    results["FREQUENCY_ERR"] = freq_err.value
    results["AVG_BURST_POWER"] = avg_burst_pow_curr.value
    results["BURST_OUT_OF_TOL"] = burst_out_of_tol.value
    return err, results, err_msg.value


def ConfigurePdatBerTchSlotPowerLevel(eqt, test_setup, meas_mode, slot, level):
    """
    Wraps to ConfigurePdatBerTchSlotPowerLevel2G driver function.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type test_setup: str
    :param test_setup: the test set up to use for measurement ("TS1" .. "TS10").
    :type meas_mode: str
    :parame meas_mode: the measurement mode. Possible values:
            - "SINGLE_SHOT"
            - "CONTINUOUS"
    :type slot: integer
    :param slot: the slot number (range 0 .. 7)
    :type level: float
    :param level: the power level to set for the slot. Range -137dBm to 13dBm
    :raise TestEquipmentException: failed to call ConfigurePdatBerTchSlotPowerLevel2G driver function
    """
    eqt.get_logger().info("Configure bit error rate data multislot configuration")
    eqt.get_logger().debug("\tTest setup: %s", str(test_setup))
    eqt.get_logger().debug("\tMeasurement mode: %s", str(meas_mode))
    eqt.get_logger().debug("\tSlot: %s", str(slot))
    eqt.get_logger().debug("\tLevel: %s", str(level))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigurePdatBerTchSlotPowerLevel2G(
        handle,
        ctypes.c_char_p(test_setup),
        ctypes.c_char_p(meas_mode),
        ctypes.c_long(slot),
        ctypes.c_double(level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetBerTchReferenceLevel(eqt):
    """
    Wraps to GetBerTchReferenceLevel2G driver function.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: the reference power level
    :raise TestEquipmentException: failed to call GetBerTchReferenceLevel2G driver function
    """
    eqt.get_logger().info("Get data bit error rate reference level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    level = ctypes.c_double()
    err = dll.GetBerTchReferenceLevel2G(
        handle,
        ctypes.byref(level),
        ctypes.byref(err_msg))
    return err, level.value, err_msg.value
