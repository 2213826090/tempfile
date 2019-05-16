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

:summary: wrapper for RS CMU200 3G test mode functions

:organization: INTEL MCG PSI
:author: ymorel
:since: 05/04/2011
"""

import ctypes


def SetBERBlockNumber(eqt, block_number):
    """
    Wraps to SetBERBlockNumber 3G driver function
    :raise TestEquipmentException: failed to call SetBERBlockNumber
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type block_number: integer
    :param block_number: the number of blocks to set: an integer from
        1 to 50000
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set BER block number to %d", block_number)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBERBlockNumber3G(
        handle,
        ctypes.c_int(block_number),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetBER(eqt):
    """
    Wraps to GetBER 3G driver function
    :raise TestEquipmentException: failed to call GetBER driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: the measured BER (Bit Error Rate)
    """
    eqt.get_logger().info("Get bit error rate")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ber = ctypes.c_double()
    err = dll.GetBER3G(handle, ctypes.byref(ber), ctypes.byref(err_msg))
    return err, ber.value, err_msg.value


def ConfigureRSSIMeasurement(eqt):
    """
    Wraps to ConfigureRSSIMeasurement3G 3G driver function
    :raise TestEquipmentException: failed to call ConfigureRSSIMeasurement3G driver
    function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Configure equipment for RSSI measurement")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureRSSIMeasurement3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def GetRSSI(eqt):
    """
    Wraps to GetRSSI 3G driver function
    :raise TestEquipmentException: failed to call GetRSSI driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: the measured RSSI (Received Signal Strength Indicator)
    """
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    rssi = ctypes.c_double()
    err = dll.GetRSSI3G(handle, ctypes.byref(rssi), ctypes.byref(err_msg))
    return err, rssi.value, err_msg.value


def SetDefaultPowerMaximumLimits3G(eqt, mode):
    """
    Enables or disables the default maximum power limits
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the mode to set:
        - "ON" : enables default limits
        - "OFF" : disables default limits
    """
    eqt.get_logger().info("Set default maximum power limits %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDefaultPowerMaximumLimits3G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigurePowerMaximumLimitsRated3G(eqt, display_mode, peak, rms):
    """
    Defines the rated values for the peak and RMS averaged UE output power
    calculated for the current slot and the maximum and minimum of several
    consecutive slots, or the average of them.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated curves
        are displayed if the measurement extends over several slots/evaluation
        periods. In general, curves are evaluated at a set of fixed, equidistant
        test points (samples). After n slots evaluation periods, n measurement
        results per test point have been taken. Possible values:
            - "CURRENT": the current slot, i.e. the last result for all test
            and points, is displayed
            - "AVERAGE": at each test point, a suitably defined average over
            all slots evaluation periods measured is displayed.
    :type peak: double
    :param peak: defines the rated value for UE Power (Peak).
        Value range: -60.0 dBm to 53.0 dBm
    :type rms: double
    :param rms: defines the rated value for UE Power (RMS).
        Value range: -60.0 dBm to 53.0 dBm
    """
    eqt.get_logger().info("Set maximum power rated limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigurePowerMaximumLimitsRated3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(peak),
        ctypes.c_double(rms),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigurePowerMaximumLimitsUpper3G(eqt, display_mode, peak, rms):
    """
    Defines the upper limits for the peak and RMS averaged UE output power calculated
    for the current slot and the maximum and minimum of several consecutive slots,
    or the average of them.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: The display mode defines which of the measured and calculated
        curves are displayed if the measurement extends over several slots/evaluation
        periods. In general, curves are evaluated at a set of fixed, equidistant test
        points (samples). After n slots evaluation periods, n measurement results per
        test point have been taken. Possible values:
            - "CURRENT": the current slot, i.e. the last result for all test
                and points, is displayed
            - "AVERAGE": at each test point, a suitably defined average over
                all slots evaluation periods measured is displayed.
    :type peak: double
    :param peak: defines the upper limit for UE Power (Peak). Value range: 0.0 dB to 5.0 dB
    :type rms: double
    :param rms: defines the upper limit for UE Power (RMS). Value range: 0.0 dB to 5.0 dB
    """
    eqt.get_logger().info("Set maximum power upper limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigurePowerMaximumLimitsUpper3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(peak),
        ctypes.c_double(rms),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigurePowerMaximumLimitsLower3G(eqt, display_mode, peak, rms):
    """
    Defines the lower limits for the peak and RMS averaged UE output power calculated
    for the current slot and the maximum and minimum of several consecutive slots,
    or the average of them
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: The display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "CURRENT": the current slot, i.e. the last result for all test
        and points, is displayed
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type peak: double
    :param peak: defines the lower limit for UE Power (Peak). Value range: 0.0 dB to 5.0 dB
    :type rms: double
    :param rms: defines the lower limit for UE Power (RMS). Value range: 0.0 dB to 5.0 dB
    """
    eqt.get_logger().info("Set maximum power lower limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigurePowerMaximumLimitsLower3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(peak),
        ctypes.c_double(rms),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDefaultPowerMinimumLimits3G(eqt, mode):
    """
    Enables or disables the default minimum power limits.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the desired mode, possible values:
        - "ON": enables default limits
        - "OFF": disables default limits
    """
    eqt.get_logger().info("Set default minimum power limits to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDefaultPowerMinimumLimits3G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigurePowerMinimumLimitsUpper3G(eqt, display_mode, peak, rms):
    """
    Defines the upper limits for the peak and RMS averaged UE output power calculated
    for the current slot and the maximum and minimum of several consecutive slots,
    or the average of them.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "CURRENT": the current slot, i.e. the last result for all test
        and points, is displayed
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type peak: double
    :param peak: defines the upper limit for UE Power (Peak). Value range: -60.0 dB to 53.0 dB
    :type rms: double
    :param rms: defines the upper limit for UE Power (RMS). Value range: -60.0 dB to 53.0 dB
    """
    eqt.get_logger().info("Set minimum power lower limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigurePowerMinimumLimitsUpper3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(peak),
        ctypes.c_double(rms),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDefaultEvmWcdmaLimits3G(eqt, mode):
    """
    Enables or disables the default EVM limits.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the desired mode, possible values:
        - "ON": enables default limits
        - "OFF": disables default limits
    """
    eqt.get_logger().info("Set default EVM WCDMA limits to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDefaultEvmWcdmaLimits3G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigureEvmWcdmaLimitsUpper3G(
    eqt,
    display_mode,
    evm_peak,
    evm_rms,
    magn_err_peak,
    magn_err_rms,
    phase_err_peak,
    phase_err_rms,
    iq_origin_offset,
    iq_imbalance,
    carrier_freq_err,
    waveform_quality,
    peak_code_domain_err,
        transmit_time_error):
    """
    Defines upper limits for the current and max/min, or average traces evaluated
    over the whole evaluation period and for the scalar modulation parameters derived
    from them.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "CURRENT": the current slot, i.e. the last result for all test
        and points, is displayed
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type evm_peak: double
    :param evm_peak: sets the error vector magnitude peak. Value range: 0.0 % to 20.0 %
    :type evm_rms: double
    :param evm_rms: Sets the error vector magnitude RMS. Value range: 0.0 % to 20.0 %
    :type magn_err_peak: double
    :param magn_err_peak: sets the magnitude error peak. The values have symetric ranges
        of limits, so the entry of positive or negative values is equivalent.
        Value range: -20.0 % to 20.0 %
    :type magn_err_rms: double
    :param magn_err_rms: sets the magnitude error RMS. Value range: 0.0 % to 20.0 %
    :type phase_err_peak: sets the phase error peak. The values have symetric ranges
        of limits, so the entry of positive or negative values is equivalent.
        Value range: -20.0 deg to 20.0 deg
    :type phase_err_rms: double
    :param phase_err_rms: sets the phase error RMS. Value range: 0.0 deg to 20.0 deg
    :type iq_origin_offset: double
    :param iq_origin_offset: sets the I/Q origin offset. Value range: -80.0 dB to -20.0 dB
    :type iq_imbalance: double
    :param iq_imbalance: sets the I/Q imbalance. Value range: -99.00 dB to 0.00 dB
    :type carrier_freq_err: integer
    :param carrier_freq_err: sets the carrier frequency error. The values have symetric
        ranges of limits, so the entry of positive or negative values is equivalent.
        Value range: -4000 Hz to 4000 Hz
    :type waveform_quality: double
    :param waveform_quality: sets the waveform quality. Value range: 0.9000 to 0.9999
    :type pcd_err: double
    :param pcd_err: sets the peak code domain error. Value range: -40.0 dB to 0.0 dB
    :type tt_error: double
    :param tt_error: sets the transmit time error. The values have symetric ranges of limits,
        so the entry of positive or negative values is equivalent.
        Note: this value IS NOT AVAILABLE IN FW V3.00.
        Value range: -25.00 us to 25.00 us
    """
    eqt.get_logger().info("Set EVM WCDMA upper limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureEvmWcdmaLimitsUpper3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(evm_peak),
        ctypes.c_double(evm_rms),
        ctypes.c_double(magn_err_peak),
        ctypes.c_double(magn_err_rms),
        ctypes.c_double(phase_err_peak),
        ctypes.c_double(phase_err_rms),
        ctypes.c_double(iq_origin_offset),
        ctypes.c_double(iq_imbalance),
        ctypes.c_int(carrier_freq_err),
        ctypes.c_double(waveform_quality),
        ctypes.c_double(peak_code_domain_err),
        ctypes.c_double(transmit_time_error),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDefaultSpectrumMfftLimits3G(eqt, mode):
    """
    Enables or disables the default settings.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the desired mode, possible values:
        - "ON": enables default limits
        - "OFF": disables default limits
    """
    eqt.get_logger().info("Set default spectrum MFFT limits to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDefaultSpectrumMfftLimits3G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigureSpectrumMfftLimitsRelative3G(
    eqt,
    display_mode,
    channel_number,
    upper_limit,
        enable):
    """
    Sets the upper limit for the ACLR determined via FFT and switches the relative
    limit check in the channel on or off.
    Note : switching off the relative limit check also disables the absolute limit check.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
        curves are displayed if the measurement extends over several slots/evaluation periods.
        In general, curves are evaluated at a set of fixed, equidistant test points (samples).
        After n slots evaluation periods, n measurement results per test point have been taken.
        Possible values:
            - "MAX": at each test point, the extreme value of all slots evaluation periods
            measured is displayed, i.e. the maximum or minimum, whichever has a larger
            absolute value.
            - "AVERAGE": at each test point, a suitably defined average over all slots
            evaluation periods measured is displayed.
    :type channel_number: integer
    :param channel_number: sets the channel number. The channel frequency is the channel
    number times 5 MHz from the carrier frequency. Available values: -2, -1, 1 or 2
    :type upper_limit: double
    :param upper_limit: sets the upper limit for the ACLR determined via FFT.
        Value range: -80.0 dBc to 0.0 dBc upper limit for the ACLR.
    :type enable: str
    :param enable: switches the limit check in a particular channel on or off.
        Possible values:
            - "ON"
            - "OFF"
    """
    eqt.get_logger().info("Set spectrum MFFT relative limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureSpectrumMfftLimitsRelative3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_int(channel_number),
        ctypes.c_double(upper_limit),
        ctypes.c_char_p(enable),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigureSpectrumMfftLimitsAbsolute3G(eqt, display_mode, abs_limit, enable):
    """
    Defines the absolute limits for the ACLR. If the ACLR in all channels is below
    the absolute limits, the ACLR measurement will pass the limit check, irrespective of the
    relative limits. This function also switches the absolute limit check in all channels
    on or off.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "MAX": at each test point, the extreme value of all slots
        evaluation periods measured is displayed, i.e. the maximum or
        minimum, whichever has a larger absolute value.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type abs_limit: double
    :param abs_limit: sets the absolute limit for the ACLR. Value range: -80.0 dBm to 33.0 dBm.
    :type enable: str
    :param enable: switches the limit check on or off. Possible values:
        - "ON"
        - "OFF"
    """
    eqt.get_logger().info("Set spectrum MFFT absolute limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureSpectrumMfftLimitsAbsolute3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(abs_limit),
        ctypes.c_char_p(enable),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigureSpectrumMfftLimitsObw3G(eqt, display_mode, upper_limit, enable):
    """
    Sets the upper limits for the OBW determined via FFT and switches the limit
    check on or off.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "CURRENT": the current slot, i.e. the last result for all test
        and points, is displayed.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type upper_limit: double
    :param upper_limit: sets the upper limit for the OBW. Value range: -80.0 dBm to 33.0 dBm.
    :type enable: str
    :param enable: switches the limit check on or off. Possible values:
        - "ON"
        - "OFF"
    """
    eqt.get_logger().info("Set spectrum MFFT occupied bandwidth limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureSpectrumMfftLimitsObw3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(upper_limit),
        ctypes.c_char_p(enable),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigureSpectrumEmaskLimitsRelative3G(
    eqt,
    display_mode,
    limit_a,
    limit_b,
    limit_c,
    limit_d,
    limit_e,
        limit_f):
    """
    Defines the relative limit lines.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "MAX": at each test point, the extreme value of all slots
        evaluation periods measured is displayed, i.e. the maximum or
        minimum, whichever has a larger absolute value.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type limit_a: double
    :param limit_a: sets the relative limit at point A. Value range: -90.0 dB to 0.0 dB.
    :type limit_b: double
    :param limit_b: sets the relative limit at point B. Value range: -90.0 dB to 0.0 dB.
    :type limit_c: double
    :param limit_c: sets the relative limit at point C. Value range: -90.0 dB to 0.0 dB.
    :type limit_d: sets the relative limit at point D. Value range: -90.0 dB to 0.0 dB.
    :type limit_e: double
    :param limit_e: sets the relative limit at point E. Value range: -90.0 dB to 0.0 dB.
    :type limit_f: double
    :param limit_f: sets the relative limit at point F. Value range: -90.0 dB to 0.0 dB.
    """
    eqt.get_logger().info("Set spectrum MFFT absolute limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigureSpectrumEmaskLimitsRelative3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_double(limit_a),
        ctypes.c_double(limit_b),
        ctypes.c_double(limit_c),
        ctypes.c_double(limit_d),
        ctypes.c_double(limit_e),
        ctypes.c_double(limit_f),
        ctypes.byref(err_msg))
    return err, err_msg.value


def EnableSpectrumEmaskLimitsRelative3G(
    eqt,
    display_mode,
    enable_liad,
        enable_lief):
    """
    Defines the relative limit lines enable.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: str
    :param display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "MAX": at each test point, the extreme value of all slots
        evaluation periods measured is displayed, i.e. the maximum or
        minimum, whichever has a larger absolute value.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type enable_liad: str
    :param enable_liad: sets the relative limit enable at point A-D. Possible values:
        - "OFF"
        - "ON"
    :type enable_lief: str
    :param enable_lief: sets the relative limit enable at point E-F. Possible values:
        - "ON"
        - "OFF"
    """
    eqt.get_logger().info("Set spectrum emission mask relative limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.EnableSpectrumEmaskLimitsRelative3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_char_p(enable_liad),
        ctypes.c_char_p(enable_lief),
        ctypes.byref(err_msg))
    return err, err_msg.value


def EnableSpectrumEmaskLimitsAsolute3G(eqt, display_mode, enable_abs_lim):
    """
    Defines the absolute limit lines G enable.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type display_mode: the display mode defines which of the measured and calculated
    curves are displayed if the measurement extends over several slots/evaluation periods.
    In general, curves are evaluated at a set of fixed, equidistant test points (samples).
    After n slots evaluation periods, n measurement results per test point have been taken.
    Possible values:
        - "MAX": at each test point, the extreme value of all slots
        evaluation periods measured is displayed, i.e. the maximum or
        minimum, whichever has a larger absolute value.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots evaluation periods measured is displayed.
    :type enable_abs_lim: str
    :param enable_abs_lim: sets the absolute limit G enable referenced to a 3.84 MHz filter.
    Available Values:
        - "ON": limit point E-F enable
        - "OFF": limit point E-F disable
    """
    eqt.get_logger().info("Set spectrum emission mask relative limits")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.EnableSpectrumEmaskLimitsAsolute3G(
        handle,
        ctypes.c_char_p(display_mode),
        ctypes.c_char_p(enable_abs_lim),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ReadPower3G(eqt, application):
    """
    Starts a single shot measurement and returns results.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type application: str
    :param application: defines the application of the power measurement.
    Available values:
        - "MAX" : maximum power application.
        - "MIN" : minimum power application.
        - "OFF" : off power application
    :rtype: tuple
        - integer: the error code of the driver function
        - dict:
            - key: "PEAK_POW_CURR" the peak power current.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "PEAK_POW_AVG" the peak power average.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "PEAK_POW_MAX" the peak power maximum.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "RMS_POW_CURR" the RMS power current.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "RMS_POW_AVG" the RMS power average.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "RMS_POW_MAX" the RMS power maximum.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "RMS_POW_MIN" the RMS power minimum.
            It is returned in the maximum application only.
            It is omitted for the minimum and off applications.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key: "OUT_OF_TOL" the out of tolerance measurement.
            Returned value range: 0.0 % to 100.0 %
        - str : error message of driver function
    """
    eqt.get_logger().info("Read %s powers", application)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    peak_pow_curr = ctypes.c_double()
    peak_pow_avg = ctypes.c_double()
    peak_pow_max = ctypes.c_double()
    rms_pow_curr = ctypes.c_double()
    rms_pow_avg = ctypes.c_double()
    rms_pow_max = ctypes.c_double()
    rms_pow_min = ctypes.c_double()
    out_of_tol = ctypes.c_double()
    err = dll.ReadPower3G(
        handle,
        ctypes.c_char_p(application),
        ctypes.byref(peak_pow_curr),
        ctypes.byref(peak_pow_avg),
        ctypes.byref(peak_pow_max),
        ctypes.byref(rms_pow_curr),
        ctypes.byref(rms_pow_avg),
        ctypes.byref(rms_pow_max),
        ctypes.byref(rms_pow_min),
        ctypes.byref(out_of_tol),
        ctypes.byref(err_msg))
    powers = {}
    powers["PEAK_POW_CURR"] = peak_pow_curr.value
    powers["PEAK_POW_AVG"] = peak_pow_avg.value
    powers["PEAK_POW_MAX"] = peak_pow_max.value
    powers["RMS_POW_CURR"] = rms_pow_curr.value
    powers["RMS_POW_AVG"] = rms_pow_avg.value
    powers["RMS_POW_MAX"] = rms_pow_max.value
    powers["RMS_POW_MIN"] = rms_pow_min.value
    powers["OUT_OF_TOL"] = out_of_tol.value
    return err, powers, err_msg.value


def ReadSpectrumEmask3G(eqt):
    """
    Starts a single shot measurement and returns results.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
        - integer: error code of the driver function
        - dict:
            - key "REF_POW_CURR": the reference power current.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key "REF_POW_AVG": the reference power average.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key "REF_POW_MAX": the reference power maximum.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key UE_POW_CURR": the UE power current.
            Returned value range: -100.0 dBm to +60.0 dBm.
            - key "OUT_OF_TOL": the out of tolerance measurement.
            Returned value range: 0.0 % to 100.0 %
        - str: error message of the driver function
    """
    eqt.get_logger().info("Read spectrum emission mask powers")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ref_pow_curr = ctypes.c_double()
    ref_pow_avg = ctypes.c_double()
    ref_pow_max = ctypes.c_double()
    ue_pow_curr = ctypes.c_double()
    out_of_tol = ctypes.c_double()
    err = dll.ReadSpectrumEmask3G(
        handle,
        ctypes.byref(ref_pow_curr),
        ctypes.byref(ref_pow_avg),
        ctypes.byref(ref_pow_max),
        ctypes.byref(ue_pow_curr),
        ctypes.byref(out_of_tol),
        ctypes.byref(err_msg))
    powers = {}
    powers["REF_POW_CURR"] = ref_pow_curr.value
    powers["REF_POW_AVG"] = ref_pow_avg.value
    powers["REF_POW_MAX"] = ref_pow_max.value
    powers["UE_POW_CURR"] = ue_pow_curr.value
    powers["OUT_OF_TOL"] = out_of_tol.value
    return err, powers, err_msg.value


def ReadEvmWcdma3G(eqt, channel, stats_mode):
    """
    Starts a single shot measurement and returns results, depending on the statistics mode.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type channel: integer
    :param channel: selects the channel for the WCDMA measurements. Available values:
        - 0: DPCH (Dedicated Physical CHannel)
    :type stats_mode: str
    :param stats_mode: defines how the measurement is calculated. Available values:
        - "CURRENT": the current slot, i.e. the last result for all test
        points, is displayed.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots/evaluation periods measured is displayed;
        - "MAXMIN": maximum/minimum at each test point, the extreme value of all slots
        evaluation periods measured is displayed, i.e. the
        maximum or minimum, whichever has a larger absolute value.
    :rtype: tuple
        - integer: error code of the driver function
        - dict:
            - key: "PEAK_EVM": the error vector magnitude peak, depending
            on the statistics mode. Returned value range: 0.0 % to 100.0 %
            - key: "RMS_EVM": the error vector magnitude RMS, depending
            on the statistics mode. Returned value range: 0.0 % to 100.0 %
            - key: "OFFSET": the I/Q origin offset, depending
            on the statistics mode. Returned value range: -100.0 dB to 0.0 dB
            - key: "FREQUENCY_ERROR": the frequency error, depending on
            the statistics mode. Returned value range: -5000.0 Hz to 5000.0 Hz
            - key: "DOM_ERROR": the peak code domain error, depending
            on the statistics mode. Returned value range: -100.0 dB to 0.0 dB
            - key: "UE_POW_CURR": the UE power current.
            Returned value range: -100.0 dBm to 60.0 dBm
            - key: "OUT_OF_TOL": the out of tolerance value.
            Returned value range: 0.0 % to 100.0 %
            - key: "SLOT_NUMBER": the slot number. Returned value range: 0 to 14
        - str: error message of the driver function
    """
    eqt.get_logger().info("Read WCDMA EVM")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    evm_peak = ctypes.c_double()
    evm_rms = ctypes.c_double()
    iq_origin_offset = ctypes.c_double()
    freq_err = ctypes.c_double()
    peak_code_dom_err = ctypes.c_double()
    ue_pow_curr = ctypes.c_double()
    out_of_tol = ctypes.c_double()
    slot_number = ctypes.c_long()
    err = dll.ReadEvmWcdma3G(
        handle,
        ctypes.c_int(channel),
        ctypes.c_char_p(stats_mode),
        ctypes.byref(evm_peak),
        ctypes.byref(evm_rms),
        ctypes.byref(iq_origin_offset),
        ctypes.byref(freq_err),
        ctypes.byref(peak_code_dom_err),
        ctypes.byref(ue_pow_curr),
        ctypes.byref(out_of_tol),
        ctypes.byref(slot_number),
        ctypes.byref(err_msg))
    results = {}
    results["PEAK_EVM"] = evm_peak.value
    results["RMS_EVM"] = evm_rms.value
    results["OFFSET"] = iq_origin_offset.value
    results["FREQUENCY_ERROR"] = freq_err.value
    results["DOM_ERROR"] = peak_code_dom_err.value
    results["UE_POW_CURR"] = ue_pow_curr.value
    results["OUT_OF_TOL"] = out_of_tol.value
    results["SLOT_NUMBER"] = slot_number.value
    return err, results, err_msg.value


def ReadSpectrumEmaskMargin3G(eqt, stats_mode):
    """
    Returns the limit line margin values in the 8 limit line areas, ordered
    according to ascending frequencies.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type stats_mode: str
    :param stats_mode: defines how the measurement is calculated. Available values:
        - "CURRENT": the current slot, i.e. the last result for all test
        points, is displayed.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots/evaluation periods measured is displayed
        - "MAX": at each test point, the maximum value of all
        slots/evaluation periods measured is displayed.
    :rtype: tuple
        - integer: error code of the driver function
        - dict:
            - key: "-12.5_-8.5MHz" : margin value of -12.5_-8.5MHz interval
            - key: "-8.5_-7.5MHz"  : margin value of -8.5_-7.5MHz interval
            - key: "-7.5_-3.5MHz"  : margin value of -7.5_-3.5MHz interval
            - key: "-3.5_-2.5MHz"  : margin value of -3.5_-2.5MHz interval
            - key: "2.5_3.5MHz"    : margin value of 2.5_3.5MHz interval
            - key: "3.5_7.5MHz"    : margin value of 3.5_7.5MHz interval
            - key: "7.5_8.5MHz"    : margin value of 7.5_8.5MHz interval
            - key: "8.5_12.5MHz"   : margin value of 8.5_12.5MHz interval
        - str: error message of the driver function
    """
    eqt.get_logger().info("Read spectrum emission mask margin")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    margin_lim_area1 = ctypes.c_double()
    margin_lim_area2 = ctypes.c_double()
    margin_lim_area3 = ctypes.c_double()
    margin_lim_area4 = ctypes.c_double()
    margin_lim_area5 = ctypes.c_double()
    margin_lim_area6 = ctypes.c_double()
    margin_lim_area7 = ctypes.c_double()
    margin_lim_area8 = ctypes.c_double()
    err = dll.ReadSpectrumEmaskMargin3G(
        handle,
        ctypes.c_char_p(stats_mode),
        ctypes.byref(margin_lim_area1),
        ctypes.byref(margin_lim_area2),
        ctypes.byref(margin_lim_area3),
        ctypes.byref(margin_lim_area4),
        ctypes.byref(margin_lim_area5),
        ctypes.byref(margin_lim_area6),
        ctypes.byref(margin_lim_area7),
        ctypes.byref(margin_lim_area8),
        ctypes.byref(err_msg))
    results = {}
    results["-12.5_-8.5MHz"] = margin_lim_area1.value
    results["-8.5_-7.5MHz"] = margin_lim_area2.value
    results["-7.5_-3.5MHz"] = margin_lim_area3.value
    results["-3.5_-2.5MHz"] = margin_lim_area4.value
    results["2.5_3.5MHz"] = margin_lim_area5.value
    results["3.5_7.5MHz"] = margin_lim_area6.value
    results["7.5_8.5MHz"] = margin_lim_area7.value
    results["8.5_12.5MHz"] = margin_lim_area8.value
    return err, results, err_msg.value


def ReadSpectrumMfft3G(eqt, aclr_units):
    """
    Starts a single shot measurement and returns results
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type aclr_units: str
    :param aclr_units: distinguishes whether the ACLR results are expressed in
    absolute units (adjacent channels powers in dBm with a value range between
    -100.0 dBm and +60.0 dBm) or in relative units (ACLR in dB in the value range).
    Available values:
        - "ABS" : ACLR results expressed in absolute units (dBm).
        - "REL" : ACLR results expressed in relative units (dB).
    :rtype: tuple
        - integer: error code of the driver function
        - list: list of strings representing the measured values. If a value
        is "NAN", the value wasn't available.
            - Carrier power peak): current. Value range: -100.0 dBm to +60.0 dBm
            - Carrier power (RMS): current. Value range: -100.0 dBm to +60.0 dBm
            - ACLR (RMS): current (4 elements). Values range: -100.0 dBm to 0.0 dBm
            - ACLR (RMS): average (4 elements). Values range: -100.0 dBm to 0.0 dBm
            - ACLR (RMS): maximum (4 elements). Values range: -100.0 dBm to 0.0 dBm
            - OBW current. Value range: 0.00 MHz to 10.00 MHz
            - OBW average. Value range: 0.00 MHz to 10.00 MHz
            - OBW maximum. Value range: 0.00 MHz to 10.00 MHz
            - OBW left. Value range: -5.00 MHz to +5.00 MHz
            - OBW right. Value range: -5.00 MHz to +5.00 MHz
            - UE power current. Value range: -100.0 dBm to +60.0 dBm
            - Out of tolerance. Value range: 0 % to 100%
        - str: error message of the driver function
    """
    units = "absolute"
    if aclr_units == "REL":
        units = "relative"
    eqt.get_logger().info("Read %s adjacent channel leakage power ratio and "
                          "occupied bandwidth using fast Fourier transform method", units)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    values = ctypes.c_char_p('\x00' * 512)
    err = dll.ReadSpectrumMfft3G(
        handle,
        ctypes.c_char_p(aclr_units),
        512,
        values,
        ctypes.byref(err_msg))
    # Format results and check results size
    results = values.value.split(',')  # pylint: disable=E1101
    return err, results, err_msg.value


def CalculatePowerMatching3G(eqt, application):
    """
    Indicates whether and in which way the tolerances for the peak and
    RMS-averaged signal power have been exceeded.
    .. note:: the value RMS power minimum is returned in the maximum application only.
    It is omitted for the minimum and off commands.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type application: str
    :param application: defines the application of the power measurement.
    Available values:
        - "MAX" : maximum power application.
        - "MIN" : minimum power application.
        - "OFF" : off power application
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - dict: a dictionary containing the following verdicts strings:
            .. note:: each verdict is a integer but will be changed into a str
                - 0 => "OK"      : result within the tolerance
                - 1 => "INVALID" : invalid measurement
                - 2 => "UNDER"   : tolerance value underflow not matching
                - 3 => "OVER"    : tolerance value exceeded not matching
                - In "MIN" and "OFF" application, RMS power minimum verdict is
                "UNAVAILABLE"
            - key: "PEAK_POW_CURR": peak power current verdict
            - key: "PEAK_POW_AVG": peak power average verdict
            - key: "PEAK_POW_MAX": peak power maximum verdict
            - key: "RMS_POW_CURR": RMS power current verdict
            - key: "RMS_POW_AVG": RMS power average verdict
            - key: "RMS_POW_MAX": RMS power maximum verdict
            - key: "RMS_POW_MIN": RMS power minimum verdict
        - str : error message of the driver function
    """
    eqt.get_logger().info("Calculate %s power verdict", str(application))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    peak_pow_curr = ctypes.c_long()
    peak_pow_avg = ctypes.c_long()
    peak_pow_max = ctypes.c_long()
    rms_pow_curr = ctypes.c_long()
    rms_pow_avg = ctypes.c_long()
    rms_pow_max = ctypes.c_long()
    rms_pow_min = ctypes.c_long()
    err = dll.CalculatePowerMatching3G(
        handle,
        ctypes.c_char_p(application),
        ctypes.byref(peak_pow_curr),
        ctypes.byref(peak_pow_avg),
        ctypes.byref(peak_pow_max),
        ctypes.byref(rms_pow_curr),
        ctypes.byref(rms_pow_avg),
        ctypes.byref(rms_pow_max),
        ctypes.byref(rms_pow_min),
        ctypes.byref(err_msg))
    verdict_to_string = ["OK", "INVALID", "UNDER", "OVER"]
    verdicts = {}
    verdicts["PEAK_POW_CURR"] = verdict_to_string[peak_pow_curr.value]
    verdicts["PEAK_POW_AVG"] = verdict_to_string[peak_pow_avg.value]
    verdicts["PEAK_POW_MAX"] = verdict_to_string[peak_pow_max.value]
    verdicts["RMS_POW_CURR"] = verdict_to_string[rms_pow_curr.value]
    verdicts["RMS_POW_AVG"] = verdict_to_string[rms_pow_avg.value]
    verdicts["RMS_POW_MAX"] = verdict_to_string[rms_pow_max.value]
    if application == "MAX":
        verdicts["RMS_POW_MIN"] = verdict_to_string[rms_pow_min.value]
    else:
        verdicts["RMS_POW_MIN"] = "UNAVAILABLE"
    return err, verdicts, err_msg.value


def CalculateEvmWcdmaMatching3G(eqt, channel, stats_mode):
    """
    Starts a single shot measurement and returns results. Indicates whether and
    in which way the error limits for the scalar measured values have been exceeded,
    depending on the statistics mode
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type channel: integer
    :param channel: selects the channel for the WCDMA measurements. Available values:
        - 0: DPCH (Dedicated Physical CHannel)
    :type stats_mode: str
    :param stats_mode: defines how the measurement is calculated. Available values:
        - "CURRENT": the current slot, i.e. the last result for all test
        points, is displayed.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots/evaluation periods measured is displayed;
        - "MAXMIN": maximum/minimum at each test point, the extreme value of all slots
        evaluation periods measured is displayed, i.e. the
        maximum or minimum, whichever has a larger absolute value.
    :rtype: tuple
        - integer: error code of the driver function
        - dict: a dictionary containing the following verdicts strings:
            .. note:: each verdict is a integer but will be changed into a str
                - 0 => "OK"      : result within the tolerance
                - 1 => "INVALID" : invalid measurement
                - 2 => "UNDER"   : tolerance value underflow not matching
                - 3 => "OVER"    : tolerance value exceeded not matching
                - In "MIN" and "OFF" application, RMS power minimum verdict is
                "UNAVAILABLE"
            - key: "PEAK_EVM": peak error vector magnitude verdict
            - key: "RMS_EVM": RMS error vector magnitude verdict
            - key: "OFFEST": I/Q origin offset verdict
            - key: "FREQUENCY_ERROR": frequency error verdict
            - key: "DOM_ERROR": peak code domain error verdict
        - str: error message of the driver function
    """
    eqt.get_logger().info("Calculate error vector magnitude verdicts")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    evm_peak = ctypes.c_int()
    evm_rms = ctypes.c_int()
    iq_origin_offset = ctypes.c_int()
    freq_err = ctypes.c_int()
    peak_code_dom_err = ctypes.c_int()
    err = dll.CalculateEvmWcdmaMatching3G(
        handle,
        ctypes.c_int(channel),
        ctypes.c_char_p(stats_mode),
        ctypes.byref(evm_peak),
        ctypes.byref(evm_rms),
        ctypes.byref(iq_origin_offset),
        ctypes.byref(freq_err),
        ctypes.byref(peak_code_dom_err),
        ctypes.byref(err_msg))
    verdict_to_string = ["OK", "INVALID", "UNDER", "OVER"]
    results = {}
    results["PEAK_EVM"] = verdict_to_string[evm_peak.value]
    results["RMS_EVM"] = verdict_to_string[evm_rms.value]
    results["OFFSET"] = verdict_to_string[iq_origin_offset.value]
    results["FREQUENCY_ERROR"] = verdict_to_string[freq_err.value]
    results["DOM_ERROR"] = verdict_to_string[peak_code_dom_err.value]
    return err, results, err_msg.value


def CalculateSpectrumEmaskMatching3G(eqt, stats_mode):
    """
    Indicates whether and in which way the error limits in all areas of the
    spectrum emission mask (see controls) are exceeded. Section AB denotes
    the frequency domain between points.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type stats_mode: str
    :param stats_mode: defines how the measurement is calculated. Available values:
        - "CURRENT": the current slot, i.e. the last result for all test
        points, is displayed.
        - "AVERAGE": at each test point, a suitably defined average over
        all slots/evaluation periods measured is displayed
        - "MAXMIN": at each test point, the extreme value of all
        slots/ evaluation periods measured is displayed, i.e. the
        maximum or minimum, whichever has a larger absolute value.
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - dict: a dictionary containing the folowing verdicts
            .. note:: each verdict is a integer but will be changed into a str
                - 0 => "OK"      : result within the tolerance
                - 1 => "INVALID" : invalid measurement
                - 2 => "UNDER"   : tolerance value underflow not matching
                - 3 => "OVER"    : tolerance value exceeded not matching
            - key: "SECTION_AB": indicates whether and in which way the tolerance
            for the section AB has been exceeded
            - key: "SECTION_BC": indicates whether and in which way the tolerance
            for the section BC has been exceeded
            - key: "SECTION_CD": indicates whether and in which way the tolerance
            for the section CD has been exceeded
            - key: "SECTION_EF": indicates whether and in which way the tolerance
            for the section EF has been exceeded
            - key: "SECTION_FE": indicates whether and in which way the tolerance
            for the section FE has been exceeded
            - key: "SECTION_DC": indicates whether and in which way the tolerance
            for the section DC has been exceeded
            - key: "SECTION_CB": indicates whether and in which way the tolerance
            for the section CB has been exceeded
            - key: "SECTION_BA": indicates whether and in which way the tolerance
            for the section BA has been exceeded
        - str: error message of the driver function
    """
    eqt.get_logger().info("Calculate %s spectrum emission mask verdicts", stats_mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    section_ab = ctypes.c_long()
    section_bc = ctypes.c_long()
    section_cd = ctypes.c_long()
    section_ef = ctypes.c_long()
    section_fe = ctypes.c_long()
    section_dc = ctypes.c_long()
    section_cb = ctypes.c_long()
    section_ba = ctypes.c_long()
    err = dll.CalculateSpectrumEmaskMatching3G(
        handle,
        ctypes.c_char_p(stats_mode),
        ctypes.byref(section_ab),
        ctypes.byref(section_bc),
        ctypes.byref(section_cd),
        ctypes.byref(section_ef),
        ctypes.byref(section_fe),
        ctypes.byref(section_dc),
        ctypes.byref(section_cb),
        ctypes.byref(section_ba),
        ctypes.byref(err_msg))
    verdict_to_string = ["OK", "INVALID", "UNDER", "OVER"]
    verdicts = {}
    verdicts["SECTION_AB"] = verdict_to_string[section_ab.value]
    verdicts["SECTION_BC"] = verdict_to_string[section_bc.value]
    verdicts["SECTION_CD"] = verdict_to_string[section_cd.value]
    verdicts["SECTION_EF"] = verdict_to_string[section_ef.value]
    verdicts["SECTION_FE"] = verdict_to_string[section_fe.value]
    verdicts["SECTION_DC"] = verdict_to_string[section_dc.value]
    verdicts["SECTION_CB"] = verdict_to_string[section_cb.value]
    verdicts["SECTION_BA"] = verdict_to_string[section_ba.value]
    return err, verdicts, err_msg.value


def CalculateSpectrumMfftMatching3G(eqt, channel_number):
    """
    Indicates whether and in which way the error limits for the scalar measured
    values have been exceeded.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type channel_number: integer
    :param channel_number: sets the channel number. The channel frequency is the channel
    number times 5 MHz from the carrier frequency. Available values: -2, -1, 1 or 2
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - dict: a dictionary containing the following verdicts
            .. note:: each verdict is a integer but will be changed into a str
                - 0 => "OK"      : result within the tolerance
                - 1 => "INVALID" : invalid measurement
                - 2 => "UNDER"   : tolerance value underflow not matching
                - 3 => "OVER"    : tolerance value exceeded not matching
            - key: "ACLR_RMS_CURR": indicates whether and in which way the tolerance
            for the ACLR RMS current has been exceeded
            - key: "ACLR_RMS_AVG": indicates whether and in which way the tolerance
            for the ACLR RMS average has been exceeded
            - key: "ACLR_RMS_MAX": indicates whether and in which way the tolerance
            for the ACLR RMS maximum has been exceeded
            - key: "OBW_CURR": indicates whether and in which way the tolerance
            for the OBW current has been exceeded
            - key: "OBW_AVG": indicates whether and in which way the tolerance
            for the OBW average has been exceeded
            - key: "OBW_MAX": indicates whether and in which way the tolerance
            for the OBW maximum has been exceeded, depending on the statistics mode
        - str: error message of the driver function
    """
    eqt.get_logger().info("Calculate %s spectrum emission mask verdicts", channel_number)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    aclr_rms_curr = ctypes.c_long()
    aclr_rms_avg = ctypes.c_long()
    aclr_rms_max = ctypes.c_long()
    obw_curr = ctypes.c_long()
    obw_avg = ctypes.c_long()
    obw_max = ctypes.c_long()
    err = dll.CalculateSpectrumMfftMatching3G(
        handle,
        ctypes.c_int(channel_number),
        ctypes.byref(aclr_rms_curr),
        ctypes.byref(aclr_rms_avg),
        ctypes.byref(aclr_rms_max),
        ctypes.byref(obw_curr),
        ctypes.byref(obw_avg),
        ctypes.byref(obw_max),
        ctypes.byref(err_msg))
    verdict_to_string = ["OK", "INVALID", "UNDER", "OVER"]
    verdicts = {}
    verdicts["ACLR_RMS_CURR"] = verdict_to_string[aclr_rms_curr.value]
    verdicts["ACLR_RMS_AVG"] = verdict_to_string[aclr_rms_avg.value]
    verdicts["ACLR_RMS_MAX"] = verdict_to_string[aclr_rms_max.value]
    verdicts["OBW_CURR"] = verdict_to_string[obw_curr.value]
    verdicts["OBW_AVG"] = verdict_to_string[obw_avg.value]
    verdicts["OBW_MAX"] = verdict_to_string[obw_max.value]
    return err, verdicts, err_msg.value
