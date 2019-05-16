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
:summary: wrapper for data 2G functions of RS CMU200
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetCustomMultislotConfig(
    eqt,
    main_timeslot,
    dl_slot_enabled,
    dl_slot_level,
    ul_slot_enabled,
        ul_slot_gamma):
    """
    Wraps to SetCustomMultislotConfig2G function
    :raise TestEquipmentException: call to SetCustomMultislotConfig2G driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type main_timeslot: integer
    :param main_timeslot: the number of the main time slot (an integer from 0 to 7)
    :type dl_slot_enabled: str
    :param dl_slot_enabled: a str of 8 "ON" | "OFF" words separated by ','
    :type dl_slot_level: str
    :param dl_slot_level: a str of 8 levels in dB (a double from -127.0 to +127.0)
    separated by ','
    :type ul_slot_enabled: str
    :param ul_slot_enabled: a str of 8 "ON" | "OFF" words separated by ','
    :type ul_slot_gamma: str
    :param ul_slot_gamma: a str of 8 gamma power control (an integer from 0 to 31)
    separated by ','
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    # Initialize enabled arrays
    dl_slot_enabled = dl_slot_enabled.split(",")
    c_dl_ts_enabled = (ctypes.c_char_p * 8)()
    for i in range(8):
        c_dl_ts_enabled[i] = dl_slot_enabled[i].strip()
    ul_slot_enabled = ul_slot_enabled.split(",")
    c_ul_ts_enabled = (ctypes.c_char_p * 8)()
    for i in range(8):
        c_ul_ts_enabled[i] = ul_slot_enabled[i].strip()
    # Initialize levels/gamma arrays
    dl_slot_level = dl_slot_level.split(",")
    c_dl_ts_levels = (ctypes.c_double * 8)()
    for i in range(8):
        c_dl_ts_levels[i] = float(dl_slot_level[i])
    c_ul_ts_levels = (ctypes.c_int * 8)()
    ul_slot_gamma = map(int, ul_slot_gamma.split(","))  # pylint: disable=W0141
    for i in range(8):
        c_ul_ts_levels[i] = ul_slot_gamma[i]
    # Do the job
    eqt.get_logger().info("Set custom multislot configuration")
    eqt.get_logger().debug("\tMain timeslot: %d", main_timeslot)
    eqt.get_logger().debug("\tEnabled DL slot: %s", dl_slot_enabled)
    eqt.get_logger().debug("\tDL slot levels: %s", dl_slot_level)
    eqt.get_logger().debug("\tEnabled UL slot: %s", ul_slot_enabled)
    eqt.get_logger().debug("\tUL slot gamma power controls: %s", ul_slot_gamma)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCustomMultislotConfig2G(handle,
                                         ctypes.c_int(main_timeslot), ctypes.byref(c_dl_ts_enabled),
                                         ctypes.byref(c_dl_ts_levels), ctypes.byref(c_ul_ts_enabled),
                                         ctypes.byref(c_ul_ts_levels), ctypes.byref(err_msg))
    return err, err_msg.value


def ProcessSignalingPacketDataActivation(eqt, state):
    """
    Wraps to ProcessSignalingPacketDataActivation2G function
    :raise TestEquipmentException: call to ProcessSignalingPacketDataActivation2G
        driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the desired state, possible values:
        - {OFF, ON, CTMA, CTMB, DISC, CRS, CBL, CDL, CRA, CRES, CREA,
           HAND, CLBS, CLBA, CBUL, SAT, CRUD}
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Process signal packet data")
    eqt.get_logger().debug("\tState: %s", state)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.ProcessSignalingPacketDataActivation2G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetNetworkPacketDataGprsCodingScheme(eqt, coding_scheme):
    """
    Wraps to SetNetworkPacketDataGprsCodingScheme2G function
    :raise TestEquipmentException: call to SetNetworkPacketDataGprsCodingScheme2G
        driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type coding_scheme: str
    :param coding_scheme: GPRS coding scheme for packet data channels:
        - CS1
        - CS2
        - CS3
        - CS4
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set GPRS coding scheme to %s", coding_scheme)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetNetworkPacketDataGprsCodingScheme2G(
        handle,
        ctypes.c_char_p(coding_scheme),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetNetworkPacketDataEgprsCodingScheme(eqt, context, coding_scheme):
    """
    Wraps to SetNetworkPacketDataEgprsCodingScheme2G function
    :raise TestEquipmentException: call to SetNetworkPacketDataEgprsCodingScheme2G
        driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type context: str
    :param context: the execution context, possible values:
        - "CONNECTED"
        - "NOT_CONNECTED"
    :type coding_scheme: str
    :param coding_scheme: EGPRS coding scheme for packet data channels:
        - {"MCS1", "MCS2", "MCS3", "MCS4", "MCS5", "MCS6",
           "MCS7", "MCS8", "MCS9"}
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set EGPRS coding scheme to %s", coding_scheme)
    eqt.get_logger().debug("\tContext: %s", context)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetNetworkPacketDataEgprsCodingScheme2G(
        handle,
        ctypes.c_char_p(context),
        ctypes.c_char_p(coding_scheme),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetBSSignalPacketDataMainTimeslot(eqt, main_timeslot):
    """
    Wraps to SetBSSignalPacketDataMainTimeslot2G function
    :raise TestEquipmentException: call to SetBSSignalPacketDataMainTimeslot2G
        driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type main_timeslot: integer
    :param main_timeslot: the new main timeslot value, range: 2 ... 6
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set data main timeslot to %d", main_timeslot)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    main_timeslot = ctypes.c_long(main_timeslot)
    err = dll.SetBSSignalPacketDataMainTimeslot2G(
        handle,
        main_timeslot,
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDataCallControlStatus(eqt):
    """
    Wraps to GetDataCallControlStatus2G driver function
    :raise TestEquipmentException: failed to call GetDataCallControlStatus2G
    driver function
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: the call control status
        - str: error message of the driver function
    """
    max_size = 32
    status = ctypes.c_char_p('\x00' * max_size)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    eqt.get_logger().info("Get data call control status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.GetDataCallControlStatus2G(
        handle,
        max_size,
        status,
        ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def SetDataTrafficChannel(eqt, traffic_channel):
    """
    Wraps to SetDataTrafficChannel2G driver function
    :raise TestEquipmentException: failed to call SetDataTrafficChannel2G
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type traffic_channel: integer
    :param traffic_channel: the call control status. Possible values (ranges:
        - GSM400 : 259 ... 293 | 306 ... 340
        - GSM850 : 128 ... 251
        - GSM900 : 0 ... 124 | 955 ... 1023
        - GSM1800: 512 ... 885
        - GSM1900: 512 ... 810
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    eqt.get_logger().info("Set data traffic channel to %d", traffic_channel)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetDataTrafficChannel2G(
        handle,
        ctypes.c_int(traffic_channel),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetPacketDataReferenceLevel(eqt):
    """
    Wraps to GetPacketDataReferenceLevel2G driver function.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - integer: the reference level for downlink channels
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call GetPacketDataReferenceLevel2G
    driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    level = ctypes.c_long()
    eqt.get_logger().info("Get packet data reference level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.GetPacketDataReferenceLevel2G(
        handle,
        ctypes.byref(level),
        ctypes.byref(err_msg))
    return err, level.value, err_msg.value


def SetPacketDataAutomatedSlotConfiguration(eqt, enabled):
    """
    Wraps to SetPacketDataAutomatedSlotConfiguration2G driver function.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type enabled: str
    :param enabled: the required enabled status. Possible values:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call SetPacketDataAutomatedSlotConfiguration2G
    driver function
    """
    eqt.get_logger().info("Set %s packet data automated slot configuration", enabled)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPacketDataAutomatedSlotConfiguration2G(
        handle,
        ctypes.c_char_p(enabled),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigurePacketDataMultislotPowerControl(eqt, ts_enabled, ts_levels):
    """
    Wraps to ConfigurePacketDataMultislotPowerControl2G driver function.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type ts_enabled: array
    :param ts_enabled: a str of 8 strings indicating the enabled status for
    each slot (slot range: 0 .. 7). Possible values:
        - "ON"
        - "OFF"
    :type ts_levels: array
    :param ts_levels: a str of 8 integers indicating the power control parameter
    in uplink for each slot (slot range: 0 .. 7)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call ConfigurePacketDataMultislotPowerControl2G
    driver function
    """
    eqt.get_logger().info("Configure data multislot power controls")
    eqt.get_logger().debug("\tEnabled slots: %s", ts_enabled)
    eqt.get_logger().debug("\tSlots level: %s", ts_levels)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    c_ts_levels = (ctypes.c_int * 8)()
    ts_levels = map(int, ts_levels.split(","))  # pylint: disable=W0141
    ts_enabled = map(lambda s: s.strip(), ts_enabled.split(","))  # pylint: disable=W0141
    for i in range(8):
        c_ts_levels[i] = ts_levels[i]
    c_ts_enabled = (ctypes.c_char_p * 8)()
    for i in range(8):
        c_ts_enabled[i] = ts_enabled[i]
    err = dll.ConfigurePacketDataMultislotPowerControl2G(
        handle,
        c_ts_enabled,
        c_ts_levels,
        ctypes.byref(err_msg))
    return err, err_msg.value


def ConfigurePacketDataMultislotConfig(eqt, ts_enabled, ts_levels):
    """
    Wraps to ConfigurePacketDataMultislotConfig2G driver function.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type ts_enabled: array
    :param ts_enabled: an array of strings indicating the enabled status for
    each slot (slot range: 0 .. 7). Possible values:
        - "ON"
        - "OFF"
    :type ts_levels: array
    :param ts_levels: an array of strings indicating the downlink
    RF level for each slot (slot range: 0 .. 7)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    :raise TestEquipmentException: failed to call ConfigurePacketDataMultislotConfig2G
    driver function
    """
    ts_enabled_str = ""
    ts_levels_str = ""
    c_ts_levels = (ctypes.c_double * 8)()
    for i in range(8):
        c_ts_levels[i] = float(ts_levels[i])
        if i == 0:
            ts_levels_str += ts_levels[i]
        else:
            ts_levels_str += ", " + ts_levels[i]
    c_ts_enabled = (ctypes.c_char_p * 8)()
    for i in range(8):
        c_ts_enabled[i] = ts_enabled[i].strip()
        if i == 0:
            ts_enabled_str += ts_enabled[i].strip()
        else:
            ts_enabled_str += ", " + ts_enabled[i].strip()
    eqt.get_logger().info("Configure data multislot configuration")
    eqt.get_logger().debug("\tEnabled slots: %s", ts_enabled_str)
    eqt.get_logger().debug("\tSlots level: %s", ts_levels_str)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ConfigurePacketDataMultislotConfig2G(
        handle,
        c_ts_enabled,
        c_ts_levels,
        ctypes.byref(err_msg))
    return err, err_msg.value
