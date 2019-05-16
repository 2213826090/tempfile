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
:summary: wrapper for 3G functions of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetCellOff(eqt):
    """
    Wraps to SetCellOff function
    :raise TestEquipmentException: failed to call SetCellOff
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set cell OFF")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellOff3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellActive(eqt):
    """
    Wraps to SetCellActive function
    :raise TestEquipmentException: failed to call SetCellActive
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set cell ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellActive3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetPagingService(eqt, ps):
    """
    Wraps to SetPagingService function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ps: str
    :param ps: the paging service to set. Possible values:
        - "AMR"
        - "GPRS"
        - "RBT"
    :raise TestEquipmentException: failed to call SetPagingService
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set paging service to %s", ps)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPagingService3G(
        handle,
        ctypes.c_char_p(ps),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPSDomainInformation(eqt, domain):
    """
    Wraps to SetPSDomainInformation function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type domain: str
    :param domain: the PS domain information to set. Possible values:
        - "ABSENT"
        - "PRESENT"
    :raise TestEquipmentException: call of SetPSDomainInformation
    driver function failed
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Set PS domain information to %s",
        domain)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPSDomainInformation3G(
        handle,
        ctypes.c_char_p(domain),
        ctypes.byref(err_msg))
    return err, err_msg.value


def DataCallNetworkRelease(eqt):
    """
    Wraps to DataCallNetworkRelease driver function
    :raise TestEquipmentException: failed to call DataCallNetworkRelease
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Release data call")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.DataCallNetworkRelease3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def CallEndAll(eqt):
    """
    Wraps to CallEndAll driver function
    :raise TestEquipmentException: failed to call CallEndAll
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Release all calls")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.CallEndAll3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellPower(eqt, bchPower):
    """
    Wraps to SetCellPower driver function
    :type bchPower: double
    :param bchPower: cell power to set
    :raise TestEquipmentException: failed to call SetCellPower
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set cell power to %f", bchPower)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellPower3G(
        handle,
        ctypes.c_double(bchPower),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetRegistrationState(eqt):
    """
    Wraps to GetRegistrationState driver function
    :raise TestEquipmentException: failed to call GetRegistrationState
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the UE registration state
    """
    eqt.get_logger().info("Get registration state")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    state = ctypes.c_char_p('\x00' * 1024)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetRegistrationState3G(handle, state, ctypes.byref(err_msg))
    return err, state.value, err_msg.value


def SetSecurityProcedure(eqt, sp):
    """
    Wraps to SetSecurityProcedure driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sp: str
    :param sp: the security procedure to use. Possible values:
        - "NONE"
        - "AUTHONLY"
        - "AUTHINT"
        - "AIC"
    :raise TestEquipmentException: failed to call SetSecurityProcedure
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set security procedure to %s", sp)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSecurityProcedure3G(
        handle,
        ctypes.c_char_p(sp),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetAuthenticationKey(eqt, K1, K2, K3, K4):
    """
    Wraps to SetAuthenticationKey driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type K1: str
    :param K1: part 1 of the key
    :type K2: str
    :param K2: part 2 of the key
    :type K3: str
    :param K3: part 3 of the key
    :type K4: str
    :param K4: part 4 of the key
    :raise TestEquipmentException: failed to call SetAuthenticationKey
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Set authentication key to %s - %s - %s - %s",
        K1,
        K2,
        K3,
        K4)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAuthenticationKey3G(
        handle,
        ctypes.c_char_p(K1),
        ctypes.c_char_p(K2),
        ctypes.c_char_p(K3),
        ctypes.c_char_p(K4),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetAuthenticationAlgorithm(eqt, algo):
    """
    Wraps to SetAuthenticationAlgorithm driver function
    :type algo: integer
    :param algo: the algorithm to use for authentication
    :raise TestEquipmentException: failed to call SetAuthenticationAlgorithm
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set authentication algorithm to %d", algo)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAuthenticationAlgorithm3G(
        handle,
        ctypes.c_int(algo),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetMMStatus(eqt):
    """
    Wraps to GetMMStatus driver function
    :raise TestEquipmentException: failed to call GetMMStatus
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the MM status. Possible values:
        - "MMSTATUS1": default returned value
        - "MMSTATUS2"
        - "MMSTATUS3"
        - "MMSTATUS4"
        - "MMSTATUS5"
    """
    eqt.get_logger().info("Get MM status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    status = ctypes.c_char_p('\x00' * 16)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetMMStatus3G(handle, 16, status, ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def GetGMMStatus(eqt):
    """
    Wraps to GetGMMStatus driver function
    :raise TestEquipmentException: failed to call GetGMMStatus
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the GMM status. Possible values:
        - "ATT": default returned value
        - "ATTINC"
        - "DET"
        - "NONE"
    """
    eqt.get_logger().info("Get GMM status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    state = ctypes.c_char_p('\x00' * 32)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetGMMStatus3G(handle, 32, state, ctypes.byref(err_msg))
    return err, state.value, err_msg.value


def SetLAC(eqt, lac):
    """
    Wraps to SetLAC driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type lac: integer
    :param lac: the LAC to set
    :raise TestEquipmentException: failed to call SetLAC
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set local area code to %d", lac)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLAC3G(handle, ctypes.c_int(lac), ctypes.byref(err_msg))
    return err, err_msg.value


def GetLAC(eqt):
    """
    Wraps to GetLAC driver function
    :raise TestEquipmentException: failed to call GetLAC
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the LAC
    """
    eqt.get_logger().info("Get local area code")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    lac = ctypes.c_uint()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetLAC3G(handle, ctypes.byref(lac), ctypes.byref(err_msg))
    return err, lac.value, err_msg.value


def SetDownlinkArfcn(eqt, arfcn):
    """
    Wraps to SetDownlinkArfcn driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type arfcn: integer
    :param arfcn: the downlink ARFCN to set
    :raise TestEquipmentException: failed to call SetDownlinkArfcn
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set downlink ARFCN to %d", arfcn)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDownlinkArfcn3G(
        handle,
        ctypes.c_long(arfcn),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDownlinkArfcn(eqt):
    """
    Wraps to GetDownlinkArfcn driver function
    :raise TestEquipmentException: failed to call GetDownlinkArfcn
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the downlink ARFCN
    """
    eqt.get_logger().info("Get downlink ARFCN")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    arfcn = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetDownlinkArfcn3G(
        handle,
        ctypes.byref(arfcn),
        ctypes.byref(err_msg))
    return err, arfcn.value, err_msg.value


def SetUplinkArfcn(eqt, arfcn):
    """
    Wraps to SetUplinkArfcn driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type arfcn: integer
    :param arfcn: the uplink ARFCN
    :raise TestEquipmentException: failed to call SetUplinkArfcn
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set uplink ARFCN to %d", arfcn)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetUplinkArfcn3G(
        handle,
        ctypes.c_long(arfcn),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetUplinkArfcn(eqt):
    """
    Wraps to GetUplinkArfcn driver function
    :raise TestEquipmentException: failed to call GetUplinkArfcn
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the uplink ARFCN
    """
    eqt.get_logger().info("Get uplink ARFCN")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    arfcn = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetUplinkArfcn3G(
        handle,
        ctypes.byref(arfcn),
        ctypes.byref(err_msg))
    return err, arfcn.value, err_msg.value


def SetUplinkChannelMode(eqt, state):
    """
    Wraps to SetUplinkChannelMode driver function
    :raise TestEquipmentException: failed to call SetUplinkChannelMode
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: str representation of the desired state:
        - "ON"
        - "OFF"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set uplink channel mode to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetUplinkChannelMode3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetUplinkChannelMode(eqt):
    """
    Wraps to GetUplinkChannelMode driver function
    :raise TestEquipmentException: failed to call GetUplinkChannelMode
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the uplink ARFCN
    """
    eqt.get_logger().info("Get uplink channel mode")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    mode = ctypes.c_short()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetUplinkChannelMode3G(
        handle,
        ctypes.byref(mode),
        ctypes.byref(err_msg))
    return err, mode.value, err_msg.value


def SetTransmitSIB5bis(eqt, sibBand):
    """
    Wraps to SetTransmitSIB5bis driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sibBand: str
    :param sibBand: the transmit SIB5bis to set. Possible values:
        - "ALL"
        - "BAND10"
        - "BAND4"
        - "BAND9"
        - "BAND49"
        - "NONE"
        - "SBAN"
    :raise TestEquipmentException: failed to call SetTransmitSIB5bis
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set transmit SIB5bis to %s", sibBand)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTransmitSIB5bis3G(
        handle,
        ctypes.c_char_p(sibBand),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetBchBandIndicatorState(eqt, state):
    """
    Wraps to SetBchBandIndicatorState driver function
    :raise TestEquipmentException: failed to call SetBchBandIndicatorState
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: str representation of the desired state
        - "ON"
        - "OFF"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Turn BCH band indicator %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBchBandIndicatorState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetBandArbitrator(eqt, band):
    """
    Wraps to SetBandArbitrator driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type band: str
    :param band: the band arbitrator to set. Possible values:
        - "BAND5"
        - "BAND6"
    :raise TestEquipmentException: failed to call SetBandArbitrator
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set band arbitrator to %s", band)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBandArbitrator3G(
        handle,
        ctypes.c_char_p(band),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetGsmNeighborCellStates(eqt, states):
    """
    Wraps to SetGsmNeighborCellStates driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type states: array
    :param states: contains the states of the 8 cells to set (see doc1)
    :raise TestEquipmentException: failed to call SetGsmNeighborCellStates
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set GSM frequency neighbor cell states")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    size = len(states)
    c_states = (ctypes.c_int * size)()
    for i in range(size):
        c_states[i] = states[i]
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetGsmNeighborCellStates3G(
        handle,
        c_states,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetInterFreqNeighborCellStates(eqt, states):
    """
    Wraps to SetInterFreqNeighborCellStates driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type states: array
    :param states: contains inter frequency neighbor states
    of the 8 cells to set
    :raise TestEquipmentException: failed to call SetInterFreqNeighborCellStates
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set inter frequency neighbor cell states")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    size = len(states)
    c_states = (ctypes.c_int * size)()
    for i in range(size):
        c_states[i] = states[i]
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetInterFreqNeighborCellStates3G(
        handle,
        c_states,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetIntraFreqNeighborCellStates(eqt, states):
    """
    Wraps to SetIntraFreqNeighborCellStates driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type states: array
    :param states: contains intra frequency neighbor states
    of the 8 cells to set
    :raise TestEquipmentException: failed to call SetIntraFreqNeighborCellStates
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set intra frequency neighbor cell states")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    size = len(states)
    c_states = (ctypes.c_int * size)()
    for i in range(size):
        c_states[i] = states[i]
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetIntraFreqNeighborCellStates3G(
        handle,
        c_states,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDrxCycle(eqt, drx):
    """
    Wraps to SetDrxCycle driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type drx: integer
    :param drx: DRX cycle length to set. Possible values:
        - "FRAM64"
        - "FRAM128"
        - "FRAM256"
        - "FRAM512"
    :raise TestEquipmentException: failed to call SetDrxCycle
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set DRX cycle length to %s", drx)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDrxCycle3G(handle, ctypes.c_char_p(drx), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPeriodicLocationUpdateTimer(eqt, period):
    """
    Wraps to SetPeriodicLocationUpdateTimer driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type period: integer
    :param period: T312 link establishment timer to set
    (in decihours 0 to 255)
    :raise TestEquipmentException: failed to call SetPeriodicLocationUpdateTimer
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Set periodic location update timer to %d",
        period)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPeriodicLocationUpdateTimer3G(
        handle,
        ctypes.c_int(period),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSrbConfigControl(eqt, state):
    """
    Wraps to SetSrbConfigControl driver function
    :raise TestEquipmentException: failed to call SetSrbConfigControl
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: str representation of the desired state:
        - "ON"
        - "OFF"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Turn SRB configuration control %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSrbConfigControl3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetScramblingCode(eqt, code):
    """
    Wraps to SetScramblingCode driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: the scrambling code to set
    :raise TestEquipmentException: failed to call SetScramblingCode
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set scrambling code to %d", code)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetScramblingCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetScramblingCode(eqt):
    """
    Wraps to GetScramblingCode driver function
    :raise TestEquipmentException: failed to call GetScramblingCode
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the scrambling code
    """
    eqt.get_logger().info("Get scrambling code")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    code = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetScramblingCode3G(
        handle,
        ctypes.byref(code),
        ctypes.byref(err_msg))
    return err, code.value, err_msg.value


def SetDpchLevel(eqt, level):
    """
    Wraps to SetDpchLevel driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type level: double
    :param level: the DPCH level to set
    :raise TestEquipmentException: failed to call SetDpchLevel
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set DPCH level to %f", level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDpchLevel3G(
        handle,
        ctypes.c_double(level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDpchLevel(eqt):
    """
    Wraps to GetDpchLevel driver function
    :raise TestEquipmentException: failed to call GetDpchLevel
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: the DPCH level
    """
    eqt.get_logger().info("Get DPCH level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    lvl = ctypes.c_double()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetDpchLevel3G(handle, ctypes.byref(lvl), ctypes.byref(err_msg))
    return err, lvl.value, err_msg.value


def SetDpchState(eqt, state):
    """
    Wraps to SetDpchState driver function
    :raise TestEquipmentException: failed to call SetDpchState
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: str representation of the desired state:
        - "ON"
        - "OFF"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set DPCH state %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDpchState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMsPower(eqt, mspower):
    """
    Wraps to SetMsPower driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mspower: integer
    :param mspower: the expected power level from the UE to set
    :raise TestEquipmentException: failed to call SetMsPower
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set MS power level to %d", mspower)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMsPower3G(
        handle,
        ctypes.c_long(mspower),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetMsPower(eqt):
    """
    Wraps to GetMsPower driver function
    :raise TestEquipmentException: failed to call GetMsPower
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the expected power level from the UE
    """
    eqt.get_logger().info("Get MS power level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    mspower = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetMsPower3G(
        handle,
        ctypes.byref(mspower),
        ctypes.byref(err_msg))
    return err, mspower.value, err_msg.value


def SetRbtChannelType(eqt, channel_type):
    """
    Wraps to SetRbtChannelType driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type channel_type: str
    :param channel_type: the RBT channel type to set:
        - "HSDParmc12" : (12.2k RMC + HSDPA)
        - "HSParmc12"  : (12.2k RMC + HSPA
        - "HSPA"
        - "RMC12"      : (12.2k RMC)
        - "RMC64"      : (64k RMC)
        - "RMC144"     : (144k RMC)
        - "RMC384"     : (384k RMC)
        - "RMC33NC"    : (33k No Coding RMC)
        - "RMCAM1264"  : (12.2k UL/64k DL AM RMC) (active cell operating mode only)
        - "RMCAM12144" : (12.2k UL/144k DL AM RMC) (active cell operating mode only)
        - "RMCAM12384" : (12.2k UL/384k DL AM RMC) (active cell operating mode only)
        - "RMCAM64384" : (64k UL/384k DL AM RMC) (active cell operating mode only)
    :raise TestEquipmentException: failed to call SetRbtChannelType
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set RBT channel type to %s", channel_type)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetRbtChannelType3G(
        handle,
        ctypes.c_char_p(channel_type),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMCCode(eqt, code):
    """
    Wraps to SetMCCode 3G function
    :raise TestEquipmentException: call to SetMCCode
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: the Mobile Country Code to set.
    An integer from 0 to 999.
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set mobile country code to %d", code)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMCCode3G(handle, ctypes.c_int(code), ctypes.byref(err_msg))
    return err, err_msg.value


def SetMNCode(eqt, code):
    """
    Wraps to SetMNCode 3G function
    :raise TestEquipmentException: call to SetMNCode
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: the Mobile Network Code to set.
    An integer from 0 to 99 for other bands.
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set mobile network code to %d", code)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMNCode3G(handle, ctypes.c_int(code), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPagingIMSI(eqt, paging_IMSI):
    """
    Wraps to SetPagingIMSI driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type paging_IMSI: str
    :param paging_IMSI: the paging IMSI to set
    :raise TestEquipmentException: failed to call SetPagingIMSI
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set paging IMSI to %s", paging_IMSI)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPagingIMSI3G(
        handle,
        ctypes.c_char_p(paging_IMSI),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetPagingIMSI(eqt):
    """
    Wraps to GetPagingIMSI driver function
    :raise TestEquipmentException: failed to call GetPagingIMSI
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the paging IMSI
    """
    eqt.get_logger().info("Get paging IMSI")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    imsi = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetPagingIMSI3G(handle, imsi, ctypes.byref(err_msg))
    return err, imsi.value, err_msg.value


def GetReportedIMSI(eqt):
    """
    Wraps to GetReportedIMSI driver function
    :raise TestEquipmentException: failed to call GetReportedIMSI
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the reported IMSI
    """
    eqt.get_logger().debug("Get reported IMSI")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    imsi = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetReportedIMSI3G(handle, imsi, ctypes.byref(err_msg))
    return err, imsi.value, err_msg.value


def GetReportedIMEI(eqt):
    """
    Wraps to GetReportedIMEI driver function
    :raise TestEquipmentException: failed to call GetReportedIMEI
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the reported IMEI
    driver function
    """
    eqt.get_logger().debug("Get reported IMEI")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    imei = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetReportedIMEI3G(handle, imei, ctypes.byref(err_msg))
    return err, imei.value, err_msg.value


def ClearUeInfo(eqt):
    """
    Wraps to ClearUeInfo driver function
    :raise TestEquipmentException: failed to call ClearUeInfo
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Clear UE information")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ClearUeInfo3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def GetCurrentServiceType(eqt):
    """
    Wraps to GetCurrentServiceType driver function
    :raise TestEquipmentException: failed to call GetCurrentServiceType
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the current service type of the cell
    """
    eqt.get_logger().info("Get current service type")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    service_type = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetCurrentServiceType3G(handle, 256, service_type,
                                      ctypes.byref(err_msg))
    return err, service_type.value, err_msg.value


def GetChannelPower(eqt):
    """
    Wraps to 3G GetChannelPower function
    :raise TestEquipmentException: call to GetChannelPower
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the str representation of channel power or "NA" if the value
    wasn't available when queried
    """
    eqt.get_logger().info("Get channel power")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    power = ctypes.c_char_p('\x00' * 32)
    err = dll.GetChannelPower3G(handle, 32, power, ctypes.byref(err_msg))
    return err, power.value, err_msg.value


def InitWCPMeasurement(eqt):
    """
    Wraps to 3G InitWCPMeasurement function
    :raise TestEquipmentException: call to InitWCPMeasurement
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Start WCP measurement")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.InitWCPMeasurement3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetBchUpdatePageState(eqt, state):
    """
    Wraps to 3G SetBchUpdatePageState function
    :raise TestEquipmentException: call to SetBchUpdatePageState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set BCH update page state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBchUpdatePageState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetBchUpdatePageState(eqt):
    """
    Wraps to 3G GetBchUpdatePageState function
    :raise TestEquipmentException: call to GetBchUpdatePageState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: The BCCH Update Page state
                - AUTO
                - INH
    """
    eqt.get_logger().info("Get BCH update page state")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    state = ctypes.c_char_p('\x00' * 8)
    err = dll.GetBchUpdatePageState(handle, state, ctypes.byref(err_msg))
    return err, state.value, err_msg.value


def ExecuteExternalHandover(eqt):
    """
    Wraps to ExecuteExternalHandover3G function
    :raise TestEquipmentException: call to ExecuteExternalHandover3G
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :return: error code of the driver function
    """
    eqt.get_logger().info("Execute external handover")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ExecuteExternalHandover3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def ExecuteHardHandover(eqt):
    """
    Wraps to ExecuteHardHandover3G function
    :raise TestEquipmentException: call to ExecuteHardHandover3G
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :return: error code of the driver function
    """
    eqt.get_logger().info("Execute hard handover")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ExecuteHardHandover3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def GetPcrDownlinkArfcn(eqt):
    """
    Wraps to GetPcrDownlinkArfcn3G driver function
    :raise TestEquipmentException: failed to call GetPcrDownlinkArfcn3G
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: The PCR downlink ARFCN
    """
    eqt.get_logger().info("Get PCR downlink ARFCN")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    pcr_arfcn = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetPcrDownlinkArfcn3G(
        handle,
        ctypes.byref(pcr_arfcn),
        ctypes.byref(err_msg))
    return err, pcr_arfcn.value, err_msg.value
