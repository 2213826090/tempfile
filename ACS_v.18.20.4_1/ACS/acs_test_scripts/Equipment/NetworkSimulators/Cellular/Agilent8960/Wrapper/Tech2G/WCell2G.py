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
:summary: wrapper for 2G functions of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetCellOff(eqt):
    """
    Wraps to SetCellOff2G function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to SetCellOff2G driver function failed
    :rtype: tuple
    :return:
        - long: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set cell OFF")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellOff2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellActive(eqt):
    """
    Wraps to 2G SetCellActive function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to SetCellActive
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set cell ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellActive2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetServingCell(eqt, serving_cell):
    """
    Wraps to 2G SetServingCell function
    :raise TestEquipmentException: call to SetServingCell
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type serving_cell: str
    :param serving_cell: str representation of the desired
    serving cell
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set cell service to %s", serving_cell)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetServingCell2G(
        handle,
        ctypes.c_char_p(serving_cell),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDtx(eqt, state):
    """
    Wraps to 2G SetDtx function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: str representation of the desired state.
        Possible values:
            - "ON"
            - "OFF"
    :rtype: integer
    :return: error code of the driver function
    :raise TestEquipmentException: call to SetDtx
    driver function failed
    """
    eqt.get_logger().info("Set DTX %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDtx2G(handle, ctypes.c_char_p(state), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPagingMultiframe(eqt, mrf):
    """
    Wraps to 2G SetPagingMultiframe function
    :raise TestEquipmentException: call to SetPagingMultiframe
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mrf: integer
    :param mrf: number of multiframes paging subchannels (2 to 9)
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set multiframes # between paging subchannels to %d",
        mrf)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPagingMultiframe2G(
        handle,
        ctypes.c_int(mrf),
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
    err = dll.DataCallNetworkRelease2G(handle, ctypes.byref(err_msg))
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
    err = dll.CallEndAll2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetFrequencyHoppingState(eqt, state):
    """
    Wraps to 2G SetFrequencyHoppingState function
    :raise TestEquipmentException: call to SetFrequencyHoppingState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: String representation of the desired state:
        - "ON"
        - "OFF"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Turn frequency hopping %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetFrequencyHoppingState2G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMSTxLevel(eqt, tx_level):
    """
    Wraps to 2G SetMSTxLevel function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type tx_level: integer
    :param tx_level: value of the Tx level to set
    :raise TestEquipmentException: call to SetMSTxLevel
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set MS Tx level to %d", tx_level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMSTxLevel2G(
        handle,
        ctypes.c_int(tx_level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetMsTxLevel(eqt):
    """
    Wraps to 2G GetMsTxLevel function
    :raise TestEquipmentException: call to GetMsTxLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the MS TX level
    """
    eqt.get_logger().info("Get MS Tx level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    level = ctypes.c_int()
    err = dll.GetMsTxLevel2G(handle, ctypes.byref(level), ctypes.byref(err_msg))
    return err, level.value, err_msg.value


def SetChannelMode(eqt, chan_mode):
    """
    Wraps to SetChannelMode function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type chan_mode: str
    :param chan_mode: channel mode to use.
        Possible values are:
            - "FRSP"
            - "EFRS"
            - "HRSP"
    :raise TestEquipmentException: call to SetChannelMode
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set cell channel mode to %s", chan_mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetChannelMode2G(
        handle,
        ctypes.c_char_p(chan_mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPeriodicLocationUpdateTimer(eqt, period):
    """
    Wraps to SetPeriodicLocationUpdateTimer function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type period: integer
    :param period: timer value in decihours (0 to 255)
    :raise TestEquipmentException: call to SetPeriodicLocationUpdateTimer
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set T3212 timer to %d decihours", period)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPeriodicLocationUpdateTimer2G(
        handle,
        ctypes.c_int(period),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetBaTable(eqt, ba_table, bch_band):
    """
    Wraps to SetBaTable function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ba_table: array
    :param ba_table: the BA table to set
    :type bch_band: str
    :param bch_band: str representation of the BCH band
        Possible values:
            - "DCS"
            - "EGSM"
            - "GSM450"
            - "GSM480"
            - "GSM750"
            - "GSM850"
            - "PCS"
            - "PGSM"
            - "DCS"
            - "RGSM"
            - "TGSM810" => BECAREFUL (not supported for the moment)
    :raise TestEquipmentException: call to SetBaTable
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set the BA table ARFCN to %s for band %s",
        str(ba_table),
        bch_band)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ba_table = map(long, ba_table.split(","))  # pylint: disable=W0141
    c_ba_table = (ctypes.c_int * len(ba_table))()
    for i in range(len(ba_table)):
        c_ba_table[i] = ba_table[i]
    err = dll.SetBaTable2G(
        handle,
        ctypes.c_int(len(ba_table)),
        c_ba_table,
        ctypes.c_char_p(bch_band),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCaTable(eqt, ca_table):
    """
    Wraps to SetCaTable function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ca_table: array
    :param ca_table: the CA table to set
    :raise TestEquipmentException: call to SetCaTable
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set CA table ARFCN to %s in current band",
        str(ca_table))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    ca_table = map(long, ca_table.split(","))  # pylint: disable=W0141
    c_ca_table = (ctypes.c_int * len(ca_table))()
    for i in range(len(ca_table)):
        c_ca_table[i] = ca_table[i]
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCaTable2G(
        handle,
        ctypes.c_int(len(ca_table)),
        c_ca_table,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetLAC(eqt, lac):
    """
    Wraps to SetLACode function
    :raise TestEquipmentException: call to SetLACode
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type lac: long
    :param lac: local area code to set
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set local area code to %d", lac)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLACode2G(handle, ctypes.c_long(lac), ctypes.byref(err_msg))
    return err, err_msg.value


def GetLAC(eqt):
    """
    Wraps to GetLACode  function
    :raise TestEquipmentException: call to GetLACode
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: long
    :return: the LAC
    """
    eqt.get_logger().info("Get local area code")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    lac = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetLACode2G(handle, ctypes.byref(lac), ctypes.byref(err_msg))
    return err, lac.value, err_msg.value


def SetBchArfcn(eqt, arfcn):
    """
    Wraps to SetBchArfcn  function
    :raise TestEquipmentException: call to SetBchArfcn
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type arfcn: integer
    :param arfcn: the BCH arfcn to set
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set BCH ARFCN to %d", arfcn)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBchArfcn2G(handle, ctypes.c_int(arfcn), ctypes.byref(err_msg))
    return err, err_msg.value


def SetBand(eqt, band):
    """
    Wraps to SetBand function
    :raise TestEquipmentException: call to SetBand
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type band: str
    :param band: the BCH band to set. Possible values:
            - "DCS"
            - "EGSM"
            - "GSM450"
            - "GSM480"
            - "GSM750"
            - "GSM850"
            - "PCS"
            - "PGSM"
            - "RGSM"
            - "TGSM810"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set band to %s", band)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBand2G(handle, ctypes.c_char_p(band), ctypes.byref(err_msg))
    return err, err_msg.value


def PreConfigureBandArfcn(eqt, band, arfcn):
    """
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    """
    eqt.get_logger().info(
        "Preconfigure ARFCN to %d for band %s",
        arfcn,
        band)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.PreConfigureBandArfcn2G(
        handle,
        ctypes.c_char_p(band),
        ctypes.c_int(arfcn),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellPower(eqt, bch_power):
    """
    Sets the power level of the broadcast channel of the cell in dBm.
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type bch_power: float
    :param bch_power:
    """
    eqt.get_logger().info("Set cell power to %f", bch_power)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellPower2G(
        handle,
        ctypes.c_double(bch_power),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetTchArfcn(eqt, arfcn):
    """
    Wraps to SetTchArfcn driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type arfcn: integer
    :param arfcn: the TCH arfcn to set
    :raise TestEquipmentException: call to SetTchArfcn driver function failed
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set TCH arfcn to %d", arfcn)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTchArfcn2G(handle, ctypes.c_int(arfcn), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPdtchArfcn(eqt, arfcn):
    """
    Wraps to SetPdtchArfcn 2G function
    :raise TestEquipmentException: call to SetPdtchArfcn
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type arfcn: integer
    :param arfcn: the ARFCN of the downlink and uplink PDTCH to set.
    A valid value can be 512. An incorrect value can be 1024
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set uplink and downlink PDTCH ARFCN to %d", arfcn)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPdtchArfcn2G(
        handle,
        ctypes.c_int(arfcn),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMCCode(eqt, code):
    """
    Wraps to SetMCCode 2G function
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
    err = dll.SetMCCode2G(handle, ctypes.c_int(code), ctypes.byref(err_msg))
    return err, err_msg.value


def SetMNCode(eqt, code):
    """
    Wraps to SetMNCode 2G function
    :raise TestEquipmentException: call to SetMNCode
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: the Mobile Network Code to set.
    An integer from 0 to 999 when band is set to PCS or GSM850.
    An integer from 0 to 99 for other bands.
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set mobile network code to %d", code)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMNCode2G(handle, ctypes.c_int(code), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPagingIMSI(eqt, paging_imsi):
    """
    Wraps to SetPagingIMSI driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type paging_imsi: str
    :param paging_imsi: the paging IMSI to set
    :raise TestEquipmentException: failed to call SetPagingIMSI
    driver function
    """
    eqt.get_logger().info("Set paging IMSI to %s", paging_imsi)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPagingIMSI2G(
        handle,
        ctypes.c_char_p(paging_imsi),
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
    paging_imsi = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetPagingIMSI2G(handle, paging_imsi, ctypes.byref(err_msg))
    return err, paging_imsi.value, err_msg.value


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
    reported_imsi = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetReportedIMSI2G(handle, reported_imsi, ctypes.byref(err_msg))
    return err, reported_imsi.value, err_msg.value


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
    reported_imei = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetReportedIMEI2G(handle, reported_imei, ctypes.byref(err_msg))
    return err, reported_imei.value, err_msg.value


def GetTxPower(eqt):
    """
    Wraps to 2G GetTxPower function
    :raise TestEquipmentException: call to GetTxPower
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the str representation of TX power or "NA" if the value
    wasn't available when queried
    """
    eqt.get_logger().info("Get Tx Power")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    power = ctypes.c_char_p('\x00' * 32)
    err = dll.GetTxPower2G(handle, 32, power, ctypes.byref(err_msg))
    return err, power.value, err_msg.value


def InitOrfsMeasurement(eqt):
    """
    Wraps to 2G InitOrfsMeasurement function
    :raise TestEquipmentException: call to InitOrfsMeasurement
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Start ORFS measurement")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.InitOrfsMeasurement2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetDtmState(eqt, state):
    """
    Wraps to SetDtmState2G function
    :raise TestEquipmentException: call to SetDtmState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: String representation of the desired state:
        - "ON"
        - "OFF"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Turn DTM %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDtmState(handle, ctypes.c_char_p(state), ctypes.byref(err_msg))
    return err, err_msg.value


def ExecuteExternalHandover(eqt):
    """
    Wraps to ExecuteExternalHandover2G function
    :raise TestEquipmentException: call to ExecuteExternalHandover2G
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :return: error code of the driver function
    """
    eqt.get_logger().info("Execute external handover")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ExecuteExternalHandover2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value
