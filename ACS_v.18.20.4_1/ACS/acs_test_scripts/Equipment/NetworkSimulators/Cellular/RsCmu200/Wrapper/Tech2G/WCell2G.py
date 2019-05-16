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
:summary: wrapper for 2G functions of RS CMU200
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetCellOff(eqt):
    """
    Wraps to SetCellOff 2G function
    :raise TestEquipmentException: call to SetCellOff
    driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: error code of the driver function
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
    :raise TestEquipmentException: call to SetCellActive
    driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
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
    :type eqt: RsCmu200
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


def GetReportedIMSI(eqt):
    """
    Wraps to GetReportedIMSI driver function
    :raise TestEquipmentException: failed to call GetReportedIMSI
    driver function
    :type eqt: RsCmu200
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


def SetLAC(eqt, lac):
    """
    Wraps to SetLACode function
    :raise TestEquipmentException: call to SetLACode
    driver function failed
    :type eqt: RsCmu200
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
    :type eqt: RsCmu200
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


def SetBand(eqt, band):
    """
    Wraps to SetBand function
    :raise TestEquipmentException: call to SetBand
    driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type band: str
    :param band: the BCH band to set
        Possible values:
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


def SetCellPower(eqt, bch_power):
    """
    This function sets the power level of the Broadcast Channel of the cell in dBm.
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type bch_power:
    :param bch_power:
    .. seealso:: SetCellPower(unsigned long p_lHandle, double p_dBchPower, char **p_pszErrorMsg)
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


def SetBchArfcn(eqt, arfcn):
    """
    Wraps to SetBchArfcn  function
    :raise TestEquipmentException: call to SetBchArfcn
    driver function failed
    :type eqt: RsCmu200
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


def SetPdtchArfcn(eqt, arfcn):
    """
    Wraps to SetPdtchArfcn 2G function
    :raise TestEquipmentException: call to SetPdtchArfcn
    driver function failed
    :type eqt: RsCmu200
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


def SetMSTxLevel(eqt, tx_level):
    """
    Wraps to 2G SetMSTxLevel function
    :type eqt: RsCmu200
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


def SetTchArfcn(eqt, arfcn):
    """
    Wraps to SetTchArfcn driver function
    :type eqt: RsCmu200
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
    err = dll.SetTchArfcn2G(
        handle,
        ctypes.c_int(arfcn),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetTchPower(eqt, power):
    """
    Wraps to SetTchPower 2G function
    :raise TestEquipmentException: call to SetTchPower driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type power: double
    :param double: the traffic channel power to set: -137 dBm to -10 dBm
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set traffic channel power to %f", power)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTchPower2G(
        handle,
        ctypes.c_double(power),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMobileDtx(eqt, state):
    """
    Wraps to SetDtx2G driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the Mobile DTX state to set (ON/OFF)
    :raise TestEquipmentException: call to SetDtx2G driver function failed
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set Mobile DTX to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    if state is "ON":
        state = True
    else:
        state = False
    err = dll.SetDtx2G(
        handle,
        ctypes.c_bool(state),
        ctypes.byref(err_msg))
    return err, err_msg.value
