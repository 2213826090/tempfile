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
:summary: wrapper for 3G functions of RS CMU200
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetCellOff(eqt):
    """
    Wraps to SetCellOff function
    :raise TestEquipmentException: failed to call SetCellOff
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set cell OFF")
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCellOff3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellActive(eqt):
    """
    Wraps to SetCellActive function
    :raise TestEquipmentException: failed to call SetCellActive
    driver function
    :type eqt: RsCmu200
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
    :type eqt: RsCmu200
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
    imsi = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetReportedIMSI3G(handle, imsi, ctypes.byref(err_msg))
    return err, imsi.value, err_msg.value


def SetUplinkArfcn(eqt, arfcn):
    """
    Wraps to SetUplinkArfcn driver function
    :type eqt: RsCmu200
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


def SetCellPower(eqt, bch_power):
    """
    Wraps to SetCellPower driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type bch_power: double
    :param bch_power: cell power to set
    :raise TestEquipmentException: failed to call SetCellPower
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set cell power to %f", bch_power)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCellPower3G(
        handle,
        ctypes.c_double(bch_power),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMsPower(eqt, mspower):
    """
    Wraps to SetMsPower driver function
    :type eqt: RsCmu200
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


def SetLAC(eqt, lac):
    """
    Wraps to SetLAC driver function
    :type eqt: RsCmu200
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
    :type eqt: RsCmu200
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


def SetBand(eqt, band):
    """
    Wraps to SetBand 3G function
    :raise TestEquipmentException: call to SetBand 3G driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type band: str
    :param band: the band to set. Possible values:
        - "BAND1"
        - "BAND2"
        - "BAND3"
        - "BAND4"
        - "BAND5"
        - "BAND6"
        - "BAND7"
        - "BAND8"
        - "BAND9"
        - "BAND10"
    """
    eqt.get_logger().info("Set band to %s", band)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBand3G(handle, ctypes.c_char_p(band), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPhysicalChannelPower(eqt, level_ref, scpich_state, power):
    """
    Wraps to SetPhysicalChannelPower 3G function (RsCMU200 only)
    :raise TestEquipmentException: call to SetPhysicalChannelPower 3G driver function failed
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type level_ref: str
    :param level_ref: the reference level to use:
        - "PCPICH" : PCPICH mode
        - "OPOW" : total channel power mode
    :type scpich_state: str
    :param scpich_state: desired state for the S-CPICH channel
        - "ON"
        - "OFF"
    :type power: str
    :param power: ten powers separated by a ','. The order of powers in the
    str is:
        - 1) P-CPICH :
            - -147 dBm to -20 dBm : PCPICH mode
            - -30 dBm to 0 dBm : total channel power mode (OPOW)
        - 2) P-SCH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference : total channel power mode (OPOW)
        - 3) S-SCH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        - 4) P-CCPCH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        - 5) S-CCPCH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        - 6) PICH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        - 7) AICH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        - 8) DPDCH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        - 9) Power Offset :
            - 0 dB to +6 dB level reference : PCPICH mode
        - 10) S-CPICH :
            - -35 dB to +15 dB level reference : PCPICH mode
            - -30 dB to 0 dB level reference = total channel power mode (OPOW)
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Configure physical channels power")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPhysicalChannelPower3G(
        handle,
        ctypes.c_char_p(level_ref),
        ctypes.c_char_p(scpich_state),
        ctypes.c_char_p(power),
        ctypes.byref(err_msg))
    return err, err_msg.value
