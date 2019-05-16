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
:summary: wrapper for data 3G functions of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetDutIpAddress(eqt, ip_num, ip_addr):
    """
    Wraps to SetDutIpAddress 3G function
    :raise TestEquipmentException: call to SetDutIpAddress
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_num: integer
    :param ip_num: number of the IP address to set (1 to 4)
    :type ip_addr: str
    :param ip_addr: the IP address to set
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set device under test IP address %d to %s",
        ip_num,
        ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutIpAddress3G(
        handle,
        ctypes.c_int(ip_num),
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutIpAddress(eqt, ip_num):
    """
    Wraps to GetDutIpAddress 3G function
    :raise TestEquipmentException: call to GetDutIpAddress
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_num: integer
    :param ip_num: the number of the IP address to return (1 to 4)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: the IP address of the device under test
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get device under test IP address %d", ip_num)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDutIpAddress3G(
        handle,
        32,
        ip_num,
        ip_addr,
        ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def SetDutPrimaryDns(eqt, ip_addr):
    """
    Wraps to SetDutPrimaryDns 3G function
    :raise TestEquipmentException: call to SetDutPrimaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: the IP address to set
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set device under test primary DNS IP address to %s",
        ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutPrimaryDns3G(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutPrimaryDns(eqt):
    """
    Wraps to GetDutPrimaryDns 3G function
    :raise TestEquipmentException: call to GetDutPrimaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: the IP address of the device under test primary DNS
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get device under test primary DNS IP address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDutPrimaryDns3G(handle, 32, ip_addr, ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def SetDutSecondaryDns(eqt, ip_addr):
    """
    Wraps to SetDutSecondaryDns 3G function
    :raise TestEquipmentException: call to SetDutSecondaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: the IP address to set
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set device under test secondary DNS IP address to %s",
        ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutSecondaryDns3G(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutSecondaryDns(eqt):
    """
    Wraps to GetDutSecondaryDns 3G function
    :raise TestEquipmentException: call to GetDutSecondaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: the IP address of the device under test secondary DNS
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get device under test secondary DNS IP address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDutSecondaryDns3G(handle, 32, ip_addr, ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def GetDataConnectionStatus(eqt):
    """
    Wraps to GetDataConnectionStatus 3G function
    :raise TestEquipmentException: call to GetDataConnectionStatus
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: the data connection status. Possible returned values:
            - "OFF": Default returned value
            - "IDLE"
            - "ATTACHING"
            - "ATTACHED"
            - "ATTACHED_INCOMPLETE"
            - "DETACHING"
            - "IDLE"
            - "OFF" => Default returned value
            - "DETACHED"
            - "PDP_ACTIVATING"
            - "PDP_ACTIVE"
            - "PDP_DEACTIVATING"
        - str: error message of the driver function
    """
    eqt.get_logger().debug("Get data connection status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    status = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDataConnectionStatus3G(
        handle,
        32,
        status,
        ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def SetInitialPsDataRRCState(eqt, state):
    """
    Wraps to SetInitialPsDataRRCState 3G function
    :raise TestEquipmentException: call to SetInitialPsDataRRCState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the desired state. Possible values:
        - "DCH": factory value
        - "FACH"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set initial PS data RRC state to %s",
        state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetInitialPsDataRRCState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMsCellPowerTarget(eqt, level):
    """
    Wraps to SetMsCellPowerTarget 3G function
    :raise TestEquipmentException: call to SetMsCellPowerTarget
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type level: integer
    :param level: the power level to set (-61dBm to 28dBm, resolution: 1)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set mobile station expected power level to %ddBm",
        level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMsCellPowerTarget3G(
        handle,
        ctypes.c_int(level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetMsCellPowerTarget(eqt):
    """
    Wraps to GetMsCellPowerTarget 3G function
    :raise TestEquipmentException: call to GetMsCellPowerTarget
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - integer: the mobile station expected power level
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get mobile station expected power level")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    level = ctypes.c_int()
    err = dll.GetMsCellPowerTarget3G(
        handle,
        ctypes.byref(level),
        ctypes.byref(err_msg))
    return err, level.value, err_msg.value


def SetAutoRRCTransitionControlState(eqt, state):
    """
    Wraps to SetAutoRRCTransitionControlState 3G function
    :raise TestEquipmentException: call to SetAutoRRCTransitionControlState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the desired state. Possible values:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Turn automatic RRC transition control %s",
        state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAutoRRCTransitionControlState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetRrcTransition(eqt, transition):
    """
    Wraps to SetRrcTransition 3G function
    :raise TestEquipmentException: call to SetRrcTransition
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type transition: str
    :param transition: the transition to set. Possible values:
        - "DCH"
        - "FACH"
        - "IDLE"
        - "PCH"
        - "UPCH"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set RRC transition to %s", transition)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetRrcTransition3G(
        handle,
        ctypes.c_char_p(transition),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCategoryControlMode(eqt, mode):
    """
    Wraps to SetCategoryControlMode 3G function
    :raise TestEquipmentException: call to SetCategoryControlMode
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the desired mode. Possible values:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Turn UE HS-DSCH category control %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCategoryControlMode3G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMacDPduSize(eqt, size):
    """
    Wraps to SetMacDPduSize 3G function
    :raise TestEquipmentException: call to SetMacDPduSize
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type size: str
    :param size: the desired mode. Possible values:
        - "BITS336"
        - "BITS656"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set PS data user defined active HS-PDSCHs to %s",
        size)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMacDPduSize3G(
        handle,
        ctypes.c_char_p(size),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetHsdpaReportedCategory(eqt):
    """
    Wraps to GetHsdpaReportedCategory 3G function
    :raise TestEquipmentException: call to GetHsdpaReportedCategory
    driver function failed or HSDPA reported isn't supported
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - integer: the HSDPA reported category, a value from 1 to 64. -1 if the
        equipment cannot report the value (default returned value).
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get UE reported HS-DSCH category")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    category = ctypes.c_int()
    err = dll.GetHsdpaReportedCategory3G(
        handle,
        ctypes.byref(category),
        ctypes.byref(err_msg))
    return err, category.value, err_msg.value


def GetHsupaReportedCategory(eqt):
    """
    Wraps to GetHsupaReportedCategory 3G function
    :raise TestEquipmentException: call to GetHsupaReportedCategory
    driver function failed or HSDPA reported isn't supported
    or cannot be reported ("NREP" or "NSUP" returned)
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - integer: the HSUPA reported category, a value from 1 to 6, -1 if the
        equipment cannot report the value (default), -2 if the functionality
        isn't supported
        - str: error message of the driver function
    """
    eqt.get_logger().info("Get UE reported E-DCH category")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    category = ctypes.c_int()
    err = dll.GetHsupaReportedCategory3G(
        handle,
        ctypes.byref(category),
        ctypes.byref(err_msg))
    return err, category.value, err_msg.value


def SetHsupaTti(eqt, tti):
    """
    Wraps to SetHsupaTti 3G function
    :raise TestEquipmentException: call to SetHsupaTti
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type tti: integer
    :param tti: the TTI to set. Possible values:
        - 2  :  2ms
        - 10 : 10ms => default value
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set HSUPA TTI to %d ms", tti)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHsupaTti3G(handle, ctypes.c_int(tti), ctypes.byref(err_msg))
    return err, err_msg.value


def SetPsDataConfigurationType(eqt, cfg_type):
    """
    Wraps to SetPsDataConfigurationType 3G function
    :raise TestEquipmentException: call to SetPsDataConfigurationType
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type cfg_type: str
    :param cfg_type: the PS data configuration type to set. Possible values:
        - "FIXED"
        - "REPORTED"
        - "USER_DEFINED"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set PS data configuration type to %s",
        cfg_type)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPsDataConfigurationType3G(
        handle,
        ctypes.c_char_p(cfg_type),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCqiValue(eqt, cqi):
    """
    Wraps to SetCqiValue 3G function
    :raise TestEquipmentException: call to SetCqiValue
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type cqi: integer
    :param cqi: the CQI value to set, an integer from 5 to 30.
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set CQI to %d", cqi)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCqiValue3G(handle, ctypes.c_int(cqi), ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnCpichLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnCpichLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnCpichLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected CPICH level to %f",
        offset)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    eqt.get_logger().info("Set HSPA cell 1 connected CPICH state ON")
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnCpichLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnCcpcLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnCcpcLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnCcpcLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected P-CCPCH/SCH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected P-CCPCH/SCH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnCcpcLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell2ConnCcpcLevel(eqt, offset):
    """
    Wraps to SetHspaCell2ConnCcpcLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell2ConnCcpcLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected CCPCH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected CCPCH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell2ConnCcpcLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnPichLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnPichLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnPichLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected PICH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected PICH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnPichLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnDpchLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnDpchLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnDpchLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected PICH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected PICH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnDpchLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnEagcLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnEagcLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnEagcLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected E-AGCH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected E-AGCH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnEagcLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnEhicLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnEhicLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnEhicLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected E-HICH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected E-HICH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnEhicLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnErgcLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnErgcLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnErgcLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected E-RGCH level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected E-RGCH state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnErgcLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHspaCell1ConnHspdLevel(eqt, offset):
    """
    Wraps to SetHspaCell1ConnHspdLevel 3G function
    :raise TestEquipmentException: call to SetHspaCell1ConnHspdLevel
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type offset: integer
    :param offset: the power offset to set (-20.0dB to 0.0dB :
    resolution 0.01)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSPA cell 1 connected HS-PDSCHs level to %f",
        offset)
    eqt.get_logger().info("Set HSPA cell 1 connected HS-PDSCHs state ON")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHspaCell1ConnHspdLevel3G(
        handle,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetGprsRadioAccessBearer(eqt, ul_rab, dl_rab):
    """
    Wraps to SetGprsRadioAccessBearer driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ul_rab: str
    :param ul_rab: uplink RAB. Possible values:
        - "64k"
        - "128k"
        - "384k"
        - "HSUPA"
    :type dl_rab: str
    :param dl_rab: uplink RAB. Possible values:
        - "64k"
        - "384k"
        - "HSDPA"
    :raise TestEquipmentException: failed to call SetGprsRadioAccessBearer
    driver function or the couple (ul_rab, dl_rab) is an invalid one.
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set radio access bearer uplink to %s and downlink to %s",
        ul_rab,
        dl_rab)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetGprsRadioAccessBearer3G(
        handle,
        ctypes.c_char_p(ul_rab),
        ctypes.c_char_p(dl_rab),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetAutomaticRrcState(eqt, state):
    """
    Wraps to SetAutomaticRrcState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: automatic RRC state transition control state:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set %s automatic RRC state transition control",
        str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetAutomaticRrcState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetAutomaticDlDataState(eqt, state):
    """
    Wraps to SetAutomaticDlDataState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: automatic transition on DL IP data state:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set %s automatic transition on DL IP data",
        str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetAutomaticDlDataState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellDchInactivityTimer(eqt, time):
    """
    Wraps to SetCellDchInactivityTimer3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type time: double
    :param time: inactivity timer length in seconds, [0.1 ; 1800], step 0.1.
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set cell DCH inactivity timer to %s second(s)",
        str(time))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCellDchInactivityTimer3G(
        handle,
        ctypes.c_double(time),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellDchTransitionState(eqt, state):
    """
    Wraps to SetCellDchTransitionState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: CELL_DCH inactivity timer destination RRC state:
        - "IDLE"
        - "FACH"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set cell DCH inactivity timer destination RRC state to %s",
        str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCellDchTransitionState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellFachInactivityTimer(eqt, time):
    """
    Wraps to SetCellFachInactivityTimer3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type time: double
    :param time: inactivity timer length in seconds, [0.1 ; 1800], step 0.1.
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set cell FACH inactivity timer to %s second(s)",
        str(time))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCellFachInactivityTimer3G(
        handle,
        ctypes.c_double(time),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCellFachTransitionState(eqt, state):
    """
    Wraps to SetCellFachTransitionState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: CELL_FACH inactivity timer destination RRC state:
        - "DCH"
        - "PCH"
        - "UPCH"
        - "IDLE"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set cell FACH inactivity timer destination RRC state to %s",
        str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCellFachTransitionState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCpcState(eqt, state):
    """
    Wraps to SetCpcState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: CPC state:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set %s CPC", str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCpcState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCcpchChannelizationCode(eqt, code):
    """
    Wraps to SetCcpchChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 63)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set secondary CCPC's channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetCcpchChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPichChannelizationCode(eqt, code):
    """
    Wraps to SetPichChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (2 to 255)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set page indicator channel's channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetPichChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetAichChannelisationCode(eqt, code):
    """
    Wraps to SetAichChannelisationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (2 to 255)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set acquisition indicator channel's channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetAichChannelisationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDpch15kspsChannelizationCode(eqt, code):
    """
    Wraps to SetDpch15kspsChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (2 to 255)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set channelization code for the downlink DPCH (15 ksps) to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetDpch15kspsChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDpch30kspsChannelizationCode(eqt, code):
    """
    Wraps to SetDpch30kspsChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 127)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set channelization code for the downlink DPCH (30 ksps) to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetDpch30kspsChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDpchHsdpa15kspsChannelizationCode(eqt, code):
    """
    Wraps to SetDpchHsdpa15kspsChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (2 to 255)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSDPA/HSPA DPCH 15 ksps (OVSF 256) channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetDpchHsdpa15kspsChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDpchHsdpa30kspsChannelizationCode(eqt, code):
    """
    Wraps to SetDpchHsdpa30kspsChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 127)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSDPA/HSPA DPCH 30 ksps (OVSF 128) channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetDpchHsdpa30kspsChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetEagchChannelizationCode(eqt, code):
    """
    Wraps to SetEagchChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param state: channelization code to set (2 to 255)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set E-AGCH channelization code to %s", str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetEagchChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetEhichChannelizationCode(eqt, code):
    """
    Wraps to SetEhichChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 127)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Sets E-HICH/E-RGCH channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetEhichChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsdpaPsDataFirstHsPdschChannelizationCode(eqt, code):
    """
    Wraps to SetHsdpaPsDataFirstHsPdschChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 11)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set PS data first HS-PDSCH channelization code to %s", str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsdpaPsDataFirstHsPdschChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsdpaRbTestFirstHsPdschChannelizationCode(eqt, code):
    """
    Wraps to SetHsdpaRbTestFirstHsPdschChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 11)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set RB test mode first HS-PDSCH channelization code to %s",
        str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsdpaRbTestFirstHsPdschChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsScch2ChannelizationCode(eqt, code):
    """
    Wraps to SetHsScch2ChannelizationCode3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type code: integer
    :param code: channelization code to set (1 to 127)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Sets HS-SCCH 2 channelization code to %s", str(code))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsScch2ChannelizationCode3G(
        handle,
        ctypes.c_int(code),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsScch3ConfigState(eqt, state):
    """
    Wraps to SetHsScch3ConfigState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: FDD test mode HSDPA/HSPA HS-SCCH 3 channel configuration state:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set %s FDD test mode HSDPA/HSPA HS-SCCH 3 channel configuration",
        str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsScch3ConfigState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsScch4ConfigState(eqt, state):
    """
    Wraps to SetHsScch4ConfigState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: FDD test mode HSDPA/HSPA HS-SCCH 4 channel configuration state:
        - "ON"
        - "OFF"
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set %s FDD test mode HSDPA/HSPA HS-SCCH 4 channel configuration",
        str(state))
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsScch4ConfigState3G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsdpaOcnsConfigState(eqt, states):
    """
    Wraps to SetHsdpaOcnsConfigState3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type states: array of str
    :param states: each HSDPA/HSPA OCNS 1 to 6 channel
    configuration state is "ON" or "OFF" word
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Sets HSDPA/HSPA OCNS 1 to 6 channel configuration states")
    c_states = (ctypes.c_char_p * len(states))()
    for idx in range(len(states)):
        eqt.get_logger().debug("\tChannel %s: %s", str(idx + 1), str(states[idx]))
        c_states[idx] = states[idx]
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsdpaOcnsConfigState3G(
        handle,
        c_states,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsdpaOcnsChannelizationCodes(eqt, codes):
    """
    Wraps to SetHsdpaOcnsChannelizationCodes3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type codes: array of integers
    :param codes: each HSDPA orthogonal channel noise simulator
    channelization code is an integer from 1 to 127
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set channelization codes of HSDPA OCNS 1 to 6")
    c_codes = (ctypes.c_int * len(codes))()
    for idx in range(len(codes)):
        eqt.get_logger().debug("\tOCNS %s: %s", str(idx + 1), str(codes[idx]))
        c_codes[idx] = codes[idx]
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsdpaOcnsChannelizationCodes3G(
        handle,
        c_codes,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsdpaCell1ConnectedPCcpchSchLevel(eqt, level):
    """
    Wraps to SetHsdpaCell1ConnectedPCcpchSchLevel3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type level: double
    :param level: HSDPA cell 1 connected P-CCPCH/SCH level (-20.00 dB to 0.00 dB)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSDPA cell 1 connected P-CCPCH/SCH level to %s",
        str(level))
    eqt.get_logger().info("Set HSDPA cell 1 connected P-CCPCH/SCH ON")
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsdpaCell1ConnectedPCcpchSchLevel3G(
        handle,
        ctypes.c_double(level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetHsdpaCell1ConnectedPichLevel(eqt, level):
    """
    Wraps to SetHsdpaCell1ConnectedPichLevel3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type level: double
    :param level: HSDPA cell 1 connected PICH level (-20.00 dB to 0.00 dB)
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set HSDPA cell 1 connected PICH level to %s",
        str(level))
    eqt.get_logger().info("Set HSDPA cell 1 connected PICH ON")
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetHsdpaCell1ConnectedPichLevel3G(
        handle,
        ctypes.c_double(level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ClearDataCounters(eqt):
    """
    Wraps ClearDataCounters 3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Clear the data counters")

    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.ClearDataCounters3G(
        handle,
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDataCounters(eqt):
    """
    Wraps GetDataCounters 3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - list: retrieved data counts in the following order:
            - Downlink packets
            - Downlink bytes
            - Uplink packets
            - Uplink bytes
        - str: error message of the driver function
    """
    eqt.get_logger().debug(
        "Retrieve the number of IP packets and byte on both uplink and downlink")

    err_msg = ctypes.c_char_p('\x00' * 1024)
    data_str = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.GetDataCounters3G(
        handle,
        1024,
        data_str,
        ctypes.byref(err_msg))
    returned_list = str(data_str.value).split(",")
    for index in range(len(returned_list)):
        returned_list[index] = float(returned_list[index])

    return err, returned_list, err_msg.value


def GetOverTheAirTxRate(eqt):
    """
    Wraps GetOverTheAirTxRate3G 3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - list: retrieved data counts in the following order:
            - Average data throughput (bps)
            - Current data throughput (bps)
            - Peak data throughput (bps)
            - Total data transfer (bytes)
        - str: error message of the driver function
    """
    eqt.get_logger().debug("Get over the air Tx rates")
    results_size = 1024
    err_msg = ctypes.c_char_p('\x00' * 1024)
    results = ctypes.c_char_p('\x00' * results_size)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.GetOverTheAirTxRate3G(
        handle,
        results_size,
        results,
        ctypes.byref(err_msg))
    returned_list = str(results.value).split(",")
    for index in range(len(returned_list)):
        returned_list[index] = float(returned_list[index])

    return err, returned_list, err_msg.value


def GetOverTheAirRxRate(eqt):
    """
    Wraps GetOverTheAirRxRate3G 3G driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - list: retrieved data counts in the following order:
            - Average data throughput (bps)
            - Current data throughput (bps)
            - Peak data throughput (bps)
            - Total data transfer (bytes)
        - str: error message of the driver function
    """
    eqt.get_logger().debug("Get over the air Rx rates")
    results_size = 1024
    err_msg = ctypes.c_char_p('\x00' * 1024)
    results = ctypes.c_char_p('\x00' * results_size)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.GetOverTheAirRxRate3G(
        handle,
        results_size,
        results,
        ctypes.byref(err_msg))
    returned_list = str(results.value).split(",")
    for index in range(len(returned_list)):
        returned_list[index] = float(returned_list[index])

    return err, returned_list, err_msg.value


def GetGprsRadioAccessBearer(eqt):
    """
    Gets GPRS Radio Access Bearer value.
    :rtype: str
    :return: the GPRS Radio Access Bearer. Possible returned values:
        - "GPRSRAB1" Default returned value
        - "GPRSRAB2"
        - "GPRSRAB3"
        - "GPRSRAB4"
        - "GPRSRAB5"
        - "PSDH64"
        - "PSDH128"
        - "PSDH384"
        - "PHSP"
    """
    eqt.get_logger().info("Get GPRS Radio Access Bearer")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    rab = ctypes.c_char_p('\x00' * 32)
    err = dll.GetGprsRadioAccessBearer3G(
        handle,
        32,
        rab,
        ctypes.byref(err_msg))
    return err, rab.value, err_msg.value


def GetRbtestChannelType(eqt):
    """
    Gets RBTEST channel type value.
    :rtype: str
    :return: the RBTEST channel type. Possible returned values:
        - HSDP12 (12.2k RMC + HSDPA)
        - HSP12 (12.2k RMC + HSPA)
        - HSPA
        - RMC12 (12.2k RMC)
        - RMC64 (64k RMC)
        - RMC144 (144k RMC)
        - RMC384 (384k RMC)
        - RMC33NC (33k No Coding RMC)
        - RMCAM1264 (12.2k UL/64k DL AM RMC)
        - RMCAM12144 (12.2k UL/144k DL AM RMC)
        - RMCAM12384 (12.2k UL/384k DL AM RMC)
        - RMCAM64384 (64k UL/384k DL AM RMC)
    """
    eqt.get_logger().info("Get the RBTEST channel type")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    rbtest_ct = ctypes.c_char_p('\x00' * 32)
    err = dll.GetRbtestChannelType3G(
        handle,
        32,
        rbtest_ct,
        ctypes.byref(err_msg))
    return err, rbtest_ct.value, err_msg.value
