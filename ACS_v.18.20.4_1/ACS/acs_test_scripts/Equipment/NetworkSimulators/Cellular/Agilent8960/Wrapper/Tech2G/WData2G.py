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
:summary: wrapper for data 2G function of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def SetDutIpAddress(eqt, ip_num, ip_addr):
    """
    Wraps to SetDutIpAddress 2G function
    :raise TestEquipmentException: call to SetDutIpAddress
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_num: integer
    :param ip_num: number of the IP address to set (1 to 4)
    :type ip_addr: str
    :param ip_addr: the IP address to set
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set device under test IP address %d to %s",
        ip_num,
        ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutIpAddress2G(
        handle,
        ctypes.c_int(ip_num),
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutIpAddress(eqt, ip_num):
    """
    Wraps to GetDutIpAddress 2G function
    :raise TestEquipmentException: call to GetDutIpAddress
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_num: integer
    :param ip_num: the number of the IP address to return (1 to 4)
    :rtype: str
    :return: the IP address of the device under test
    """
    eqt.get_logger().info("Get device under test IP address %d", ip_num)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDutIpAddress2G(
        handle,
        32,
        ip_num,
        ip_addr,
        ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def SetDutPrimaryDns(eqt, ip_addr):
    """
    Wraps to SetDutPrimaryDns 2G function
    :raise TestEquipmentException: call to SetDutPrimaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: the IP address to set
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set device under test primary DNS IP address to %s",
        ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutPrimaryDns2G(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutPrimaryDns(eqt):
    """
    Wraps to GetDutPrimaryDns 2G function
    :raise TestEquipmentException: call to GetDutPrimaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the IP address of the device under test primary DNS
    """
    eqt.get_logger().info("Get device under test primary DNS IP address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDutPrimaryDns2G(handle, 32, ip_addr, ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def SetDutSecondaryDns(eqt, ip_addr):
    """
    Wraps to SetDutSecondaryDns 2G function
    :raise TestEquipmentException: call to SetDutSecondaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: the IP address to set
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set device under test secondary DNS IP address to %s",
        ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutSecondaryDns2G(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutSecondaryDns(eqt):
    """
    Wraps to GetDutSecondaryDns 2G function
    :raise TestEquipmentException: call to GetDutSecondaryDns
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the IP address of the device under test secondary DNS
    """
    eqt.get_logger().info("Get device under test secondary DNS IP address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDutSecondaryDns2G(handle, 32, ip_addr, ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def GetDataConnectionStatus(eqt):
    """
    Wraps to GetDataConnectionStatus 2G function
    :raise TestEquipmentException: call to GetDataConnectionStatus
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the data connection status. Possible returned values:
        - "IDLE" => Default returned value
        - "ATTACHING"
        - "DETACHING"
        - "ATTACHED"
        - "STARTING"
        - "ENDING"
        - "TRANSFERRING"
        - "PDP_ACTIVATING"
        - "PDP_ACTIVE"
        - "PDP_DEACTIVATING"
        - "CS_DATA_CON_ACTIVE"
        - "SUSPENDED"
    """
    eqt.get_logger().debug("Get data connection status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    state = ctypes.c_char_p('\x00' * 32)
    err = dll.GetDataConnectionStatus2G(
        handle,
        32,
        state,
        ctypes.byref(err_msg))
    return err, state.value, err_msg.value


def SetInitialPsDataRRCState(eqt, state):
    """
    Wraps to SetInitialPsDataRRCState 2G function
    :raise TestEquipmentException: call to SetInitialPsDataRRCState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the desired state. Possible values:
        - "DCH" => factory value
        - "FACH"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set initial PS data RRC state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetInitialPsDataRRCState2G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetConnectionType(eqt, conn_type):
    """
    Wraps to SetConnectionType 2G function
    :raise TestEquipmentException: call to SetConnectionType
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type conn_type: str
    :param conn_type: the connection type to set.
    Possible values:
        - "A"
        - "B"
        - "ACKB"
        - "BLER"
        - "AUTO" => default value
        - "SRBL"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set data connection type to %s", conn_type)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetConnectionType2G(
        handle,
        ctypes.c_char_p(conn_type),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetDtmMultislotConfig(eqt, config):
    """
    Wraps to SetDtmMultislotConfig 2G function
    :raise TestEquipmentException: call to SetDtmMultislotConfig
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type config: str
    :param config: the connection type to set.
    Possible values:
        - "D1U1" | "D1U2" | "D1U3" | "D1U4" | "D1U5" | "D1U6"
        - "D2U1" | "D2U2" | "D2U3" | "D2U4" | "D2U5"
        - "D3U1" | "D3U2" | "D3U3" | "D3U4"
        - "D4U1" | "D4U2" | "D4U3"
        - "D5U1" | "D5U2"
        - "D6U1"
        - "CUST"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set DTM multislot configuration to %s", config)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDtmMultislotConfig2G(
        handle,
        ctypes.c_char_p(config),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPdtchModulationCodingScheme(eqt, downlink, uplink):
    """
    Wraps to SetPdtchModulationCodingScheme 2G function
    :raise TestEquipmentException: call to SetPdtchModulationCodingScheme
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type downlink: str
    :param downlink: the downlink coding scheme to set.
    Possible values:
        - "MCS1" | "MCS2" | "MCS3" | "MCS4" | "MCS5" | "MCS6" |
          "MCS7" | "MCS8" | "MCS9"
    :type uplink: str
    :param uplink: the uplink coding scheme to set.
    Possible values:
        - "MCS1" | "MCS2" | "MCS3" | "MCS4" | "MCS5" | "MCS6" |
          "MCS7" | "MCS8" | "MCS9"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info(
        "Set downlink coding scheme to %s and uplink coding scheme to %s",
        downlink,
        uplink)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPdtchModulationCodingScheme2G(
        handle,
        ctypes.c_char_p(downlink),
        ctypes.c_char_p(uplink),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPdtchPuncturingScheme(eqt, scheme):
    """
    Wraps to SetPdtchPuncturingScheme 2G function
    :raise TestEquipmentException: call to SetPdtchPuncturingScheme
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type scheme: str
    :param scheme: the PDTCH puncturing scheme to set.
    Possible values:
        - "PS1"
        - "PS2"
        - "PS3"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set PDTCH puncturing scheme to %s", scheme)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPdtchPuncturingScheme2G(
        handle,
        ctypes.c_char_p(scheme),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPdtchUplinkCodingScheme(eqt, scheme):
    """
    Wraps to SetPdtchUplinkCodingScheme 2G function
    :raise TestEquipmentException: call to SetPdtchUplinkCodingScheme
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type scheme: str
    :param scheme: the PDTCH uplink coding scheme to set.
    Possible values:
        - "CS1"
        - "CS2"
        - "CS3"
        - "CS4"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set PDTCH uplink coding scheme to %s", scheme)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPdtchUplinkCodingScheme2G(
        handle,
        ctypes.c_char_p(scheme),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetPdtchDownlinkCodingScheme(eqt, scheme):
    """
    Wraps to SetPdtchDownlinkCodingScheme 2G function
    :raise TestEquipmentException: call to SetPdtchDownlinkCodingScheme
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type scheme: str
    :param scheme: the PDTCH downlink coding scheme to set.
    Possible values:
        - "CS1"
        - "CS2"
        - "CS3"
        - "CS4"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set PDTCH downlink coding scheme to %s", scheme)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPdtchDownlinkCodingScheme2G(
        handle,
        ctypes.c_char_p(scheme),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetMultislotConfig(eqt, config):
    """
    Wraps to SetMultislotConfig 2G function
    :raise TestEquipmentException: call to SetMultislotConfig
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type config: str
    :param config: the multislot configuration to set.
    Possible values:
        - "D1U1" | "D1U2" | "D1U3" | "D1U4" | "D1U5" | "D1U6"
        - "D2U1" | "D2U2" | "D2U3" | "D2U4" | "D2U5"
        - "D3U1" | "D3U2" | "D3U3" | "D3U4"
        - "D4U1" | "D4U2" | "D4U3"
        - "D5U1" | "D5U2"
        - "D6U1"
        - "CUST"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set multislot configuration to %s", config)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMultislotConfig2G(
        handle,
        ctypes.c_char_p(config),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCustomMultislotConfig(eqt, main_timeslot, dl_slot_enabled,
                             dl_slot_level, ul_slot_enabled, ul_slot_gamma):
    """
    Wraps to SetCustomMultislotConfig 2G function
    :raise TestEquipmentException: call to SetMultislotConfig
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :param mainTimeslot  : The number of the main time slot (an integer from 0 to 7).
    :param dlSlotEnabled : A str of 8 "ON" | "OFF" words separated by ','.
    :param dlSlotLevel   : A str of 8 levels in dB (a double from -127.0 to +127.0) separated by ','.
    :param ulSlotEnabled : A str of 8 "ON" | "OFF" words separated by ','.
    :param ulSlotGamma   : A str of 8 gamma power control (an integer from 0 to 31) separated by ','.
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set custom multislot configuration")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCustomMultislotConfig2G(
        handle,
        ctypes.c_int(main_timeslot),
        ctypes.c_char_p(dl_slot_enabled),
        ctypes.c_char_p(dl_slot_level),
        ctypes.c_char_p(ul_slot_enabled),
        ctypes.c_char_p(ul_slot_gamma),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetServingCell(eqt):
    """
    Wraps to 2G GetServingCell function
    :raise TestEquipmentException: failed to call GetServingCell
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the serving cell
    """
    eqt.get_logger().info("Get cell service")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    serving_cell = ctypes.c_char_p('\x00' * 32)
    err = dll.GetServingCell2G(handle, 32, serving_cell, ctypes.byref(err_msg))
    return err, serving_cell.value, err_msg.value
