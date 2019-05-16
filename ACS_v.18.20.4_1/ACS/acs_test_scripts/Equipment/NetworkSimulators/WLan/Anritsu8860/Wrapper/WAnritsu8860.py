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
:summary: Wrapper for WLAN Anritsu 8860.
:author: ymorel
:since: 22/03/2011
"""

import ctypes


def Connect(eqt, con_type, board_id, gpib_address, ip_address):
    """
    Wraps to Connect driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type con_type: str
    :param con_type: Connection type. Possible values:
        - "GPIB"
        - "TCPIP"
    :type board_id: integer
    :param board_id: The GPIB board ID (if GPIB selected).
    :type gpib_address: integer
    :param gpib_address: The GPIB board ID (if GPIB selected).
    :type ip_address: str
    :param ip_address: The ip address of the equipment (if TCPIP selected).
    :rtype: integer
    :return: the error code of the driver function
    :raise TestEquipmentException: Unable to connect to Anritsu 8860 equipment
    """
    eqt.get_logger().info("Connection")
    dll = eqt.get_dll()
    handle = ctypes.c_ulong()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Connect(ctypes.byref(handle), ctypes.c_char_p(con_type), board_id,
                      gpib_address, ctypes.c_char_p(ip_address), ctypes.byref(err_msg))
    return err, handle.value, err_msg.value


def Disconnect(eqt):
    """
    Wraps to Disconnect function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: failed to disconnect equipment
    """
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Disconnect(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def PerformFullPreset(eqt):
    """
    Wraps to PerformFullPreset function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: failed to perform full preset
    """
    eqt.get_logger().info("Perform full preset")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.PerformFullPreset(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetMeasurementMode(eqt, mode):
    """
    Wraps to SetMeasurementMode function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode : Measurement mode. Possible values :
        - "TXMODE"
        - "RXMODE"
    :raise TestEquipmentException: Invalid measurement mode.
    """
    eqt.get_logger().info("Set measurement mode to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMeasurementMode(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetMeasurementMode(eqt):
    """
    Wraps to GetMeasurementMode function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: mode : Measurement mode. Possible values :
        - "TXMODE"
        - "RXMODE"
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get measurement mode")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    mode = ctypes.c_char_p('\x00' * 8)
    err = dll.GetMeasurementMode(handle, 8, mode, ctypes.byref(err_msg))
    return err, mode.value, err_msg.value


def AddLossEntry(eqt, channel, offset):
    """
    Wraps to AddLossEntry driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type channel: integer
    :param channel: The channel number
    :type channel: double
    :param channel: The offset to set in dBm
    :raise TestEquipmentException: failed to add loss entry
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Add loss entry item for channel %d: %d",
        channel,
        offset)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.AddLossEntry(
        handle,
        channel,
        ctypes.c_double(offset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetLossTableState(eqt, state):
    """
    Wraps the SetLossTableState driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: The loss table state. Possible values :
        - OFF (Disable the path loss table)
        - ON (Enable the path loss table (Default Value))
    """
    eqt.get_logger().info("Set loss table state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLossTableState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetLossTableState(eqt):
    """
    Wraps the GetLossTableState driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the loss table state. Possible values :
        - OFF (the loss table is disabled)
        - ON (the loss path table is enabled)
    """
    eqt.get_logger().info("Get loss table state")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    state = ctypes.c_char_p('\x00' * 4)
    err = dll.GetLossTableState(handle, 4, state, ctypes.byref(err_msg))
    return err, err_msg.value


def SetSignalState(eqt, state):
    """
    Wraps the SetSignalState driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the signal generator state. Possible values :
        - OFF (Disable the signal generator function)
        - ON (Enable the signal generator function (Default Value))
    """
    eqt.get_logger().info("Set signal generator state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSignalState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetSignalState(eqt):
    """
    Wraps the GetSignalState driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the signal generator state. Possible values :
        - OFF (Disable the signal generator function)
        - ON (Enable the signal generator function (Default Value))
    """
    eqt.get_logger().info("Get signal generator state")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    state = ctypes.c_char_p('\x00' * 4)
    err = dll.GetSignalState(handle, 4, state, ctypes.byref(err_msg))
    return err, state.value, err_msg.value


def SetWlanIpParams(eqt, ip_addr, subnet):
    """
    Wraps the SetWlanIpParams driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: WLAN IP Address.
    :type subnet: str
    :param subnet: WLAN Subnet Mask.
    """
    eqt.get_logger().info(
        "Set WLAN IP address to %s, Subnet to %s",
        ip_addr,
        subnet)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetWlanIpParams(
        handle,
        ctypes.c_char_p(ip_addr),
        ctypes.c_char_p(subnet),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetWlanIpAddress(eqt):
    """
    Wraps the GetWlanIpAddress driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: subnet: WLAN IP Address.
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get WLAN IP address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 32)
    err = dll.GetWlanIpAddress(handle, 32, ip_addr, ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def GetWlanIpSubnetMask(eqt):
    """
    Wraps the GetWlanIpSubnetMask driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: subnet: WLAN Subnet Mask.
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get WLAN IP subnet mask")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    subnet = ctypes.c_char_p('\x00' * 32)
    err = dll.GetWlanIpSubnetMask(handle, 32, subnet, ctypes.byref(err_msg))
    return err, subnet.value, err_msg.value


def SetTransmitPower(eqt, power):
    """
    Wraps the SetTransmitPower driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type power: double
    :param power: Power level at the test port connector.
    :raise TestEquipmentException: Failed to set transmit power.
    """
    eqt.get_logger().info("Set transmit power to %.3f", power)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTransmitPower(
        handle,
        ctypes.c_double(power),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetTransmitPower(eqt):
    """
    Wraps the GetTransmitPower driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type power: double
    :return: power: Power level at the test port connector.
    """
    eqt.get_logger().info("Get transmit power")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    power = ctypes.c_double()
    err = dll.GetTransmitPower(
        handle,
        ctypes.byref(power),
        ctypes.byref(err_msg))
    return err, power.value, err_msg.value


def SetWlanStandard(eqt, standard):
    """
    Wraps the SetWlanStandard driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type standard: str
    :param standard: WLAN Standard. Possible values :
        - "802.11a"
        - "802.11b"
        - "802.11g"
    :raise TestEquipmentException: Invalid Wlan standard.
    """
    eqt.get_logger().info("Set WLAN standard to %s", standard)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetWlanStandard(
        handle,
        ctypes.c_char_p(standard),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetWlanStandard(eqt):
    """
    Wraps the GetWlanStandard driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: standard: WLAN Standard. Possible values :
        - "802.11a"
        - "802.11b"
        - "802.11g"
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get WLAN standard")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    standard = ctypes.c_char_p('\x00' * 8)
    err = dll.GetWlanStandard(handle, 8, standard, ctypes.byref(err_msg))
    return err, standard.value, err_msg.value


def SetWlanChannel(eqt, channel):
    """
    Wraps the SetWlanChannel driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type channel: integer
    :param channel: WLAN channel to set (1 to 196).
    :raise TestEquipmentException: Invalid channel, must be 1 to 196.
    """
    eqt.get_logger().info("Set WLAN channel to %d", channel)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetWlanChannel(handle, channel, ctypes.byref(err_msg))
    return err, err_msg.value


def GetWlanChannel(eqt):
    """
    Wraps the GetWlanChannel driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: channel: current WLAN channel set (1 to 196).
    :raise TestEquipmentException: invalid channel, must be 1 to 196.
    """
    eqt.get_logger().info("Get WLAN channel")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    channel = ctypes.c_int()
    err = dll.GetWlanChannel(
        handle,
        ctypes.byref(channel),
        ctypes.byref(err_msg))
    return err, channel.value, err_msg.value


def SetWlanDataRate(eqt, rate):
    """
    Wraps the SetWlanDataRate driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type rate: str
    :param rate: WLAN data rate. Possible values :
        - {"1"; "2"; "5.5"; "6"; "9"; "11"; "12"; "18"; "24";
           "36"; "48"; "54"}
    :raise TestEquipmentException: Invalid data rate.
    """
    eqt.get_logger().info("Set WLAN data rate to %s", rate)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetWlanDataRate(
        handle,
        ctypes.c_char_p(rate),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetWlanDataRate(eqt):
    """
    Wraps the GetWlanDataRate driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: rate: WLAN data rate. Possible values :
        - {"1"; "2"; "5.5"; "6"; "9"; "11"; "12"; "18"; "24";
           "36"; "48"; "54"
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get WLAN data rate")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    rate = ctypes.c_char_p('\x00' * 8)
    err = dll.GetWlanDataRate(handle, 8, rate, ctypes.byref(err_msg))
    return err, rate.value, err_msg.value


def SetTestMode(eqt, mode):
    """
    Wraps the SetTestMode driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: equipment test mode. Possible values :
        - "DIRECT"
        - "NETWORK"
    :raise TestEquipmentException: invalid test mode.
    """
    eqt.get_logger().info("Set WLAN equipment test mode to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTestMode(handle, ctypes.c_char_p(mode), ctypes.byref(err_msg))
    return err, err_msg.value


def GetTestMode(eqt):
    """
    Wraps the GetTestMode driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: mode: equipment test mode. Possible values :
        - "DIRECT"
        - "NETWORK"
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get WLAN equipment test mode")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    mode = ctypes.c_char_p('\x00' * 8)
    err = dll.GetTestMode(handle, 8, mode, ctypes.byref(err_msg))
    return err, mode.value, err_msg.value


def SetNetworkType(eqt, nw_type):
    """
    Wraps the SetNetworkType driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type nw_type: str
    :param nw_type: network type. Possible values :
        - "ADHOC"
        - "ACCESS_POINT"
        - "STATION"
    :raise TestEquipmentException: invalid network type.
    """
    eqt.get_logger().info("Set network type to %s", nw_type)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetNetworkType(
        handle,
        ctypes.c_char_p(nw_type),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetNetworkType(eqt):
    """
    Wraps the GetNetworkType driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: nw_type: Network type. Possible values :
        - "ADHOC"
        - "ACCESS_POINT"
        - "STATION"
    :raise Insufficient memory allocation.
    """
    eqt.get_logger().info("Get network type")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    nw_type = ctypes.c_char_p('\x00' * 16)
    err = dll.GetNetworkType(handle, 16, nw_type, ctypes.byref(err_msg))
    return err, nw_type.value, err_msg.value


def CreateNetwork(eqt, ssid):
    """
    Wraps the CreateNetwork driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type ssid: str
    :param ssid: service set identity of the WLAN network (32 characters max).
    :raise TestEquipmentException: SSID has more than 32 characters.
    """
    eqt.get_logger().info("Create network %s", ssid)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.CreateNetwork(
        handle,
        ctypes.c_char_p(ssid),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetNetworkSsid(eqt):
    """
    Wraps the GetNetworkSsid driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: service set identity (SSID) of the Wlan network.
    """
    eqt.get_logger().info("Get network SSID")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ssid = ctypes.c_char_p('\x00' * 64)
    err = dll.GetNetworkSsid(handle, 64, ssid, ctypes.byref(err_msg))
    return err, ssid.value, err_msg.value


def SetDutPower(eqt, power):
    """
    Wraps the SetDutPower driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type power: integer
    :param power: DUT power level in dBm (between -30 and 30).
    """
    eqt.get_logger().info("Set DUT power %d", power)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutPower(handle, power, ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutPower(eqt):
    """
    Wraps the GetDutPower driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: power: DUT power level in dBm (between -30 and 30).
    """
    eqt.get_logger().info("Get DUT power")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    power = ctypes.c_int32()
    err = dll.GetDutPower(handle, ctypes.byref(power), ctypes.byref(err_msg))
    return err, power.value, err_msg.value


def SetMeasTriggerSource(eqt, source, trigger_edge, trigger_level):
    """
    Wraps the SetMeasTriggerSource driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type source: integer
    :param source: measurement trigger source. Possible values :
        - RF
        - FREE_RUN
        - EXT
        - VIDEO
    :raise TestEquipmentException: invalid measurement trigger source.
    """
    eqt.get_logger().info("Set measurement trigger source to %s", source)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMeasTriggerSource(
        handle,
        ctypes.c_char_p(source),
        ctypes.c_char_p(trigger_edge),
        ctypes.byref(ctypes.c_int32(trigger_level)),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetMeasTriggerSource(eqt):
    """
    Wraps the GetMeasTriggerSource driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: source: measurement trigger source. Possible values :
        - RF
        - FREE_RUN
        - EXT
        - VIDEO
    :raise TestEquipmentException: invalid measurement trigger source.
    """
    eqt.get_logger().info("Get measurement trigger source")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    source = ctypes.c_char_p('\x00' * 16)
    err = dll.GetMeasTriggerSource(
        handle,
        16,
        source,
        ctypes.byref(err_msg))
    return err, source.value, err_msg.value


def LaunchAutoSetup(eqt):
    """
    Wraps the LaunchAutoSetup driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    """
    eqt.get_logger().info("Launch auto setup")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.LaunchAutoSetup(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetIpPropertiesMode(eqt, mode):
    """
    Wraps the SetIpPropertiesMode driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: mode for IP properties. Possible values :
        - "AUTO"
        - "MANUAL"
    :raise TestEquipmentException: invalid IP properties mode.
    """
    eqt.get_logger().info("Set IP properties mode to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetIpPropertiesMode(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetIpPropertiesMode(eqt):
    """
    Wraps the GetIpPropertiesMode driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the mode for IP properties. Possible values :
        - "AUTO"
        - "MANUAL"
    """
    eqt.get_logger().info("Get IP properties mode")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    mode = ctypes.c_char_p('\x00' * 8)
    err = dll.GetIpPropertiesMode(handle, 8, mode, ctypes.byref(err_msg))
    return err, mode.value, err_msg.value


def SetDutIp(eqt, ip_addr):
    """
    Wraps the SetDutIp driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type ip_addr: str
    :param ip_addr: device under test IP V4 address.
    """
    eqt.get_logger().info("Set DUT ip to %s", ip_addr)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutIp(handle, ctypes.c_char_p(ip_addr), ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutIp(eqt):
    """
    Wraps the GetDutIp driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: device under test IP V4 address.
    """
    eqt.get_logger().info("Get DUT IP address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    ip_addr = ctypes.c_char_p('\x00' * 24)
    err = dll.GetDutIp(handle, 24, ip_addr, ctypes.byref(err_msg))
    return err, ip_addr.value, err_msg.value


def GetStationsNumber(eqt, nb_exp_stations, time):
    """
    Wraps the GetStationsNumber driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type nb_exp_stations: integer
    :param nb_exp_stations: this control sets the number
    of different MAC addresses expected to find before the search ends.
    :param time: integer
    :type time: this control sets the permissible time for the search to take place.
    :raise TestEquipmentException: invalid number of requested MAC addresses.
    :raise TestEquipmentException: invalid permissible time.
    """
    eqt.get_logger().info(
        "Get station number (%d expected stations during %s seconds)",
        nb_exp_stations, str(time))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    nb_stations = ctypes.c_long()
    err = dll.GetStationsNumber(
        handle,
        nb_exp_stations,
        time,
        ctypes.byref(nb_stations),
        ctypes.byref(err_msg))
    return err, nb_stations.value, err_msg.value


def GetStationMacAddress(eqt, nb_exp_stations, time,
                         station_number):
    """
    Wraps the GetStationMacAddress driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type nb_exp_stations: integer
    :param nb_exp_stations: this control sets the number
    of different MAC addresses expected to find before the search ends.
    :param time: integer
    :type time: this control sets the permissible time for the search to take place.
    :param station_number: integer
    :type station_number: station to find MAC address.
    :raise TestEquipmentException: invalid number of requested MAC addresses.
    :raise TestEquipmentException: invalid permissible time.
    """
    eqt.get_logger().info(
        "Get station %s (%d expected stations during %s seconds)",
        str(station_number), nb_exp_stations, str(time))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    mac_addresses = ctypes.c_char_p(('\x00' * 256))
    err = dll.GetStationMacAddress(
        handle,
        nb_exp_stations,
        time,
        station_number,
        mac_addresses,
        ctypes.byref(err_msg))
    return err, mac_addresses.value, err_msg.value


def SetDutMacAddress(eqt, mac):
    """
    Wraps the SetDutMacAddress driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type mac: str
    :param mac: the DUT MAC address to set.
    """
    eqt.get_logger().info("Set DUT MAC address %s", mac)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetDutMacAddress(
        handle,
        ctypes.c_char_p(mac),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetDutMacAddress(eqt):
    """
    Wraps the GetDutMacAddress driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: mac: The DUT MAC address to get.
    """
    eqt.get_logger().info("Get DUT MAC address")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    mac = ctypes.c_char_p('\x00' * 24)
    err = dll.GetDutMacAddress(handle, 24, mac, ctypes.byref(err_msg))
    return err, mac.value, err_msg.value


def GetDUTConnectionState(eqt, dut_mac_address):
    """
    Wraps the GetDUTConnectionState driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type dut_mac_address: str
    :param dut_mac_address: The DUT MAC address to test.
    :return: connected: The DUT connection state. Possible values :
        - "CONNECTED"
        - "DISCONNECTED"
    """
    eqt.get_logger().info("Check the DUT connection")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    connected = ctypes.c_char_p('\x00' * 24)
    err = dll.GetDUTConnectionState(
        handle,
        ctypes.c_char_p(dut_mac_address),
        connected,
        ctypes.byref(err_msg))
    return err, connected.value, err_msg.value


def GetPacketErrorRate(eqt):
    """
    Wraps the GetPacketErrorRate driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: measured packet error Rate.
    """
    eqt.get_logger().info("Get WLAN packet error rate")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    per = ctypes.c_double()
    err = dll.GetPacketErrorRate(
        handle,
        ctypes.byref(per),
        ctypes.byref(err_msg))
    eqt.get_logger().info("Measured packet error rate: %f", per.value)
    return err, per.value, err_msg.value


def GetPERPacketNumber(eqt):
    """
    Wraps the GetPERPacketNumber driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :return: packet_number: number of packets sent in Packet Error Rate Measurement.
    """
    eqt.get_logger().info("Get Wlan PER packet number to send")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    packet_number = ctypes.c_int()
    err = dll.GetPERPacketNumber(
        handle,
        ctypes.byref(packet_number),
        ctypes.byref(err_msg))
    return err, packet_number.value, err_msg.value


def SetPERPacketNumber(eqt, packet_number):
    """
    Wraps the SetPERPacketNumber driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :type packet_number: integer
    :param packet_number : number of packets to send for packet error
    rate measurement.
    """
    eqt.get_logger().info("Set Wlan PER packet number to %d", packet_number)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetPERPacketNumber(handle, packet_number, ctypes.byref(err_msg))
    return err, err_msg.value


def GetPowerAverage(eqt):
    """
    Wraps the GetPowerAverage driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: measured power average.
    """
    eqt.get_logger().info("Get WLAN power average")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    power_average = ctypes.c_double()
    err = dll.GetPowerAverage(
        handle,
        ctypes.byref(power_average),
        ctypes.byref(err_msg))
    return err, power_average.value, err_msg.value


def GetPowerPeak(eqt):
    """
    Wraps the GetPowerPeak driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: measured power peak.
    """
    eqt.get_logger().info("Get WLAN power peak")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    power_peak = ctypes.c_double()
    err = dll.GetPowerPeak(
        handle,
        ctypes.byref(power_peak),
        ctypes.byref(err_msg))
    return err, power_peak.value, err_msg.value


def GetEvmRmsPercent(eqt):
    """
    Wraps the GetEvmRmsPercent driver function
    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper
    :rtype: double
    :return: measured error vector magnitude.
    """
    eqt.get_logger().info("Get WLAN error vector magnitude percentage")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    evm_rms_percent = ctypes.c_double()
    err = dll.GetEvmRmsPercent(
        handle,
        ctypes.byref(evm_rms_percent),
        ctypes.byref(err_msg))
    return err, evm_rms_percent.value, err_msg.value


def SetBeaconInterval(eqt, beacon):
    """
    Wraps the SetBeaconInterval driver function which
    sets the approximate interval between beacons.

    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper

    :type beacon: int
    :param beacon: beacon interval in milliseconds, from 20 to 1000.

    :rtype: Tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set beacon interval to %s ms", (str(beacon)))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBeaconInterval(
        handle,
        ctypes.c_int(beacon),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetBeaconPreamble(eqt, preamble):
    """
    Wraps the SetBeaconPreamble driver function which
    sets the length of the preamble for the reference
    radio transmission.

    .. warning:: Only useable for 802.11b/g/a standard.

    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper

    :type preamble: str
    :param preamble: preamble to set.
        - "LONG"
        - "SHORT"

    :rtype: Tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info("Set beacon preamble to %s", (str(preamble)))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBeaconPreamble(
        handle,
        ctypes.c_char_p(preamble),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetOperationalRate(eqt, rate_set):
    """
    Wraps the SetOperationalRate driver function which
    defines how the operational rate set is broadcasted
    in the beacon packets.

    :type eqt: Anritsu8860
    :param eqt: the equipment that uses the wrapper

    :type rate_set: str
    :param rate_set: Operational rate to set. Allowed values:
        - ALL
        - SINGLE
        - MULTIPLE
        - USER

    :rtype: Tuple
    :return:
        - integer: error code of the driver function
        - str: error message of the driver function
    """
    eqt.get_logger().info(
        "Set beacon operational rate set to %s",
        (str(rate_set)))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetOperationalRate(
        handle,
        ctypes.c_char_p(rate_set),
        ctypes.byref(err_msg))
    return err, err_msg.value
