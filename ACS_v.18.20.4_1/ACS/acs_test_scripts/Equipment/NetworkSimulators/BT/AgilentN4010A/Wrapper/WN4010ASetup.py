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
:summary: wrapper setup function for bluetooth network simulator Agilent N4010A
:since: 23/03/2011
:author: ymorel
"""

import ctypes


def SetOperatingMode(eqt, mode):
    """
    Wraps to SetOperatingMode function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the connection type to use. Possible values:
        - "LINK"
        - "RFA"
        - "RFG"
    :raise TestEquipmentException: call to SetOperatingMode driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set operating mode to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetOperatingMode(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetLinkType(eqt, link_type):
    """
    Wraps to SetLinkType function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type link_type: str
    :param link_type: the connection type to use:
        - "ACL"
        - "SCO"
        - "TEST"
    :raise TestEquipmentException: call to SetLinkType driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set link type to %s", link_type)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLinkType(
        handle,
        ctypes.c_char_p(link_type),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetTestModeLinkType(eqt, mode):
    """
    Wraps to SetTestModeLinkType function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the test mode link to set:
        - "LOOP"
        - "TRAN"
    :raise TestEquipmentException: call to SetTestModeLinkType driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set test mode link type to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTestModeLinkType(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SelectLinkProfile(eqt, profile):
    """
    Wraps to SelectLinkProfile function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type profile: str
    :param profile: the head set profile to set:
        - "NONE"
        - "HSP": head set profile
    :raise TestEquipmentException: call to SelectLinkProfile driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Select %s link profile", profile)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SelectLinkProfile(
        handle,
        ctypes.c_char_p(profile),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetLinkProfileState(eqt, state):
    """
    Wraps to SetLinkProfileState function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param profile: the head set profile state to set:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: call to SetLinkProfileState driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set link profile state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLinkProfileState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetLossCompensationState(eqt, state):
    """
    Wraps to SetLossCompensationState function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param profile: the loss compensation state to set:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: call to SetLossCompensationState driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set loss compensation state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLossCompensationState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetFixedLossCompensation(eqt, loss_comp):
    """
    Wraps to SetFixedLossCompensation function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type loss_comp: double
    :param loss_comp: loss compensation is in dB. Range -50dB to +40dB.
    :raise TestEquipmentException: call to SetFixedLossCompensation driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set fixed loss compensation to %f", loss_comp)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetFixedLossCompensation(
        handle,
        ctypes.c_double(loss_comp),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetEUTPowerClass(eqt, power_class):
    """
    Wraps to SetEUTPowerClass function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type power_class: str
    :param power_class: the power class to set:
        - "PC1"
        - "PC2"
        - "PC3"
    :raise TestEquipmentException: call to SetEUTPowerClass driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set EUT power class to %s", power_class)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetEUTPowerClass(
        handle,
        ctypes.c_char_p(power_class),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ResetEUTBdAddress(eqt):
    """
    Wraps to ResetEUTBdAddress function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to ResetEUTBdAddress driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Reinitialize bluetooth device addresses")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ResetEUTBdAddress(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetLinkInquiryDuration(eqt, duration):
    """
    Wraps to SetLinkInquiryDuration function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type duration: double
    :param duration: duration in seconds: range 1.28 to 61.44. Value is
    rounded to the nearest multiple of 1.28 (example: giving 2.3s sets
    2.56s).
    :raise TestEquipmentException: call to SetLinkInquiryDuration driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set link inquiry timeout to %f", duration)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLinkInquiryDuration(
        handle,
        ctypes.c_double(duration),
        ctypes.byref(err_msg))
    return err, err_msg.value


def InitiateInquiryProcedure(eqt):
    """
    Wraps to InitiateInquiryProcedure function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to InitiateInquiryProcedure driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Initiate inquiry procedure")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.InitiateInquiryProcedure(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def ListBDAddress(eqt):
    """
    Wraps to ListBDAddress function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to ListBDAddress driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - list: the addresses of the bluetooth devices that have responded to
    the inquiry procedure
        - str: log message
    """
    eqt.get_logger().info("List bluetooth device addresses")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    addresses = ctypes.c_char_p('\x00' * 1024)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ListBDAddress(handle, 1024, addresses, ctypes.byref(err_msg))
    return err, addresses.value, err_msg.value


def SetSTETxPowerLevel(eqt, power_level):
    """
    Wraps to SetSTETxPowerLevel function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type power_level: double
    :param power_level: the power level in dBm. Range -90.0 to 0.0,
    resolution 0.1.
    :raise TestEquipmentException: call to SetSTETxPowerLevel driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set transmit power level to %f", power_level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSTETxPowerLevel(
        handle,
        ctypes.c_double(power_level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSTERxPowerLevel(eqt, power_level):
    """
    Wraps to SetSTERxPowerLevel function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type power_level: double
    :param power_level: the power level in dBm. Range -70.0 to 25,
    resolution 5.
    :raise TestEquipmentException: call to SetSTERxPowerLevel driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set expected input power to %f", power_level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSTERxPowerLevel(
        handle,
        ctypes.c_double(power_level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSpecificTxPowerLevel(eqt, power_level):
    """
    Wraps to SetSpecificTxPowerLevel function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type power_level: double
    :param power_level: the power level in dBm. Range -95.0 to 0.0,
    resolution 0.1.
    :raise TestEquipmentException: call to SetSpecificTxPowerLevel driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info(
        "Set specific transmit power level to %f",
        power_level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSpecificTxPowerLevel(
        handle,
        ctypes.c_double(power_level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSpecificRxPowerLevel(eqt, power_level):
    """
    Wraps to SetSpecificRxPowerLevel function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type power_level: double
    :param power_level: the power level in dBm. Range -70 to 25,
    resolution 5.
    :raise TestEquipmentException: call to SetSpecificRxPowerLevel driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info(
        "Set specific expected input power level to %f",
        power_level)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSpecificRxPowerLevel(
        handle,
        ctypes.c_double(power_level),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ResetLinkConfigureParameters(eqt):
    """
    Wraps to ResetLinkConfigureParameters function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to ResetLinkConfigureParameters driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Reset link parameters")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ResetLinkConfigureParameters(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetFrequencyHoppingState(eqt, state):
    """
    Wraps to SetFrequencyHoppingState function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param profile: the frequency hopping state to set:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: call to SetFrequencyHoppingState driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set frequency hopping state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetFrequencyHoppingState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def ResetTestSequence(eqt):
    """
    Wraps to ResetTestSequence function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to ResetTestSequence driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Reset test sequence")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ResetTestSequence(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def AddTest(eqt, name):
    """
    Wraps to SetFrequencyHoppingState function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type name: str
    :param name: the test name to set. Possible values:
        - {"CFDR"; "ICFT"; "MCH"; "MIL"; "MSEN"; "OPOW"; "PCON"; "SSEN";
           "BFP"; "DPEN"; "EMIL"; "ESEN"; "FSM"; "GTIM"; "RPOW"}
    :raise TestEquipmentException: call to AddTest driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Add %s test to test sequence", name)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.AddTest(handle, ctypes.c_char_p(name), ctypes.byref(err_msg))
    return err, err_msg.value


def SelectTest(eqt, name, occurrence):
    """
    Wraps to SelectTest function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type name: str
    :param name: the test name to set. Possible values:
        - {"CFDR"; "ICFT"; "MCH"; "MIL"; "MSEN"; "OPOW"; "PCON"; "SSEN";
           "BFP"; "DPEN"; "EMIL"; "ESEN"; "FSM"; "GTIM"; "RPOW"}
    :type occurrence: integer
    :param occurrence: the occurrence of the test name to select
    :raise TestEquipmentException: call to SelectTest driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info(
        "Select occurrence %d of test %s",
        occurrence,
        name)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SelectTest(
        handle,
        ctypes.c_char_p(name),
        ctypes.c_int(occurrence), ctypes.byref(err_msg))
    return err, err_msg.value


def SetTestSequenceLoopMode(eqt, mode):
    """
    Wraps to SetTestSequenceLoopMode function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the test name to set:
        - "SING"
        - "CONT"
        - "FIX"
    :raise TestEquipmentException: call to SetTestSequenceLoopMode driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set test sequence loop mode to %s", mode)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTestSequenceLoopMode(
        handle,
        ctypes.c_char_p(mode),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetTestSequenceLoopNumber(eqt, nb_loop):
    """
    Wraps to SetTestSequenceLoopNumber function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type nb_loop: integer
    :param nb_loop: the number of test loops (1 to 99).
    :raise TestEquipmentException: call to SetTestSequenceLoopNumber driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set test sequence loop number to %d", nb_loop)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetTestSequenceLoopNumber(
        handle,
        ctypes.c_int(nb_loop),
        ctypes.byref(err_msg))
    return err, err_msg.value


def RunTestSequence(eqt):
    """
    Wraps to RunTestSequence function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to RunTestSequence driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Run test sequence")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.RunTestSequence(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def AutoDisconnectTestSet(eqt, state):
    """
    Wraps to AutoDisconnectTestSet function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param profile: the auto disconnect state to set:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: call to AutoDisconnectTestSet driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set auto disconnect state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.AutoDisconnectTestSet(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def DisconnectTestSet(eqt):
    """
    Wraps to DisconnectTestSet function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :raise TestEquipmentException: call to DisconnectTestSet driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Disconnect test set")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.DisconnectTestSet(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetImpairmentState(eqt, state):
    """
    Wraps to SetImpairmentState function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param profile: the impairment state to set:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: call to SetImpairmentState driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set impairment state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetImpairmentState(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetBitsNumber(eqt, nb_bits):
    """
    Wraps to SetBitsNumber function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type nb_bits: unsigned long
    :param nb_bits: the number of returned payload bits to use for the test
    :raise TestEquipmentException: call to SetBitsNumber driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Set bits number to %s", nb_bits)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetBitsNumber(
        handle,
        ctypes.c_ulong(nb_bits),
        ctypes.byref(err_msg))
    return err, err_msg.value
