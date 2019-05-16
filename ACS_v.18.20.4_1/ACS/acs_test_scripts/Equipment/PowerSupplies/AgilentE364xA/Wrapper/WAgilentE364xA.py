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
:summary: wrapper for Agilent E364xA power supply
:since: 09/03/2010
:author: ymorel
"""

import ctypes


def Connect(eqt, board_id, gpib_addr):
    """
    Wraps to Connect function
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    :type board_id: integer
    :param board_id: GPIB board ID
    :type gpib_addr: integer
    :param gpib_addr: GPIB address (between 0 and 15).
    :rtype: integer
    :return: handle of the connected equipment.
    """
    eqt.get_logger().info("Connection")
    dll = eqt.get_dll()
    handle = ctypes.c_ulong()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Connect(
        board_id,
        gpib_addr,
        ctypes.byref(handle),
        ctypes.byref(err_msg))
    return err, handle.value, err_msg.value


def Disconnect(eqt):
    """
    Wraps to Disconnect function
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    """
    eqt.get_logger().info("Disconnection")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Disconnect(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetMaxCurrent(eqt, maxi, port):
    """
    This function sets the maximum current allowed by the power supply
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    :param maxi: maximum current allowed
    :param port: port number on which the current level has to be set
    :rtype: none
    :raise: raises TestEquipmentException in case of failure
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetMaxCurrent(
        handle,
        ctypes.c_float(maxi),
        ctypes.c_int(port),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetVoltageLevel(eqt, voltage, port):
    """
    This function sets the current voltage of the power supply
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    :param voltage: current voltage
    :param port: port number on which the voltage level has to be set
    :rtype: none
    :raise: raises TestEquipmentException in case of failure
    """
    eqt.get_logger().info(
        "Set voltage to %s V on port %s",
        str(voltage),
        str(port))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetVoltageLevel(
        handle,
        ctypes.c_float(voltage),
        ctypes.c_int(port),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetCurrentMeasurement(eqt, port, meas_type, log=True):
    """
    Wraps to GetCurrentMeasurement function
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    :type port: integer
    :param port:
    :type meas_type: str
    :param meas_type: the type of measurement to do. Possible values:
        - "DC"    : dc current
        - "ACDC"  : ac+dc rms current
        - "HIGH"  : high current
        - "LOW"   : low current
        - "MAX"   : maximum current
        - "MIN"   : minimum current
    :type log : boolean
    :param log: if true, the log is enable

    :rtype: float
    :return: the result of the requested measurement
    """
    if log:
        eqt.get_logger().info("Get current measurement %s", str(meas_type))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    level = ctypes.c_float()
    err = dll.GetCurrentMeasurement(
        handle,
        ctypes.c_int(port),
        ctypes.c_char_p(meas_type),
        ctypes.byref(level),
        ctypes.byref(err_msg))
    return err, level.value, err_msg.value


def SwitchOn(eqt):
    """
    Turns on the power supply
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SwitchOn(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SwitchOff(eqt):
    """
    Turns off the power supply
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SwitchOff(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def PerformFullPreset(eqt):
    """
    Resets all power supply parameters to their default values
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.PerformFullPreset(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def GetEqID(eqt):
    """
    Gets the equipment identification str
    :type eqt: AgilentE364xA
    :param eqt: the equipment that uses the wrapper
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    eqt_id = ctypes.c_char_p('\x00' * 256)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.GetEqID(handle, 256, eqt_id, ctypes.byref(err_msg))
    return err, eqt_id.value, err_msg.value
