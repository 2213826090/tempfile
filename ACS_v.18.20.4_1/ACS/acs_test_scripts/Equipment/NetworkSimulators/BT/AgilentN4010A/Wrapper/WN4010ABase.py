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
:summary: wrapper to base bluetooth network simulator Agilent N4010A
:since: 23/03/2011
:author: ymorel
"""

import ctypes


def Connect(eqt, conn_type, board_id, gpib_addr, ip_addr):
    """
    Wraps to Connect function
    :raise TestEquipmentException: Connect driver function failed
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type conn_type: str
    :param conn_type: the connection type to use:
        - "GPIB"
        - "TCPIP"
    :type board_id: integer
    :param board_id: GPIB board ID
    :type gpib_addr: integer
    :param gpib_addr: GPIB address (between 0 and 15).
    :type ip_addr: str
    :param ip_addr: the IP address of the remote equipment
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - unsigned long: connection handle of the equipment.
        - str: log message
    """
    eqt.get_logger().info("Connection")
    dll = eqt.get_dll()
    handle = ctypes.c_ulong()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Connect(ctypes.c_char_p(conn_type), board_id, gpib_addr,
                      ctypes.c_char_p(ip_addr), ctypes.byref(handle), ctypes.byref(err_msg))
    return err, handle.value, err_msg.value


def Disconnect(eqt):
    """
    Wraps to Disconnect function
    :raise TestEquipmentException: Disconnect driver function failed
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Disconnection from Agilent N4010A")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Disconnect(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def GetEqId(eqt):
    """
    Wraps to GetEqtId function
    :raise TestEquipmentException: GetEqtId driver function failed
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: identification str of the equipment
        - str: log message
    """
    eqt.get_logger().info("Get identification string")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    eq_id = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetEqId(handle, eq_id, ctypes.byref(err_msg))
    return err, eq_id.value, err_msg.value


def PerformFullPreset(eqt):
    """
    Wraps to PerformFullPreset function
    :raise TestEquipmentException: PerformFullPreset driver function failed
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - str: log message
    """
    eqt.get_logger().info("Perform full preset")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.PerformFullPreset(handle, ctypes.byref(err_msg))
    return err, err_msg
