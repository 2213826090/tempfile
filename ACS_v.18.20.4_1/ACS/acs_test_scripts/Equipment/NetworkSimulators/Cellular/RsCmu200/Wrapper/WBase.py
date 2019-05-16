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
:summary: wrapper for basic functions of RS CMU200
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def Connect(eqt, board_id, gpib_address):
    """
    Wraps to Connect function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type board_id: integer
    :param board_id: GPIB board ID
    :type gpib_address: integer
    :param gpib_address: GPIB address (between 0 and 15).
    :rtype: tuple
    :return:
        - error code of driver function
        - handle value
        - error message
    """
    eqt.get_logger().info("Connection to RS CMU200")
    dll = eqt.get_dll()
    handle = ctypes.c_ulong()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Connect(
        board_id,
        gpib_address,
        ctypes.byref(handle),
        ctypes.byref(err_msg))
    return err, handle.value, err_msg.value


def Disconnect(eqt):
    """
    Wraps to Disconnect function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    """
    eqt.get_logger().info("Disconnection from RS CMU200")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.Disconnect(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def PerformFullPreset(eqt):
    """
    Wraps to PerformFullPreset function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    """
    eqt.get_logger().info("Perform full preset")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.PerformFullPreset(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetAppFormat(eqt, app_format):
    """
    Wraps to SetAppFormat function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type app_format: str
    :param app_format: the application format to set
        - "1xEV-DO"
        - "AMPS/136"
        - "GSM/GPRS"
        - "IS-2000/IS-95/AMPS"
        - "IS-856"
        - "WCDMA"
    """
    eqt.get_logger().info("Set application format to %s", app_format)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAppFormat(
        handle,
        ctypes.c_char_p(app_format),
        ctypes.byref(err_msg))
    return err, err_msg.value


def LoadConfigurationFile(eqt, source, filename):
    """
    Wraps to LoadConfigurationFile function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type source: str
    :param source: the source from which to load the configuration.
    Possible values:
        - "INTERNAL"
        - "EXTERNAL"
    :type filename: str
    :param filename: the configuration file to load
    """
    eqt.get_logger().info("Load configuration file : %s", filename)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.LoadConfigurationFile(
        handle,
        ctypes.c_char_p(source),
        ctypes.c_char_p(filename),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetUlDlAttenuation(eqt, ul_att, dl_att):
    """
    Wraps to SetUlDlAttenuation function (RsCMU200 only)
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type ul_att: double
    :param ul_att: uplink attenuation to set:
        - 2G: -50 dB to +90 dB
        - 3G: -50 dB to +50 dB
    :type dl_att: double
    :param dl_att: downlink attenuation to set:
        - 2G: -50 dB to +90 dB
        - 3G: -50 dB to +50 dB
    :raise TestEquipmentException: failed to call SetUlDlAttenuation driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Set UL and DL attenuation to %f and %f",
        ul_att,
        dl_att)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetUlDlAttenuation(
        handle,
        ctypes.c_double(ul_att),
        ctypes.c_double(dl_att),
        ctypes.byref(err_msg))
    return err, err_msg.value
