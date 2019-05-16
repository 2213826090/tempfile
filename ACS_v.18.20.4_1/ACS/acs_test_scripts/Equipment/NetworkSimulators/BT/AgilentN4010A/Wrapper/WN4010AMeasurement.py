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
:summary: wrapper to measurement functions of bluetooth network simulator
Agilent N4010A
:since: 23/03/2011
:author: ymorel
"""

import ctypes


def GetOutputPowerAverage(
    eqt,
    channel="SUMMARY",
    meas_type="MAX",
        occurrence=1):
    """
    Wraps to GetOutputPowerAverage function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type channel: str
    :param channel: the channel to measure
        - "SUMMARY": measure all channels
        - "LOW"
        - "MEDIUM"
        - "HIGH"
    :type meas_type: str
    :param meas_type: the type of measurement to operate
        - "MIN"
        - "MAX"
        - "AVERAGE"
    :type occurrence: integer
    :param occurrence: the occurrence of the test in the sequence (1 to 10)
    :raise TestEquipmentException: call to GetOutputPowerAverage driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - double: the result of the measurement
        - str: log message
    """
    eqt.get_logger().info("Get output power")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    result = ctypes.c_double()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetOutputPowerAverage(
        handle,
        ctypes.c_char_p(channel),
        ctypes.c_char_p(meas_type),
        ctypes.c_int(occurrence),
        ctypes.byref(result),
        ctypes.byref(err_msg))
    return err, result.value, err_msg.value


def GetSingleSlotSensitivityBER(
    eqt,
    channel="SUMMARY",
    meas_type="",
        occurrence=1):
    """
    Wraps to GetSingleSlotSensitivityBER function
    :type eqt: AgilentN4010A
    :param eqt: the equipment that uses the wrapper
    :type channel: str
    :param channel: the channel to measure
        - "SUMMARY": measure all channels
        - "LOW"
        - "MEDIUM"
        - "HIGH"
    :type meas_type: str
    :param meas_type: the type of measurement to operate
        - "MAX": specifies that you want the maximum result returned
        - "": standard measurement
    :type occurrence: integer
    :param occurrence: the occurrence of the test in the sequence (1 to 10)
    :raise TestEquipmentException: call to GetSingleSlotSensitivityBER driver function failed
    :rtype: tuple
    :return:
        - integer: error code of the driver function
        - double: the result of the measurement
        - str: log message
    """
    eqt.get_logger().info("Get single slot sensitivity BER")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    result = ctypes.c_double()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetSingleSlotSensitivityBER(
        handle,
        ctypes.c_char_p(channel),
        ctypes.c_char_p(meas_type),
        ctypes.c_int(occurrence),
        ctypes.byref(result),
        ctypes.byref(err_msg))
    return err, result.value, err_msg.value
