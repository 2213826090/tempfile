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
:summary: wrapper for 3G voice call functions of RS CMU200
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def MtOriginateCall(eqt):
    """
    Wraps to MtOriginateCall driver function
    :raise TestEquipmentException: failed to call MtOriginateCall
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Network originated call")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.MtOriginateCall3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetAudioCodec(eqt, codec):
    """
    Wraps to SetAudioCodec driver function
    :raise TestEquipmentException: failed to call SetAudioCodec
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type codec: str
    :param codec: the audio codec to set. Possible values:
        - "AMR_NB_1220" | "AMR_NB_1020" | "AMR_NB_795"  | "AMR_NB_740"
        - "AMR_NB_670"  | "AMR_NB_590"  | "AMR_NB_515"  | "AMR_NB_475"
        - "AMR_WB_2385" | "AMR_WB_2305" | "AMR_WB_1985" | "AMR_WB_1825"
        - "AMR_WB_1585" | "AMR_WB_1425" | "AMR_WB_1265" | "AMR_WB_885"
        - "AMR_WB_660"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set audio codec to %s", codec)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAudioCodec3G(
        handle,
        ctypes.c_char_p(codec),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSpeechConfiguration(eqt, config):
    """
    Wraps to SetSpeechConfiguration driver function
    :raise TestEquipmentException: failed to call SetSpeechConfiguration
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type config: str
    :param config: . Possible values:
        - "ECHO"
        - "RT_VOICE_CODER"
        - "EXTERNAL"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set speech configuration to %s", config)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSpeechConfiguration3G(
        handle,
        ctypes.c_char_p(config),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetCallControlStatus(eqt):
    """
    Wraps to GetCallControlStatus driver function
    :raise TestEquipmentException: failed to call GetCallControlStatus
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the call control status. Possible returned values:
        - "CALL": alerting
        - "CONNECTED": connected
        - "HAND": handoff
        - "IDLE": idle
        - "PAG": paging
        - "REG": registering
        - "REL": releasing
        - "SREQ": setup request
    """
    eqt.get_logger().debug("Get call control status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    status = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetCallControlStatus3G(handle, status, ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def VoiceCallNetworkRelease(eqt):
    """
    Wraps to VoiceCallNetworkRelease driver function
    :raise TestEquipmentException: failed to call VoiceCallNetworkRelease
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Release voice call")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.VoiceCallNetworkRelease3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value