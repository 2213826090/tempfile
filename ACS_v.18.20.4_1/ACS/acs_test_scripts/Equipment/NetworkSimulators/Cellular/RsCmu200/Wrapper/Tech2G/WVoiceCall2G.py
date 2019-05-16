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

:summary: wrapper for RS CMU200 2G voice call functions

:organization: INTEL MCG PSI
:author: ymorel
:since: 05/04/2011
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
    err = dll.MtOriginateCall2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetMtOriginateCallTimeout(eqt, timeout):
    """
    Wraps to SetMtOriginateCallTimeout driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type timeout: integer
    :param timeout: the maximum timeout before aborting call setup to set (0..60).
    :raise TestEquipmentException: failed to call MtOriginateCall
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Set Mobile originated call timeout to %s s",
        str(timeout))
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetMtOriginateCallTimeout2G(
        handle,
        ctypes.c_int(timeout),
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
    :return: the call control status.
    """
    eqt.get_logger().debug("Get call control status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    status = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetCallControlStatus2G(handle, status, ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def SetAudioCodec(eqt, codec):
    """
    Wraps to SetAudioCodec driver function
    :raise TestEquipmentException: failed to call SetAudioCodec
    driver function
    :type eqt: RsCmu200
    :param eqt: the equipment that uses the wrapper
    :type codec: str
    :param codec: the audio codec to set. Possible values:
        - "FR" | "HR" | "EHR"
        - "FR_AMR_1220" | "FR_AMR_1020" | "FR_AMR_795" | "FR_AMR_740"
        - "FR_AMR_670"  | "FR_AMR_590"  | "FR_AMR_515" | "FR_AMR_475"
        - "HR_AMR_795"  | "HR_AMR_740"  | "HR_AMR_670" | "HR_AMR_590"
        - "HR_AMR_515"  | "HR_AMR_475"
        - Following WFSP codecs are not supported by CMU200 :
            - "AMR_WB_1265" | "AMR_WB_885" | "AMR_WB_660"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set audio codec to %s", codec)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetAudioCodec2G(
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
    :param config: the speech configuration to set. Possible values:
        - "ECHO"
        - "NONE"
        - "PRBS15"
        - "PRBS9"
        - "SIN300"
        - "SIN1000"
        - "SIN3000"
        - "MULTITONE"
        - "CUST": CUSTOM
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set speech configuration to %s", config)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSpeechConfiguration2G(
        handle,
        ctypes.c_char_p(config),
        ctypes.byref(err_msg))
    return err, err_msg.value


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
    err = dll.VoiceCallNetworkRelease2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value