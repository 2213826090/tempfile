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
:summary: wrapper for voice call 3G functions of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def MtOriginateCall(eqt):
    """
    Wraps to MtOriginateCall driver function
    :raise TestEquipmentException: failed to call MtOriginateCall
    driver function
    :type eqt: Agilent8960
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


def GetCallControlStatus(eqt):
    """
    Wraps to GetCallControlStatus driver function
    :raise TestEquipmentException: failed to call GetCallControlStatus
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the call control status. Possible returned values:
        - "CALL" : Alerting
        - "CONNECTED" : Connected
        - "HAND" : Handoff
        - "IDLE" : Idle
        - "PAG"  : Paging
        - "REG"  : Registering
        - "REL"  : Releasing
        - "SREQ" : Setup Request
    """
    eqt.get_logger().debug("Get call control status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    status = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetCallControlStatus3G(handle, status, ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def SetAmrDlRate(eqt, rab, amr_rate):
    """
    Wraps to SetAmrDlRate driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type rab: : str
    :param rab: the radio access bearer for which to set the AMR rate.
    :type amr_rate: str
    :param amr_rate: the AMR rate to set.
    Possible AMR rates in function of radio access bearer:
        - "AMR1220P":
            - "AMR475" | "AMR590" | "AMR795" | "AMR1220"
        - "AMRS":
            - "AMR475" | "AMR590" | "AMR795" | "AMR1220"
        - "WAMR1"
            - "AMR660" | "AMR885" | "AMR1265"
        - "WAMR2"
            - "AMR660" | "AMR885" | "AMR1265" | "AMR1220" | "AMR1585"
        - "WAMR3"
            - "AMR660" | "AMR885" | "AMR1265" | "AMR2385"
        - "WAMR4"
            - "AMR660" | "AMR885" | "AMR1265"
    :raise TestEquipmentException: failed to call SetAmrDlRate
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetAmrDlRate3G(
        handle,
        ctypes.c_char_p(rab),
        ctypes.c_char_p(amr_rate),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetAmrUlSubset(eqt, vcr, subset):
    """
    Wraps to SetAmrUlSubset driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type vcr: : str
    :param vcr: the voice call rate.
    :type subset: str
    :param subset: the subset to set.
    Possible subsets in function of voice call rate:
        - "AMR1220P":
            - "SUB1" | "SUB2" | "SUB3" | "SUB4"
        - "WAMR4"
            - "SUB1" | "SUB2" | "SUB3"
    :raise TestEquipmentException: failed to call SetAmrUlSubset
    driver function
    :rtype: integer
    :return: the error code of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetAmrUlSubset3G(
        handle,
        ctypes.c_char_p(vcr),
        ctypes.c_char_p(subset),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SendAmrUlSubset(eqt):
    """
    Wraps to SendAmrUlSubset driver function
    :raise TestEquipmentException: failed to call SendAmrUlSubset
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SendAmrUlSubset3G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetAudioCodec(eqt, codec):
    """
    Wraps to SetAudioCodec driver function
    :raise TestEquipmentException: failed to call SetAudioCodec
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type codec: str
    :param codec: the audio codec to set. Possible values:
        - "AMR1220" | "AMR1020" | "AMR795"  | "AMR740"
        - "AMR670"  | "AMR590"  | "AMR515"  | "AMR475"
        - "AMR2385" | "AMR2305" | "AMR1985" | "AMR1825"
        - "AMR1585" | "AMR1425" | "AMR1265" | "AMR885"
        - "AMR660"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set audio codec to %s", codec)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
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
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type config: str
    :param config: speech configuration to set. Possible values:
                - "ECHO"
                - "RTV"
                - "EXT"
                - "PESQ"
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


def SetEchoLoopbackDelay(eqt, delay):
    """
    Wraps to SetEchoLoopbackDelay driver function
    :raise TestEquipmentException: failed to call SetEchoLoopbackDelay
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type delay: double
    :param config: the speech echo loopback delay to set. A double
    from 0.1 to 4 with a resolution of 0.2 (in seconds).
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set echo loopback delay to %f", delay)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetEchoLoopbackDelay3G(
        handle,
        ctypes.c_double(delay),
        ctypes.byref(err_msg))
    return err, err_msg.value


def VoiceCallNetworkRelease(eqt):
    """
    Wraps to VoiceCallNetworkRelease driver function
    :raise TestEquipmentException: failed to call VoiceCallNetworkRelease
    driver function
    :type eqt: Agilent8960
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