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
:summary: wrapper for 2G voice call functions of Agilent 8960
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
    err = dll.MtOriginateCall2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetMtOriginateCallTimeout(eqt, timeout):
    """
    Wraps to SetMtOriginateCallTimeout driver function
    :type timeout: integer
    :param timeout: the maximum timeout before aborting call setup to set (0..999).
    :raise TestEquipmentException: failed to call MtOriginateCall
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info(
        "Set mobile originated call timeout to %s s",
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
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the call control status.
        Possible returned values:
            - "IDLE" : idle
            - "SREQ" : setup request
            - "ALER" : alerting
            - "CONN" : connected
            - "DISC" : disconnecting
    """
    eqt.get_logger().debug("Get call control status")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    status = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetCallControlStatus2G(handle, status, ctypes.byref(err_msg))
    return err, status.value, err_msg.value


def SetChannelModeCodecSet(eqt, mode, codec_set):
    """
    Wraps to SetChannelModeCodecSet driver function
    :raise TestEquipmentException: failed to call SetChannelModeCodecSet
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the mode for which the codec set will be set.
    :type codec_set: str
    :param codec_set: the str containing the codecs to set.
    The codec set must be defined in order, with codec 1 being
    first and codec 4 being last. Also, the codec rates must be
    in ascending order such that the codec with lowest rate is
    first and the codec with the highest rate is last. For WFSP
    mode, only three codecs are required.
    Possible values of the codec set in function of the mode:
        - AFSP codecs (Adaptive Full Speech) :
        (example: "AFS5900, AFS7400, AFS12200, UNUS")
            - "AFS12200" | "AFS10200" | "AFS7950" | "AFS7400" | "AFS6700" |
              "AFS5900" | "AFS5150" | "AFS4750" | "UNUS"
        - AHSP codecs (Adaptive Half Speech) :
        (example: "AFS5900, AHS6700, AHS7400, AHS7950")
            - "AHS7950" | "AHS7400" | "AHS6700" | "AHS5900" | "AHS5150" |
              "AHS4750" | "UNUS"
        - WFSP codecs (Wideband Full Speech) :
        (example: "WFS12650 , UNUS, UNUS")
            - "WFS6600" | "WFS8850" | "WFS12650" | "UNUS"
    :rtype: integer
    :return: the error code of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    codec_set_table = codec_set.split(",")
    codec_set_str = (ctypes.c_char_p * len(codec_set_table))()

    for i in range(len(codec_set_table)):
        codec_set_str[i] = ctypes.c_char_p(codec_set_table[i])

    err = dll.SetChannelModeCodecSet2G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.c_int(len(codec_set_table)),
        codec_set_str,
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetChannelModeCurrentCodec(eqt, mode, codec):
    """
    Wraps to SetChannelModeCurrentCodec driver function
    :raise TestEquipmentException: failed to call SetChannelModeCurrentCodec
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the mode for which the codec set will be set.
    :type codec: str
    :param codec: the codec to set
    Possible codec to set in function of the mode:
        - AFSP codecs (Adaptive Full Speech) :
            - "AFS12200" | "AFS10200" | "AFS7950" | "AFS7400" |
              "AFS6700" | "AFS5900" | "AFS5150" | "AFS4750" | "STR" | "MSR"
        - AHSP (Adaptive Half Speech) :
            - "AHS7950" | "AHS7400" | "AHS6700" | "AHS5900" |
              "AHS5150" | "AHS4750" | "STR" | "MSR"
        - WFSP (Wideband Full Speech) :
            - "WFS6600" | "WFS8850" | "WFS12650" | "STR" | "MSR"
    :rtype: integer
    :return: the error code of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err = dll.SetChannelModeCurrentCodec2G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.c_char_p(codec),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetChannelModeThreshold(eqt, mode, thresholds):
    """
    Wraps to SetChannelModeThreshold driver function
    :raise TestEquipmentException: failed to call SetChannelModeThreshold
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type mode: str
    :param mode: the mode for which the codec set will be set. Possible values:
        - "AFSP" : Adaptive Full Speech
        - "AHSP" : Adaptive Half Speech
        - "WFSP" : Wideband Full Speech
    :type thresholds: str
    :param thresholds: couple of threshold (0 to 31.5) and hysteresis (0 to 7.5) to
    specify when the codec needs to be changed (resolution 0.5)
    Number of threshold values to give in function of the mode:
        - AFSP : 3 couples (example: "6,1.5,10.5,2,18,2.5")
        - AHSP : 3 couples (example: "6,1.5,10.5,2,18,2.5")
        - WFSP : 2 couples (example: "6,1.5,18,2.5")

    :rtype: integer
    :return: the error code of the driver function
    """
    err_msg = ctypes.c_char_p('\x00' * 1024)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    threshold_table = map(float, thresholds.split(","))  # pylint: disable=W0141
    c_thresholds = (ctypes.c_double * len(threshold_table))()
    for i in range(len(threshold_table)):
        c_thresholds[i] = threshold_table[i]
    err = dll.SetChannelModeThreshold2G(
        handle,
        ctypes.c_char_p(mode),
        ctypes.c_int(len(threshold_table)),
        c_thresholds, ctypes.byref(err_msg))
    return err, err_msg.value


def SetHrspSubChannel(eqt, sub_channel):
    """
    Wraps to SetHrspSubChannel driver function
    :raise TestEquipmentException: failed to call SetHrspSubChannel
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sub_channel: integer
    :param sub_channel: the half rate speech sub channel to set (0 | 1).
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set HRSP subchannel to %d", sub_channel)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetHrspSubChannel2G(
        handle,
        ctypes.c_int(sub_channel),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetLogicalSpeechChannel(eqt, lsp):
    """
    Wraps to SetLogicalSpeechChannel driver function
    :raise TestEquipmentException: failed to call SetLogicalSpeechChannel
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type lsp: str
    :param lsp: the logical speech channel to set. Possible values:
        - "FS"
        - "EFS"
        - "HS"
        - "AFS"
        - "AHS"
        - "WFS"
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set logical speech channel to %s", lsp)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetLogicalSpeechChannel2G(
        handle,
        ctypes.c_char_p(lsp),
        ctypes.byref(err_msg))
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
        - FS
        - EFS
        - HS
        - AFSP codecs:
            - "AFS12200" | "AFS10200" | "AFS7950" | "AFS7400" |
              "AFS6700" | "AFS5900" | "AFS5150" | "AFS4750" |
        - AHSP codecs:
            - "AHS7950" | "AHS7400" | "AHS6700" | "AHS5900" |
              "AHS5150" | "AHS4750"
        - WFSP codecs:
            - "WFS6600" | "WFS8850" | "WFS12650"
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
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type config: str
    :param config: . Possible values:
            - "ECHO"
            - "NONE"
            - "PRBS15"
            - "PRBS9"
            - "SIN300"
            - "SIN1000"
            - "SIN3000"
            - "MULTITONE"
            - "SID"
            - "CUST"
            - "RTV"
            - "PESQ"
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


def SetEchoLoopbackDelay(eqt, delay):
    """
    Wraps to SetEchoLoopbackDelay driver function
    :raise TestEquipmentException: failed to call SetEchoLoopbackDelay
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type delay: double
    :param config: the speech echo loopback delay to set. A double
    from 0 to 4 with a resolution of 0.2 (in seconds).
    :rtype: integer
    :return: the error code of the driver function
    """
    eqt.get_logger().info("Set speech echo loopback delay to %f", delay)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetEchoLoopbackDelay2G(
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
    err = dll.VoiceCallNetworkRelease2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value