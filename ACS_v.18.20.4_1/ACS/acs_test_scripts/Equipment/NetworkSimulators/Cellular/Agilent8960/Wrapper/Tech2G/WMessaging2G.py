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
:summary: wrapper for 2G messaging function of Agilent 8960
:since: 08/03/2011
:author: ymorel
"""

import ctypes


def ClearMessageData(eqt):
    """
    Wraps to ClearMessageData driver function
    :raise TestEquipmentException: call to ClearMessageData
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Clear message data")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.ClearMessageData2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetSmsMessageQueuingState(eqt, state):
    """
    Wraps to SetSmsMessageQueuingState driver function
    :raise TestEquipmentException: call to SetSmsMessageQueuingState
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: desired state:
        - "ON"
        - "OFF"
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set SMS message queuing state to %s", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSmsMessageQueuingState2G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetLastSMS(eqt):
    """
    Wraps to GetLastSMS driver function
    :raise TestEquipmentException: call to GetLastSMS
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the text of the last SMS
    """
    eqt.get_logger().info("Get last SMS text")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    sms_text = ctypes.c_char_p('\x00' * 1024)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetLastSMS2G(handle, 1024, sms_text, ctypes.byref(err_msg))
    return err, sms_text.value, err_msg.value


def GetLastSMSLength(eqt):
    """
    Wraps to GetLastSMSLength driver function
    :raise TestEquipmentException: call to GetLastSMSLength
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the length of the last SMS
    """
    eqt.get_logger().info("Get last SMS length")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    length = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetLastSMSLength2G(
        handle,
        ctypes.byref(length),
        ctypes.byref(err_msg))
    return err, length.value, err_msg.value


def GetNbReceivedSMS(eqt):
    """
    Wraps to GetNbReceivedSMS driver function
    :raise TestEquipmentException: call to GetNbReceivedSMS
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: the number of SMS received by the test set
    """
    eqt.get_logger().debug("Get number of received SMS")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    nb_received = ctypes.c_long()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetNbReceivedSMS2G(
        handle,
        ctypes.byref(nb_received),
        ctypes.byref(err_msg))
    return err, nb_received.value, err_msg.value


def MoveNextSMS(eqt):
    """
    Wraps to MoveNextSMS driver function
    :raise TestEquipmentException: call to MoveNextSMS
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Move to next SMS")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.MoveNextSMS2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def SetSMSDataCodingScheme(eqt, scheme):
    """
    Wraps to SetSMSDataCodingScheme driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type scheme: integer
    :param scheme: data coding scheme to set (0 to 255)
    :raise TestEquipmentException: call to SetSMSDataCodingScheme
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set data coding scheme to %d", scheme)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSMSDataCodingScheme2G(
        handle,
        ctypes.c_int(scheme),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SelectSMSContent(eqt, sms_content):
    """
    Wraps to SelectSMSContent driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sms_content: str
    :param sms_content: the SMS content to select.
        Possible values:
            - "TXT1"
            - "TXT2"
            - "CTEX"
            - "CDAT"
    :raise TestEquipmentException: call to SelectSMSContent
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Select %s SMS content", sms_content)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SelectSMSContent2G(
        handle,
        ctypes.c_char_p(sms_content),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SelectSMSTransportation(eqt, sms_trans):
    """
    Wraps to SelectSMSTransportation driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sms_trans: str
    :param sms_trans: the SMS transportation mechanism to select.
        Possible values:
            - "GSM"
            - "GPRS"
    :raise TestEquipmentException: call to SelectSMSTransportation
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Select %s SMS transportation", sms_trans)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SelectSMSTransportation2G(
        handle,
        ctypes.c_char_p(sms_trans),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetCustomSMSText(eqt, sms_text):
    """
    Wraps to SetCustomSMSText driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sms_text: str
    :param sms_text: the text of the SMS to send
    :raise TestEquipmentException: failed to call SetCustomSMSText
    driver function
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set SMS text to %s", sms_text)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCustomSMSText2G(
        handle,
        ctypes.c_char_p(sms_text),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SendSMS(eqt):
    """
    Wraps to SendSMS driver function
    :raise TestEquipmentException: failed to call SendSMS
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Send SMS")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SendSMS2G(handle, ctypes.byref(err_msg))
    return err, err_msg.value


def GetSMSSendState(eqt):
    """
    Wraps to GetSMSSendState driver function
    :raise TestEquipmentException: failed to call GetSMSSendState
    driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the str representation of the SMS send state.
        Possible returned values:
            - "IDLE" => default returned value
            - "SEND"
            - "ACK"
            - "NACK"
            - "REJ"
            - "FAIL"
    """
    eqt.get_logger().info("Get SMS send state")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    state = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetSMSSendState2G(handle, 256, state, ctypes.byref(err_msg))
    return err, state.value, err_msg.value


def SetServiceCenterAddress(eqt, service_center):
    """
    Wraps to SetServiceCenterAddress driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type service_center: str
    :param service_center: the address of the SMS service center
    to set
    :raise TestEquipmentException: call to SetServiceCenterAddress
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set SMS service center address to %s",
                          service_center)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetServiceCenterAddress2G(
        handle,
        ctypes.c_int(len(service_center)),
        ctypes.c_char_p(service_center),
        ctypes.byref(err_msg))
    return err, err_msg.value


def SetSmsMoLoopbackState(eqt, state):
    """
    Wraps to SetSmsMoLoopbackState driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type state: str
    :param state: the desired state. Possible values:
        - "ON"
        - "OFF"
    :raise TestEquipmentException: call to SetSmsMoLoopbackState
    driver function failed
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set SMS mobile originated loopback %s, if state is on Send MO SMS to NetworkSimulator then Send MT SMS to DUT ", state)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSmsMoLoopbackState2G(
        handle,
        ctypes.c_char_p(state),
        ctypes.byref(err_msg))
    return err, err_msg.value


def GetSmsDestination(eqt):
    """
    Wraps to GetSmsDestination driver function
    :raise TestEquipmentException: call to GetSmsDestination
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the destination of the last received SMS
    """
    eqt.get_logger().info("Get last received SMS destination")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    dest = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetSmsDestination2G(handle, 256, dest, ctypes.byref(err_msg))
    return err, dest.value, err_msg.value


def SetCustomSMSData(eqt, sms_data):
    """
    Wraps to SetCustomSMSData driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type sms_data: str
    :param sms_data: the data of the SMS to send. Size of the data
    to set must be a pair number between 0 and 280. String of hexadecimal
    digits (0-9; a-f; A-F).
    :raise TestEquipmentException: failed to call SetCustomSMSData
    driver function
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set SMS data to %s", sms_data)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetCustomSMSData2G(
        handle,
        ctypes.c_int(len(sms_data)),
        ctypes.c_char_p(sms_data), ctypes.byref(err_msg))
    return err, err_msg.value


def SetSmsSenderAdress(eqt, address):
    """
    Wrap to SetSmsSenderAdress driver function
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :type address: str
    :param address: address to be set. Range: 2 to 20 characters.
    SMS address to set. ASCII str of BCD digits 0-9, the symbols
    * and #, and the lower case characters a, b, c, and f.
    :rtype: integer
    :return: error code of the driver function
    """
    eqt.get_logger().info("Set SMS sender address to %s", address)
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.SetSmsSenderAdress2G(
        handle,
        ctypes.c_int(len(address)),
        ctypes.c_char_p(address), ctypes.byref(err_msg))
    return err, err_msg.value


def GetLastSMSTransportation(eqt):
    """
    Wraps to GetLastSMSTransportation driver function
    :raise TestEquipmentException: call to GetLastSMSTransportation
    driver function failed
    :type eqt: Agilent8960
    :param eqt: the equipment that uses the wrapper
    :rtype: str
    :return: the last SMS transportation:
        - "INV"
        - "GSM"
        - "GPRS"
    """
    eqt.get_logger().info("Get last SMS transportation")
    dll = eqt.get_dll()
    handle = eqt.get_handle()
    transportation = ctypes.c_char_p('\x00' * 256)
    err_msg = ctypes.c_char_p('\x00' * 1024)
    err = dll.GetLastSMSTransportation2G(
        handle,
        256,
        transportation,
        ctypes.byref(err_msg))
    return err, transportation.value, err_msg.value
