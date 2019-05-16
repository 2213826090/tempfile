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

:organization: INTEL NDG SW
:summary: wrapper for Diolan IO adapter. contain Result codes definition, import from the C header file
:since: 07/08/2014
:author: dpierrex
"""


def DLN_SUCCEEDED(result):
    return hex(result) < '0x80'


def DLN_FAILED(result):
    return hex(result) > '0x80'

# Successful return codes (DLN_RESULT < 0x80)
DLN_RES_SUCCESS = 0
DLN_RES_SUCCESSFUL_REINIT = 1
DLN_RES_PENDING = 2

# Error codes (DLN_RESULT > 0x80)
DLN_RES_HOST_LOOKUP_FAILED = 0xA4
DLN_RES_CONNECTION_FAILED = 0x94
DLN_RES_HOST_NAME_TOO_LONG = 0x98
DLN_RES_ALREADY_CONNECTED = 0x99
DLN_RES_CONNECTION_LOST = 0xA0
DLN_RES_NOT_CONNECTED = 0xA1
DLN_RES_HARDWARE_NOT_FOUND = 0x81
DLN_RES_OUTDATED_DRIVER = 0x82
DLN_RES_FAIL = 0x83
DLN_RES_MESSAGE_ABSENT = 0x84
DLN_RES_BAD_PARAMETER = 0x85
DLN_RES_MEMORY_ERROR = 0x86
DLN_RES_NOT_INITIALIZED = 0x87
DLN_RES_INVALID_COMMAND_SIZE = 0x88
DLN_RES_INVALID_RESPONSE_SIZE = 0x89
DLN_RES_INVALID_MESSAGE_SIZE = 0x8A
DLN_RES_NOTIFICATION_NOT_REGISTERED = 0x8B
# DLN_RES_INVALID_STREAM_NUMBER = 0x8C
DLN_RES_TRANSACTION_TIMEOUT = 0x8D
DLN_RES_DEVICE_REMOVED = 0x8E
DLN_RES_INVALID_HANDLE = 0x8F
DLN_RES_INVALID_MESSAGE_TYPE = 0x90
DLN_RES_NOT_IMPLEMENTED = 0x91
DLN_RES_TOO_MANY_CONNECTIONS = 0x92
DLN_RES_ALREADY_INITIALIZED = 0x93
DLN_RES_INTERNAL_ERROR = 0x96
DLN_RES_DEVICE_NUMBER_OUT_OF_RANGE = 0x97
DLN_RES_MESSAGE_SENDING_FAILED = 0xA2
DLN_RES_NO_FREE_STREAM = 0xA3

DLN_RES_PIN_IN_USE = 0xA5

DLN_RES_INVALID_LED_NUMBER = 0xA6
DLN_RES_INVALID_LED_STATE = 0xA7
DLN_RES_INVALID_PORT_NUMBER = 0xA8
DLN_RES_INVALID_EVENT_TYPE = 0xA9

DLN_RES_PIN_NOT_CONNECTED_TO_MODULE = 0xAA