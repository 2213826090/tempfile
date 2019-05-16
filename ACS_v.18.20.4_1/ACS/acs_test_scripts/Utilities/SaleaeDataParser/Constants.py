"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL CCG
:summary: This file implements the constants for the SaleaeDataParser
:since: 2015-01-27
:author: emarchan
"""


# Reference is 'WCI-2 Transport Layer' (slide 5 of "LTE Debuging Tools Proposal - 0.3.pptx")
DATA_DIRECTION_C2M = "CM"  # Connectivity to Modem
DATA_DIRECTION_M2C = "MC"  # Modem to Connectivity
CSV_INPUT_FILE_FIELDS = [
    'TSTAMP',  # Time since epoch with microseconds precision
    'VALUE',
    'RX_PARITY_ERROR',
    'RX_FRAMING_ERROR']
OUTPUT_LIST_FIELDS = [
    'ID',
    'TSTAMP',
    'DIRECTION',
    'VALUE',
    'RX_FRAMING_ERROR']
RT_FIELDS_TYPES = {  # Message Type Indicator
    0:"REAL_TIME",
    1:"TRANSPORT_CONTROL",
    2:"TRANSPARENT_DATA",
    3:"MWS_INACTIVITY_DURATION",  # if DIRECTION==MC (MWS->BT)
    4:"MWS_SCAN_FREQUENCY",  # if DIRECTION==MC (MWS->BT)
    5:"VENDOR_SPECIFIC_6",
    6:"VENDOR_SPECIFIC_7"}

def hex_to_bin_str(hex_string):
    """
    Converts an hex value into a string of 8 bits.
    :type hex_string: String
    :param result: Value to convert
    :rtype: String
    :return hex_string converted into a string of 8 bits.
    """
    result = bin(int(hex_string, 16)).replace('0b', '')
    # Fill 0s on top of the string to have 8-bits-length
    while len(result) < 8:
        result = "0" + result
    return result
