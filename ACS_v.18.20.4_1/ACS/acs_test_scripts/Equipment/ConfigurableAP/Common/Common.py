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
:summary: Common class to provide utilities methods for configurable Access Point
:since:20/01/2012
:author: ssavrimoutou
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

import itertools
import operator
import hashlib
from functools import reduce


class WifiAuthenticationTypes(object):

    """
    Class enumerating the list of Wifi Authentication types
    The names contain the standard encryption and its algorithm
    """
    # Open authentication
    OPEN = "OPEN"
    # List of WEP authentication
    WEP_64 = "WEP64"
    WEP_64_OPEN = "WEP64-OPEN"
    WEP_128 = "WEP128"
    WEP_128_OPEN = "WEP128-OPEN"
    WEP_256 = "WEP256"
    # List of WPA authentication
    WPA_PSK_TKIP = "WPA-PSK-TKIP"
    EAP_WPA = "EAP-WPA"
    # List of WPA2 authentication
    WPA2_PSK_AES = "WPA2-PSK-AES"
    EAP_WPA2 = "EAP-WPA2"

    # Non mandatory Wifi configurations (according to Wifi specifications)
    WPA_PSK_AES = "WPA-PSK-AES"
    WPA_PSK_TKIP_AES = "WPA-PSK-TKIP-AES"
    WPA2_PSK_TKIP = "WPA2-PSK-TKIP"
    WPA2_PSK_TKIP_AES = "WPA2-PSK-TKIP-AES"
    WPA_WPA2_PSK_TKIP_AES = "WPA-WPA2-PSK-TKIP-AES"

class WifiKeyExchangeTypes(object):

    """
    Class enumerating the list of wifi key exchange types
    The names contain the standard exchange mode
    """
    # WPS (Wireless Protected Setup)
    WPS_PIN_FROM_AP = "WPS-PIN-AP"
    WPS_PIN_FROM_DUT = "WPS-PIN-DUT"
    WPS_PBC = "WPS-PBC"


def prng(x, a, c, m):
    """
    Here is an implementation of a generic Linear Congruential Generator used to compute 40/64 bits WEP keys in python
    Further details are explained at following link : http://en.wikipedia.org/wiki/Linear_congruential_generator

    :type x :int in range : 0 <= x < m
    :param x: the "seed" or "start value"

    :type m : int > 0
    :param m : the "modulus"

    :type a : int 0 < a < m
    @ param a : the "multiplier"

    :type c : int 0 <= c < m
    :param c : the "increment"
    """
    while True:
        x = (a * x + c) % m
        yield x


def build_wep_key_64(passphrase):
    """
    Generate the 64 bits WEP key

    :type passphrase : str
    :param passphrase : The passphrase to connect in WEP 64 bits

    :rtype: str
    :return: the WEP 64 bits key
    """

    if len(passphrase) == 0:
        raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Empty passphrase")

    bits = [0, 0, 0, 0]
    for i, c in enumerate(passphrase):
        bits[i & 3] ^= ord(c)
    val = reduce(operator.__or__, (b << 8 * i for (i, b) in enumerate(bits)))
    keys = []
    for i, b in enumerate(itertools.islice(prng(val, 0x343fd, 0x269ec3, 1 << 32), 20)):
        keys.append((b >> 16) & 0xff)
    return ((('%02X' * 5 + ' ') * 4) % tuple(keys)).strip().split(' ')


def build_wep_key_128(passphrase):
    """
    Generate the 128 bits WEP key

    :type passphrase : str
    :param passphrase : The passphrase to connect in WEP 128 bits

    :rtype: str
    :return: the WEP 128 bits key
    """
    if len(passphrase) == 0:
        raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, "Empty passphrase")

    buf = (passphrase * ((64 / len(passphrase)) + 1))[0:64]
    m = hashlib.md5()  # pylint: disable=E1101
    m.update(buf)
    return [m.hexdigest()[0:26].upper()]
