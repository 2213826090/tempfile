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
:summary: Utilities class for MMS implementation
:since: 31/07/2013
:author: dgonza4x
"""

import re


def is_pin_valid(pin):
    """
    Checks that a SIM PIN code is correct.
    A PIN code must represents a number containing 4 to 8 digits

    :param pin: pin code to check
    :type pin: str

    :rtype: bool
    :return: Check status : C{True} means that PIN is correct,
    C{False} in case of an invalid PIN
    """
    pin_str = str(pin)
    reg_expr = re.compile("^[0-9]{4,8}$")
    return reg_expr.match(pin_str) is not None


def is_puk_valid(puk):
    """
    Checks that a SIM PUK code is correct.
    A PUK code must represents a number containing 8 digits.

    :param puk: sim puk code to check
    :type puk: str

    :rtype: bool
    :return: Check status : C{True} means that PUK is correct,
    C{False} otherwise
    """
    puk_str = str(puk)
    reg_expr = re.compile("^[0-9]{8}$")
    return reg_expr.match(puk_str) is not None
