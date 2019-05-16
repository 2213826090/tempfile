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

:organization: INTEL CCG OPM PC&WTE
:summary: This file implements the UECMD factory for android N dessert
:since: 2015-08-03
:author: ldesvigx
"""

from acs_test_scripts.Device.UECmd.Imp.Android.M.Factory import Factory as DadFactory
import os


class Factory(DadFactory):

    """
    UECmd factory
    """

    def __init__(self, device):
        DadFactory.__init__(self, device)

    def get_uecmd(self, name):
        """
        Instantiate UECmd module

        :type name: str
        :param name: Name of the UECmd module to instantiate

        :return: UECmd instance if available
        """

        # Check local first
        my_path = os.path.dirname(os.path.abspath(__file__))
        # pylint: disable=E1101
        uecmd = self._get_local_uecmd(my_path, name)

        if not uecmd:
            # Ask to the father
            uecmd = DadFactory.get_uecmd(self, name)

        return uecmd
