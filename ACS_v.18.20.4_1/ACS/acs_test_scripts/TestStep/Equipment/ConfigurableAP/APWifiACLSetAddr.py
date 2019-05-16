"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements Test Step for WiFi AP set ACL addr
:since 16/07/2014
:author: jfranchx
"""

from acs_test_scripts.TestStep.Equipment.ConfigurableAP.APBase import APBase
from acs_test_scripts.Utilities.NetworkingUtilities import is_valid_mac_address
from ErrorHandling.AcsConfigException import AcsConfigException


class APWifiACLSetAddr(APBase):
    """
    Implements WiFi AP Set ACL Addr
    """

    def run(self, context):
        """
        Run the test step
        """
        APBase.run(self, context)

        # Check MAC_ADDR is a valid MAC address
        if not is_valid_mac_address(self._pars.mac_addr):
            msg = "Parameter %s is not a valid mac address" % self._pars.mac_addr
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Add or remove a MAC Address
        if self._pars.set_addr == "ADD":
            self._configurable_ap.add_mac_address_to_acl(self._pars.mac_addr)
        else:
            self._configurable_ap.del_mac_address_from_acl(self._pars.mac_addr)
