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
:summary: This file implements the LAB WIFI TETHERING USB WEB BROWSING UC
The goal of this UC is to validate the tethered USB interface over wifi
:since: 16/10/2013
:author: apairex
"""

import re

from acs_test_scripts.UseCase.Networking.LAB_WIFI_TETHERING_USB_BASE import LabWifiTetheringUsbBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabWifiTetheringUsbWebBrowsing(LabWifiTetheringUsbBase):

    """
    Lab Wifi Tethering USB Web Browsing Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiTetheringUsbBase.__init__(self, tc_name, global_config)

        # TestCase Parameters
        self._website_url = str(self._tc_parameters.get_param_value("WEBSITE_URL"))
        self._webpage_loading_timeout = self._tc_parameters.get_param_value("TIMEOUT")

        if self._website_url.upper() in ["", "NONE"]:
            # In case of empty WEBSITE_URL TC parameter, use the IP server in the bench config
            self._website_url = self._wifi_server_ip_address

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Configure and connect wifi to the AP
        LabWifiTetheringUsbBase.set_up(self)

        self._webpage_loading_timeout = int(self._webpage_loading_timeout)

        # Extract the IP/Server name from the URL
        host_search = re.search(r'(?:https?://)?([^:/$]+)', self._website_url)
        if host_search is None:
            msg = "Unable to extract host address from URL: %s" % self._website_url
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        host = host_search.group(1)

        # Test that we cannot establish a connection to the web server
        self._test_server_unreachable(host)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiTetheringUsbBase.run_test(self)

        # Enable USB Tethering on DUT
        self._networking_api.start_usb_tethering(unplug=True)

        # Tethering should have now started on DUT, wait for USB interface to come up
        self._computer.dhclient(self._computer.get_usb_interface())

        # Open the browser and load the url before timeout
        try:
            self._computer.web_browsing_test(self._website_url, self._webpage_loading_timeout)
        except TestEquipmentException as teexc:
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, teexc.get_specific_message())

        # Stop USB tethering
        self._networking_api.stop_usb_tethering(unplug=True)

        return Global.SUCCESS, "No error"
