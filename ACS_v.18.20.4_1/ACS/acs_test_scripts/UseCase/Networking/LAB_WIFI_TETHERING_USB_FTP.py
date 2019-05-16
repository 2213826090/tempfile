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
:summary: This file implements the LAB WIFI TETHERING USB FTP UC
The goal of this UC is to validate the tethered USB interface over wifi
:since: 18/10/2013
:author: apairex
"""

import os
import time
import Queue

from acs_test_scripts.UseCase.Networking.LAB_WIFI_TETHERING_USB_BASE import LabWifiTetheringUsbBase
from UtilitiesFWK.Utilities import Global, str_to_bool
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.UECmdTypes import XFER_DIRECTIONS


class LabWifiTetheringUsbFtp(LabWifiTetheringUsbBase):

    """
    Lab Wifi Tethering USB Web Browsing Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiTetheringUsbBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION", "")
        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("DL_FILE", ""))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(self._ftp_path, self._tc_parameters.get_param_value("UL_FILE", ""))
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT", "")

        # Read the Optional parameter XFER_DURATION from UseCase xml Parameter
        self._xfer_duration = self._tc_parameters.get_param_value("XFER_DURATION", None, int)

        # Read the Wifi Sleep policy parameter
        self._keep_wifi_on = self._tc_parameters.get_param_value("KEEP_WIFI_ON_DURING_SLEEP", "").lower()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Configure and connect wifi to the AP
        LabWifiTetheringUsbBase.set_up(self)

        # Check TC parameter validity
        self._direction = self._direction.upper()
        if self._direction == "DL":
            if self._tc_parameters.get_param_value("DL_FILE", "") == "":
                msg = "DL_FILE TC parameter is missing"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        elif self._direction == "UL":
            if self._tc_parameters.get_param_value("UL_FILE", "") == "":
                msg = "UL_FILE TC parameter is missing"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            msg = "DIRECTION TC parameter is missing"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if not str(self._xfer_timeout).isdigit():
            msg = "XFER_TIMEOUT TC parameter is not valid"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._xfer_timeout = int(self._xfer_timeout)

        # Apply KEEP_WIFI_ON_DURING_SLEEP initial condition
        if self._keep_wifi_on == "always":
            self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["NEVER"])
        elif self._keep_wifi_on == "only_when_plugged_in":
            self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["NEVER_WHILE_PLUGGED"])
        elif self._keep_wifi_on == "never":
            self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["WHEN_SCREEN_OFF"])

        return (Global.SUCCESS, "No error")

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

        # Test FTP transfer
        if (self._direction == "DL"):
            direction = XFER_DIRECTIONS.DL  # pylint: disable=E1101
            file2xfer = self._dlfilename

        elif (self._direction == "UL"):
            direction = XFER_DIRECTIONS.UL  # pylint: disable=E1101
            file2xfer = self._ulfilename

        # if repeat mode has not been defined, do an ftp transfer only one time
        if not self._xfer_duration:
            self._logger.info("FTP transfer %s for file %s ..." % (str(direction), str(file2xfer)))
            self._computer.ftp_xfer(direction,  # pylint: disable=E1101
                                    self._ftp_ip_address,
                                    self._ftp_username,
                                    self._ftp_password,
                                    file2xfer,
                                    self._xfer_timeout)
        else:
            self._logger.info("FTP transfer %s during %d seconds for file %s ..."
                               % (str(direction), self._xfer_duration, str(file2xfer)))

            start = time.time()
            # repeat transfer until the duration has been reached
            while (start + self._xfer_timeout) > time.time():
                self._computer.ftp_xfer(direction,  # pylint: disable=E1101
                                    self._ftp_ip_address,
                                    self._ftp_username,
                                    self._ftp_password,
                                    file2xfer,
                                    self._xfer_timeout)

        # Stop USB tethering
        self._networking_api.stop_usb_tethering(unplug=True)

        return (Global.SUCCESS, "No error")

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiTetheringUsbBase.tear_down(self)

        # Restore default WIFI sleep policy if changed in setup
        if self._keep_wifi_on != "":
            self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["DEFAULT"])

        return (Global.SUCCESS, "No error")
