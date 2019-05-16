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
:summary: This file implements the Modem NVM READBACK with DUT in POS Use Case
:since: 06/09/2012
:author: asebbanx
"""

import serial
import time

from UtilitiesFWK.Utilities import Global, internal_shell_exec
from acs_test_scripts.UseCase.System.LAB_MODEM_NVM_READBACK_BASE import LabModemNvmReadbackBase
from ErrorHandling.AcsBaseException import AcsBaseException


class LabModemNvmReadbackPos(LabModemNvmReadbackBase):

    """
    Lab Modem NVM READBACK with DUT in I{POS} (Provisioning OS) Use Case Class.
    It is assumed that the DUT is already in I{POS} when this Use Case is executed.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call inherited base init method
        LabModemNvmReadbackBase.__init__(self, tc_name, global_config)

    def set_up(self):
        """
        Initialize the test
        """
        # Run inherited set_up method
        LabModemNvmReadbackBase.set_up(self)

        # Check that the phone is in POS
        # Check the state of the phone as returned by adb
        # We do not want to use the 'get_state' method
        # of the device as it adds some post processing
        # on the state value
        (_exit_status, output) = internal_shell_exec("adb get-state", 5)  # pylint: disable=W0212
        state = output.strip()

        self._logger.debug("Initial phone state '%s'" % state)

        if state not in ("bootloader", "unknown"):
            message = "Unexpected phone state: %s. expected 'bootloader' or 'unknown'." % state
            self._logger.error(message)
            return Global.FAILURE, message

        # Start AT proxy in tunneling mode
        self._logger.debug("Starting AT proxy")
        try:
            self._at_proxy_tty = self._modem_flashing_api.start_at_proxy_from_pos()
        except AcsBaseException as ex:
            message = ex.get_error_message()
            error_code = ex.get_errorcode()
            self._logger.error(message)
            return error_code, message

        # Open the tty to talk to the modem
        self._logger.debug("Opening serial port to %s" % self._at_proxy_tty)
        self._at_proxy_serial = serial.Serial(
            port=self._at_proxy_tty,
            baudrate=self._at_proxy_baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1)
        self._at_proxy_serial.open()

        # Return the status
        return Global.SUCCESS, "No error."

    def tear_down(self):
        """
        End and dispose the test
        """
        # Run inherited tear_down method
        LabModemNvmReadbackBase.tear_down(self)

        # Close the tty
        self._logger.debug("Closing serial port (%s)" % self._at_proxy_tty)
        self._at_proxy_serial.close()

        # Stop AT proxy
        try:
            self._modem_flashing_api.stop_at_proxy_from_pos()
            time.sleep(3)
        except AcsBaseException as ex:
            message = ex.get_error_message()
            self._logger.error(message)

        # Return the status
        return Global.SUCCESS, "No error."
