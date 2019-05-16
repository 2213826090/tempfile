"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements the LIVE NFC UC
:since: 14/06/2012
:author: lpastor
"""

import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException


class LiveNfcBase(UseCaseBase):

    """
    Live NFC Test base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        # Get UECmdLayer
        self._nfc_api = self._device.get_uecmd("LocalConnectivity")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        self._previous_nfc_status = None
        self._is_nfc_beam_initial_state_on = None

        # Get "Beam control option" from test case xml file (set ON, set OFF or nothing)
        self._beam_used, self._beam_wished_value = self._get_beam_config()

        # Each raw measurement data & xml files are
        # stored in a folder bearing the name
        # of the TC + a time stamp
        directory_name = self._name + "_" + time.strftime("%Y-%m-%d_%Hh%M.%S")
        report_tree = global_config.campaignConfig.get("campaignReportTree")
        self._saving_directory = report_tree.create_subfolder(directory_name)

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Check current NFC status
        max_loop = 10
        self._previous_nfc_status = self._nfc_api.get_nfc_status()
        while (max_loop > 0) and ((self._previous_nfc_status == "TURNING_ON")
                                  or (self._previous_nfc_status == "TURNING_OFF")):
            time.sleep(0.50)
            max_loop -= 1
            self._previous_nfc_status = self._nfc_api.get_nfc_status()

        if (self._previous_nfc_status == "TURNING_ON") \
                or (self._previous_nfc_status == "TURNING_OFF"):
            msg = "Bad NFC Status " + self._previous_nfc_status
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Active NFC if not active
        if self._previous_nfc_status == "OFF":
            # NFC Disconnected
            self._nfc_api.nfc_enable()

        # If Beam to be (de-)activated
        if self._beam_used:
            # Store initial value
            self._is_nfc_beam_initial_state_on = self._nfc_api.get_nfc_beam_status()

            if self._beam_wished_value != self._is_nfc_beam_initial_state_on:
                # if Beam is not at the wished value, set it to the correct one.
                if self._beam_wished_value:
                    self._nfc_api.enable_nfc_beam()
                else:
                    self._nfc_api.disable_nfc_beam()

        self._phonesystem_api.display_on()
        self._phonesystem_api.set_phone_lock(0)
        self._nfc_api.force_nfc_state(1)

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # If Beam to be (de-)activated, reset it to initial value
        if self._beam_used:
            if self._nfc_api.get_nfc_beam_status() != self._is_nfc_beam_initial_state_on:
                if self._is_nfc_beam_initial_state_on:
                    self._nfc_api.enable_nfc_beam()
                else:
                    self._nfc_api.disable_nfc_beam()

        # Re establish the NFC status to the original status
        if self._previous_nfc_status == "ON":
            if self._nfc_api.get_nfc_status() == "OFF":
                self._nfc_api.nfc_enable()
        elif self._previous_nfc_status == "OFF":
            if self._nfc_api.get_nfc_status() == "ON":
                self._nfc_api.nfc_disable()

        self._phonesystem_api.display_off()
        self._phonesystem_api.set_phone_lock(1)
        self._nfc_api.force_nfc_state(0)

        return Global.SUCCESS, "No errors"

    def _get_beam_config(self):
        """
        Get NFC Beam control for NFC tests

        :rtype: list of 2 elements (boolean, boolean)
        :return: true if NFC Beam precondition is set,
        NFC Beam ON or OFF prior to the test to be r
        """

        # Read NFC_BEAM_INITIAL_STATE parameter from test case xml file
        param_nfc_beam_prerequisite = \
            str(self._tc_parameters.get_param_value("NFC_BEAM_INITIAL_STATE"))

        if param_nfc_beam_prerequisite.lower() in ["1", "on", "true", "yes"]:
            beam_used = True
            beam_wished_value = True
        elif param_nfc_beam_prerequisite.lower() in ["0", "off", "false", "no"]:
            beam_used = True
            beam_wished_value = False
        else:
            beam_used = False
            beam_wished_value = False

        return beam_used, beam_wished_value
#------------------------------------------------------------------------------

    def _get_beam_check_config(self):
        """
        Get NFC Beam check for NFC tests

        :rtype:boolean
        :return: NFC Beam Check ON or OFF
        """

        # Read NFC_BEAM_CHECK parameter from test case xml file
        param_nfc_beam_check = \
            str(self._tc_parameters.get_param_value("NFC_BEAM_CHECK"))

        if param_nfc_beam_check.lower() in ["1", "on", "true", "yes"]:
            beam_checked = True
        elif param_nfc_beam_check.lower() in ["0", "off", "false", "no"]:
            beam_checked = False
        else:
            beam_checked = False

        return beam_checked
#------------------------------------------------------------------------------
