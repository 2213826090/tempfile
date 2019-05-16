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
:summary:  This file implements usecase that do a ftp transfer and a voice call, while DTM is set to ON on 2G CEll
UC for testing ACS framework
:since: 05/04/2012
:author: Lvacheyx
"""


import time
from UtilitiesFWK.Utilities import Global
from LAB_FIT_TEL_EGPRS_FTP_VC_BASE import LabFitTelEgprsftpVcBase
from ErrorHandling.AcsConfigException import AcsConfigException


class LabFitTelEgprsftpVcDtmOn(LabFitTelEgprsftpVcBase):

    """
    Lab 2G DTM ON FTP and Voice call both active
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        self._ftp_direction = None
        self._is_phone_number_checked = False
        self._ftp_data_transfer_state_timeout = 10

        # Call LabMobilityBase Init function
        LabFitTelEgprsftpVcBase.__init__(self, tc_name, global_config)

        # Read PHONE_NUMBER from testcase xml parameters
        if self._tc_parameters.get_param_value("PHONE_NUMBER") not in (None, ''):
            self._is_phone_number_checked = True
            if str(self._tc_parameters.get_param_value("PHONE_NUMBER")).isdigit():
                self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")

            elif self._tc_parameters.get_param_value("PHONE_NUMBER") == "[PHONE_NUMBER]":
                self._phone_number = str(self._device.get_phone_number())
            else:
                self._phone_number = None
        else:
            self._phone_number = None

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabFitTelEgprsftpVcBase Run function
        LabFitTelEgprsftpVcBase.run_test(self)

        # Start an ftp tranfer
        self._ftp_task_id = \
            self._networking_api.start_ftp_xfer(
                self._ftp_direction,
                self._server_ip_address,
                self._username,
                self._password,
                self._ftp_filename,
                self._device.get_ftpdir_path()
            )

        # wait 20 seconds for ensure that transfer is established
        self._logger.info(
            "Wait 20 seconds for ensure that transfer is established")
        time.sleep(20)

        # Check that operator gave a valid Voice call number
        # Perform MO voice call on active network simulator
        if self._is_phone_number_checked:
            if self._phone_number is None:
                self._logger.warning("Operator Phone Number cannot be used to perform a voice call \
                             due to invalid test parameter value (Phone Number %s)"
                                     % (str(self._tc_parameters.get_param_value("PHONE_NUMBER"))))

            else:
                self._voicecall_api.dial(self._phone_number)

        else:
            # Raise an error message as no valid phone number has been set by the operator
            self._error.Msg = "Phone number has no valid value (%s) so voice call can not be performed" % \
                              self._phone_number
            self._logger.error(self._error.Msg)
            raise AcsConfigException(AcsConfigException.PROHIBITIVE_BEHAVIOR, self._error.Msg)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_2g.check_call_connected(self._call_setup_timeout)

        # Check data state "TRANSFERRING" before 10 seconds
        self._ns_data_2g.check_data_connection_transferring(self._ftp_data_transfer_state_timeout, True)

        # Check call is connected for CALL_DURATION seconds
        self._ns_voice_call_2g.is_voice_call_connected(self._call_duration)

        # Release the voice call
        self._ns_voice_call_2g.voice_call_network_release()

        # Check data state "TRANSFERRING" before 10 seconds
        self._ns_data_2g.check_data_connection_transferring(self._ftp_data_transfer_state_timeout, True)

        # Stop ftp transfer
        self._networking_api.stop_ftp_xfer(self._ftp_task_id)

        return Global.SUCCESS, "No errors"
