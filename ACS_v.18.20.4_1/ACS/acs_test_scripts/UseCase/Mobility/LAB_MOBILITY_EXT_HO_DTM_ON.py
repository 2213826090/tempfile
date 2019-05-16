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
:summary:  This file implements usecase that do a mobility handover
from a 3G cell to a 2G cell, during ftp transfer, while DTM is set to ON on 2G CEll
:since: 20/03/2012
:author: Lvacheyx
"""
import os
import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabMobilityExtHoDtmOn(LabMobility3gsmBase):

    """
    Mobility handover while ftp transfer and voice call are both active
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        self._ftp_direction = None
        self._ftp_filename = None
        self._is_phone_number_checked = False
        self._ftp_task_id = 0
        self._check_data_transfer_state_timeout = 10
        self._check_voice_call_state_timeout = 10

        # Call LabMobility3gsmBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)

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

        # Read ftp DIRECTION from testcase xml parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")

        # Get FTP server parameters
        self._server = global_config.benchConfig.get_parameters("LAB_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._server_username = self._server.get_param_value("username")
        self._server_password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        # Read the DL_FILE value from UseCase xml Parameter
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILENAME"))
        # Read the UL_FILE value from UseCase xml Parameter
        self._ulfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("UL_FILENAME"))

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LabMobility3gsmBase Setup function
        LabMobility3gsmBase.set_up(self)

        # IF DIRECTION is UL starting upload FTP
        #    Start ftp upload of "put100M" file using  following parameters :
        #        DIRECTION, IP, username, password
        if self._direction == "UL":
            self._ftp_direction = str(self._uecmd_types.XFER_DIRECTIONS.UL)  # pylint: disable=E1101
            self._ftp_filename = self._ulfilename

        # ELIF DIRECTION is DL starting download FTP
        #    Start ftp download of "get500M" file using  following parameters :
        #        DIRECTION, IP, username, password
        elif self._direction == "DL":
            self._ftp_direction = str(self._uecmd_types.XFER_DIRECTIONS.DL)  # pylint: disable=E1101
            self._ftp_filename = self._dlfilename

        # ELSE raise an error
        # Raise an error in case the direction is not known
        else:
            self._logger.error("Unknown ftp direction (%s)"
                               % self._direction)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "Unknown ftp direction.")

        # Set cell on
        self._ns1_cell.set_cell_on()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LabMobility3gsmBase Run function
        LabMobility3gsmBase.run_test(self)

        # Initialize local variables used for Handovers
        ns1_vc = self._ns1_vc
        ns2_vc = self._ns2_vc
        nb_success_hand = 0
        ns1_cell = self._ns1_cell
        ns1_data = self._ns1_data
        ns2_cell = self._ns2_cell
        ns2_data = self._ns2_data

        # Check Data Connection State => PDP_ACTIVE before timeout
        RegUtil.check_dut_data_connection_state_before_timeout("PDP_ACTIVE",
                                                               ns1_cell,
                                                               self._networking_api,
                                                               self._logger,
                                                               self._registration_timeout,
                                                               flightmode_cycle=True,
                                                               blocking=False)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(
            self._registration_timeout)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(ns1_data.get_network_type(),
                                                          self._registration_timeout)

        # Set cell on
        ns2_cell.set_cell_on()

        # Perform try catch in order to catch errors during the for iteration.
        # If this try catch isn't done here and a crash appears during the for
        # iteration, the Tear Down will be called without recording Use Case
        # verdict and maybe 1 or more succeeded handovers.
        try:
            # For iteration 0 to HANDOVER_NUMBER:
            while nb_success_hand != self._jump_num:

                # Start an ftp tranfer
                self._ftp_task_id = \
                    self._networking_api.start_ftp_xfer(
                        self._ftp_direction,
                        self._server_ip_address,
                        self._server_username,
                        self._server_password,
                        self._ftp_filename,
                        self._device.get_ftpdir_path())

                # wait 20 seconds for ensure that transfer is established
                self._logger.info(
                    "Wait 20 seconds for ensure that transfer is established")
                time.sleep(20)

                # Check that operator gave a valid Voice call number
                # Perform MO voice call on NS1
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
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

                # Check call state "CONNECTED" before callSetupTimeout seconds
                ns1_vc.check_call_connected(self._call_setup_time,
                                            blocking=False)

                # Check data state "TRANSFERRING" before timeout
                ns1_data.check_data_connection_transferring(self._check_data_transfer_state_timeout)

                # Perform handover using 60 seconds timeout
                ns1_cell.execute_external_handover()
                self._logger.info("Wait 60 seconds for ensure that" +
                                  " external handover is done")
                time.sleep(60)

                # Switch NS1 and NS2 network simulators
                # (when the handover finishes, the roles are reversed)
                tmp_cell = ns1_cell
                tmp_data = ns1_data
                ns1_cell = ns2_cell
                ns1_data = ns2_data
                ns2_cell = tmp_cell
                ns2_data = tmp_data
                tmp_vc = ns1_vc
                ns1_vc = ns2_vc
                ns2_vc = tmp_vc

                # Check data state "TRANSFERRING" before timeout
                ns1_data.check_data_connection_transferring(self._check_data_transfer_state_timeout,
                                                            False,
                                                            blocking=False)

                # Check call state "CONNECTED" before timeout to validate handover
                ns1_vc.check_call_connected(self._check_voice_call_state_timeout,
                                            blocking=False)

                # Get RAT from Equipment
                network_type = ns1_data.get_network_type()

                # Check that DUT is registered on the good RAT
                self._modem_api.check_network_type_before_timeout(network_type,
                                                                  self._registration_timeout)

                # Increment number of succeeded handovers
                nb_success_hand += 1

                # Stop ftp transfer
                self._networking_api.stop_ftp_xfer(self._ftp_task_id)

                # Release the voice call
                ns1_vc.voice_call_network_release()

            # End For

        # Catch a possible exception
        except TestEquipmentException as ex:
            msg = "The handover number %d failed " % (nb_success_hand + 1)
            msg += "(%d succeeded handovers on %d)." \
                % (nb_success_hand, self._jump_num)

            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "Exception during handover process: %s (%s)"
                                   % (ex.get_error_message(), msg))

        except AcsBaseException as ex:
            # if nb_success_hand < self._jump_num
            # raise an exception
            # else only log the error
            if nb_success_hand < self._jump_num:
                msg = "The handover number %d failed " % (nb_success_hand + 1)
                msg += "(%d succeeded handovers on %d)." \
                    % (nb_success_hand, self._jump_num)
                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR,
                                       "Exception during handover due to board: %s (%s)"
                                       % (ex.get_error_message(), msg))
            else:
                # log error as a warning, but no Usecase exception
                # is raised because the purpose of the Usecase has been reached
                self._logger.warning(ex.get_error_message())
        finally:
            # Stop ftp tranfer if an error occurs
            self._networking_api.stop_ftp_xfer(self._ftp_task_id)

        # Compute final verdict
        msg = "%d handovers done." % nb_success_hand
        return Global.SUCCESS, msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Stop ftp transfer
        self._networking_api.stop_ftp_xfer(self._ftp_task_id)

        # Release the voice call
        self._ns1_cell.voice_call_network_release()

        # Deactivate PDP context
        self._networking_api.deactivate_pdp_context(self._ssid)

        # Call LabMobility3gsmBase TearDown function
        LabMobility3gsmBase.tear_down(self)

        return Global.SUCCESS, "No errors"
