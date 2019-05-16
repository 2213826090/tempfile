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
:summary: Use Case LAB MMS to perform MO / MT MMS over Network simulator
(for 2G / 3G RAT)
:since: 19/06/2013
:author: lvacheyx
"""
import os
import time

from UtilitiesFWK.Utilities import Global
from LAB_MMS_BASE import LabMmsBase
from acs_test_scripts.Utilities.MmsUtilities import MmsMessage
from acs_test_scripts.Equipment.NowSmsServer.NowSmsServer import NowSmsServer
from ErrorHandling.DeviceException import DeviceException


class LabMms(LabMmsBase):

    """
    Constructor
    """

    def __init__(self, tc_name, global_config):
        """
        Call base class initialization
        """
        # Call LabMmsBase __init__ function
        LabMmsBase.__init__(self, tc_name, global_config)

        # Retrieve the SENDER_NUMBER parameter, set default to "8960".
        self._sender_number = str(self._tc_parameters.get_param_value("MMS_SENDER_NUMBER", "8960"))

        # Retrieve the DESTINATION_NUMBER parameter.
        # check if not already defined in Now SMS server apn
        if self._destination_number in (None, ""):
            # Retrieve the DESTINATION_NUMBER parameter from test case.
            self._destination_number = str(self._tc_parameters.get_param_value("MMS_DESTINATION_NUMBER"))
            self._logger.info("Use MMS Destination number from test case file (%s)." % self._destination_number)
        else:
            self._logger.info("Use MMS Destination number from bench config file (%s)." % self._destination_number)

        if self._destination_number.upper() == "[PHONE_NUMBER]":
            self._destination_number = str(self._device.get_phone_number())

        self._destination_number = str(self._destination_number.strip())

        # Retrieve the DIRECTION parameter.
        self._direction = str(self._tc_parameters.get_param_value("MMS_DIRECTION"))

        # Retrieve the SUBJECT parameter.
        self._subject = str(self._tc_parameters.get_param_value("MMS_SUBJECT"))

        # Retrieve the TEXT parameter.
        self._text = str(self._tc_parameters.get_param_value("MMS_TEXT"))

        # Retrieve the MMS_TYPE parameter.
        self._media = str(self._tc_parameters.get_param_value("MMS_MEDIA"))

        # Retrieve the ATTACHMENT parameter.
        self._attachment_name = str(self._tc_parameters.get_param_value("MMS_ATTACHMENT_NAME"))
        if self._attachment_name in (None, "None"):
            self._attachment_name = None

        if "MO" in self._direction:
            # Send MO MMS from the DUT
            # Retrieve the attachment path from the DUT.
            self._attachment_path = str(self._dut_config.get("userdataPath"))
        else:
            # Send MT MMS from Now SMS Server
            # Retrieve the attachment path from ACS.
            self._attachment_path = str(os.path.join(os.getcwd(), "_Embedded", "USERDATA"))

        # Retrieve the MMS Timeout parameter.
        self._timeout = self._tc_parameters.get_param_value("MMS_TIMEOUT", 0, int)

        # Instantiate NowSmsServer class
        self._nowsms_server = NowSmsServer(self._nowsms_url, self._user_name, self._user_password)

        # Create mms object
        self._mms = MmsMessage(self._sender_number,
                              self._destination_number,
                              self._direction,
                              self._subject,
                              self._text,
                              self._media,
                              self._attachment_name,
                              self._attachment_path)

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Setting up the test
        """
        # Call LabMmsBase set_up function
        LabMmsBase.set_up(self)

        # Kill MMS application and checks if stock messaging
        # application is still running.
        self._mms_messaging.kill_mms_app()

        LabMmsBase.check_mms_attachment(self, self._mms)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        verdict = Global.SUCCESS
        output_msg = ""

        # Call LabMmsBase set_up function
        LabMmsBase.run_test(self)

        try:
            # Clear all SMS and MMS.
            self._mms_messaging.delete_all_messages()

            # Go out of coverage before sending MMS, if necessary
            if self._move_out_of_coverage == 1:
                self._logger.info("Go to out of coverage before sending MMS")
                verdict, output = self._go_out_of_coverage()
                output_msg += output
                if verdict == Global.FAILURE:
                    return verdict, output_msg

            # Send MO or MT MMS and get phone time when
            # MMS is successfully sent
            self.send_mms(self._mms, self._timeout)

            # Restore coverage after the delay
            if self._move_out_of_coverage == 1:
                # Wait during out_of_coverage_timer
                self._logger.info("Wait for %d seconds before go back to coverage." % self._out_of_coverage_timer)
                time.sleep(self._out_of_coverage_timer)
                # Restore cell power
                self._logger.info("Restore cell power to %d dBm." % self._cell_power)
                self._ns_cell.set_cell_power(self._cell_power)
                # Check DUT is register
                self._check_network_registration()

            # Check that MO or MT MMS have been received
            if self._move_out_of_coverage == 2:
                # Wait for MMS reception notification
                self._mms_messaging.wait_for_mms_notification(self._timeout)

                # Go out of coverage just after MMS notification
                self._logger.info("Go to out of coverage before sending MMS")
                verdict, output = self._go_out_of_coverage()
                if verdict == Global.FAILURE:
                    raise DeviceException(DeviceException.CRITICAL_FAILURE, output)

                # Wait during out_of_coverage_timer
                self._logger.info("Wait for %d seconds before go back to coverage." % self._out_of_coverage_timer)
                time.sleep(self._out_of_coverage_timer)

                # Restore cell power
                self._logger.info("Restore cell power to %d dBm." % self._cell_power)
                self._ns_cell.set_cell_power(self._cell_power)

                # Check DUT is register
                self._check_network_registration()

                # Wait for MMS reception download
                self._mms_messaging.wait_for_mms_download(self._timeout)
            else:
                self.read_mms(self._timeout)

        except Exception as exception:
            verdict = Global.FAILURE
            output = "Error: %s. " % str(exception)
            self._logger.error(output)
            output_msg += output

        return verdict, output_msg

    def tear_down(self):
        """
        Tear down
        """
        # Delete all MMS and SMS.
        self._mms_messaging.delete_all_messages()

        # Killing the MMS application.
        self._mms_messaging.kill_mms_app()

        # Call LabMmsBase tear_down function
        LabMmsBase.tear_down(self)

        return Global.SUCCESS, "No error"

    def send_mms(self, mms, timeout):
        """
        Sends the MMS using the direction MO or MT
        """
        if mms._direction == "MO":
            # Read number of MMS present in the server
            self._nowsms_server.read_mms_number()
            # Wake up the phone screen.
            self._logger.info("Turning the screen on.")
            self._phonesystem_api.set_phone_lock("off")
            self._phonesystem_api.wake_screen()
            self._phonesystem_api.set_phone_lock(0)

            # Send MO MMS from the DUT
            self._mms_messaging.send_mms(mms._media,
                                         mms._destination_number,
                                         mms._subject,
                                         mms._text,
                                         mms._attachment_file)

            self._mms_messaging.check_mms_sent(mms._destination_number, timeout)
        else:
            # Send MT MMS from Now SMS Server
            self._nowsms_server.send(mms._media,
                                     mms._sender_number,
                                     mms._destination_number,
                                     mms._subject,
                                     mms._text,
                                     mms._attachment_file)

    def read_mms(self, timeout):
        """
        Read the MMS using the direction MO or MT

        """
        if self._direction == "MO":
            # Check that MO MMS has been received on the server
            self._nowsms_server.read(timeout)
        else:
            # Wait for MMS reception notification
            self._mms_messaging.wait_for_mms_notification(timeout)
            # Wait for MMS reception download
            self._mms_messaging.wait_for_mms_download(timeout)
