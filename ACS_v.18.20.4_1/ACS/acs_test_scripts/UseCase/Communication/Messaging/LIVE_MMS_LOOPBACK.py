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
:summary: Use Case Live MMS loopback
:since: 02/10/2012
:author: rbertolx
"""

import time
import os
from UtilitiesFWK.Utilities import Global
from LIVE_MESSAGING_BASE import LiveMessagingBase
from ErrorHandling.DeviceException import DeviceException


class LiveMmsLoopback(LiveMessagingBase):
    """
    Use Case Live MMS loopback class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LiveMessagingBase init function.
        LiveMessagingBase.__init__(self, tc_name, global_config)

        # Retrieve the repeat count
        self._repeat_count = 0
        repeat_count = self._tc_parameters.get_param_value("REPEAT_COUNT","0")
        if repeat_count and repeat_count.isdigit():
            self._repeat_count = int(repeat_count)

        # Retrieve the mms_type parameters.
        self._mms_type = self._tc_parameters.get_param_value("MMS_TYPE")

        # Retrieve the MMS subject.
        self._mms_subject = self._tc_parameters.get_param_value("MMS_SUBJECT")

        if  self._mms_type == "text":
            self._attachment_file = None   
        else:
            self._multimedia_path = self._device.multimedia_path
            # Retrieve the path of the MMS attachment.
            self._attachment_file = os.path.join(
            self._multimedia_path,
            self._tc_parameters.get_param_value("ATTACHED_FILE"))

        # Retrieve the time that the test should wait to receive the MMS.
        self._send_mms_timeout = \
            int(self._tc_parameters.get_param_value("SENT_MMS_TIMEOUT"))

        self._received_mms_timeout = \
            int(self._tc_parameters.get_param_value("RECEIVED_MMS_TIMEOUT"))

        # Retrieve System API in order to wake up the phone screen.
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        self._system_api = self._device.get_uecmd("System")
        self._modem_api = self._device.get_uecmd("Modem")

        self._initial_pdp_context_status = None
        self._initial_sleep_timeout_value = None

    def set_up(self):
        """
        Setting up the test
        """
        # Call LiveMessagingBase set_up function.
        LiveMessagingBase.set_up(self)

        # Backup the initial pdp context status
        # pylint: disable=W0212
        self._initial_pdp_context_status = \
            self._networking_api._get_pdp_context_status()
        # pylint: enable=W0212
        # If deactivated, activate it
        if self._initial_pdp_context_status == "2":
            # Then deactivate PDP context status
            self._networking_api.activate_pdp_context(check=False)
            time.sleep(self._wait_btwn_cmd)

            # Check registration state is connected using
            # registrationTimeout from Device_Catalog.xml (Non blocking
            # for this test if function isn't implemented on CDK)
            self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # store initial Sleep Timeout value
        self._initial_sleep_timeout_value = self._phonesystem_api.get_screen_timeout()
        time.sleep(self._wait_btwn_cmd)

        # If the MMS type picture check if the file exist.
        if self._mms_type.lower() == "picture":
            # Checks if the attachment file exist.
            self._phonesystem_api.check_file_exist(self._attachment_file)

        # Kill the messaging application.
        self._kill_messaging_app()

        # set sleep timeout to 30 second
        self._phonesystem_api.set_screen_timeout(30)
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Execute the test
        """
        # Initializing some local variables.
        time_to_receive_mms = None
        time_to_send_mms = None
        time_waited = 0

        # Max number of iteration done while waiting for the screen to turn off.
        # Used to prevent endless loops.
        max_time_waited = 200

        # Call LiveMessagingBase run_test function.
        LiveMessagingBase.run_test(self)

        # Clear all SMS and MMS.
        self._mms_api.delete_all_messages()

        # Wait for the screen to turn off.
        while self._phonesystem_api.get_screen_status() \
            and time_waited < max_time_waited:
            time.sleep(5)
            time_waited += 5
        self._logger.info("Waited %s seconds for the screen to turn off."
                          % time_waited)
        # Checks if the screen turned off before a fixed timeout.
        if time_waited >= max_time_waited:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "The screen did not turned off in %s."
                                  % max_time_waited)

        # Wake up the phone screen.
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Turning the screen on.")
        self._phonesystem_api.set_phone_lock("off")
        self._phonesystem_api.wake_screen()
        self._phonesystem_api.set_phone_lock(0)

        # register on intent to receive incoming mms
        self._mms_api.register_for_mms_reception()

        # Build the MMS.
        time.sleep(self._wait_btwn_cmd)
        self._mms_api.send_mms(self._mms_type,
                                self._destination_number,
                                self._mms_subject,
                                self._message,
                                self._attachment_file,
                                self._repeat_count)

        time.sleep(self._wait_btwn_cmd)

        # Waiting for the MMS to be send.
        sent_time = self._mms_api.check_mms_sent(self._destination_number, self._send_mms_timeout)

        # Waiting on incoming message.
        reception_date = self._mms_api.wait_for_incoming_mms(self._received_mms_timeout,
                                                                self._destination_number)

        # Logging the time taken to receive the MMS.
        time_to_receive_mms = float(reception_date) - float(sent_time)

        self._logger.info("The MMS has been received in %s seconds."
                          % time_to_receive_mms)

        # Compare the sent and received MMS.
        self._mms_api.request_remote_send_received_mms_compare(self._mms_type)

        return (Global.SUCCESS, "MMS sent in %s secs and received in %s secs"
                                % (self._mms_api.time_to_send_mms, time_to_receive_mms))

    def tear_down(self):
        """
        End and dispose the test
        """
        # Call LiveMessagingBase tear_down function.
        LiveMessagingBase.tear_down(self)

        # Delete all MMS and SMS.
        self._mms_api.delete_all_messages()

        # Kill the messaging application.
        self._kill_messaging_app()

        # set initial sleep timeout
        self._phonesystem_api.set_screen_timeout(self._initial_sleep_timeout_value)
        time.sleep(self._wait_btwn_cmd)

        # Lock the screen.
        self._phonesystem_api.set_phone_lock("on")
        self._phonesystem_api.display_off()

        # If at start data was deactivated, go back to this state.
        if self._initial_pdp_context_status == "2":
            # Then deactivate PDP context status
            self._networking_api.deactivate_pdp_context()
            time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"

    def _kill_messaging_app(self):
        """
        Kills the MMS application if it was running and makes sure it has been
        killed successfully
        """
        # Get the pid of the MMS application
        pid = self._system_api.pid_of("com.android.mms")
        # Killing the MMS application, if running
        if pid != "":
            self._mms_api.kill_mms_app()
            # Checking if the MMS application as really been killed.
            new_pid = self._system_api.pid_of("com.android.mms")
            if new_pid == pid:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "The stock messaging application"
                                      " should not be running.")
        else:
            self._logger.info("The MMS application was not running.")
