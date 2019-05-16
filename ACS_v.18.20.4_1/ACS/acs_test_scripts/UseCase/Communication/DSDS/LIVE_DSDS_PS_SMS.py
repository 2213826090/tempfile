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
:summary: Use Case Live DSDS scenario with simultaneous browsing and SMS
:since: 03/03/2016
:author: nowelchx
"""
import time
import os
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global,internal_shell_exec
from acs_test_scripts.Utilities.SmsUtilities import compute_sms_equals_dual_phone, SmsMessage
from Device.DeviceManager import DeviceManager
from threading import Thread,current_thread
import threading
import Queue
import traceback
from uiautomator import Device

class LiveDsdsFtpSms(UseCaseBase):

    """
    Use Case Live SMS loopback class.
    """

    def __init__(self, tc_name, global_config):

        UseCaseBase.__init__(self, tc_name, global_config)

        self.awake = False
        self._test_timeout=10
        self._call_setup_time = 15
        self.d = Device(self._device.retrieve_serial_number())
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        self._phone_number = str(self._device.get_phone_number())
        self._destination_number = \
            str(self._tc_parameters.get_param_value("DESTINATION_NUMBER"))
        self._message = \
            str(self._tc_parameters.get_param_value("MESSAGE_CONTENT", ""))
        self._phone = DeviceManager().get_device("PHONE1")

        self._phone2 = DeviceManager().get_device("PHONE2")

        self._phone_number2 = str(self._phone2.get_phone_number())
        self._default_sim = self._tc_parameters.get_param_value("DEFAULT_SIM")

        self._network_pref = self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE", None)
        if self._network_pref:
            self._network_pref = self._network_pref.upper()
        self._initial_pref_network = None
        #Parameter to download or upload
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        self._server_name = \
            self._tc_parameters.get_param_value("SERVER_NAME",
                                                "LAB_SERVER")
        self._callduration = \
             int(self._tc_parameters.get_param_value("CALL_DURATION"))
        self._sms_transfer_timeout= \
             int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        # Get FTP server parameters
        self._server = \
            global_config.benchConfig.get_parameters(self._server_name)
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        #The file need to download
        self._dlfilename = os.path.join(
            self._ftp_path,
            self._tc_parameters.get_param_value("DL_FILE", ""))
        #Max time for file transfer
        self._xfer_timeout = \
            int(self._tc_parameters.get_param_value("XFER_TIMEOUT"))

        # The base name of the file we download
        self._ftp_file_basename = self._tc_parameters.get_param_value("BASENAME")
        self._sms_api = self._device.get_uecmd("SmsMessaging")
        self._sms_api2 = self._phone2.get_uecmd("SmsMessaging")
        self._mms_api = self._device.get_uecmd("MmsMessaging")
        self._sms_api2 = self._phone2.get_uecmd("SmsMessaging")

        #self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        #self._phone_system_api2 = self._phone2.get_uecmd("PhoneSystem")
        self._ftp_api = self._device.get_uecmd("Ftp")
        self._modem_api = self._device.get_uecmd("Modem")
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        #self._phone_system_api2 = self._phone2.get_uecmd("PhoneSystem")

        self._networking_api = self._device.get_uecmd("Networking")
        #self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
        #self._networking_api2 = self._phone2.get_uecmd("Networking")
        self._voicecall_api = self._phone.get_uecmd("VoiceCall")

        if self._phone2 is not None:
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            self._phone_system_api2 = self._phone2.get_uecmd("PhoneSystem")
#--------------------------------------------------------------------------------------

    def set_up(self):
        """
        Setup for the test case
        """

        # Call set_up of use case base
        UseCaseBase.set_up(self)

        # Check if we have the second phone available
        if self._phone2 is None:
            # We are using this multi UC with only one phone
            error_msg = \
                "This use case requires two phones to be executed !"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)
        self._sms_api.delete_all_sms()
        # Boot the other phone (the DUT is already booted)
        if not self._phone2.is_available():
            DeviceManager().boot_device("PHONE2")
        if self._default_sim == "SIM2":
            #Setting the sim to Default
            (verdict,msg)=self._setDefaultSecondSim()
        else:
            (verdict,msg)=self._setDefaultSim()

        time.sleep(10)
        self._absolute_path_on_dut = "".join([
            self._device.multimedia_path,
            self._ftp_file_basename])

        # Delete the FTP download file on the DUT
        self._delete_file_if_exists(self._absolute_path_on_dut)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        time.sleep(self._wait_btwn_cmd)
        time.sleep(self._wait_btwn_cmd)

        # There is a preferred network to set, backup initial, then set configured one
        if self._network_pref is not None:
            self._initial_pref_network = self._dut_config.get("defaultPreferredNetwork")
            time.sleep(self._wait_btwn_cmd)

            if self._networking_api.is_preferred_network_type_valid(self._network_pref):
                # Setting the DUT preferred network type to the one specified
                # in the TC.
                self._networking_api.set_preferred_network_type(self._network_pref)
                time.sleep(self._wait_btwn_cmd)

                # Check the DUT is camped on a compatible network with the selected
                # preferred network.
                self._modem_api.check_rat_with_pref_network(self._network_pref, self._registration_timeout)
                time.sleep(self._wait_btwn_cmd)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Unknown network type: %s" % self._network_pref)

        return verdict,msg
#--------------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        #Call UseCase base run function
        UseCaseBase.run_test(self)

        queue = Queue.Queue()
        #creating empty list to append the threads
        threads=[]
        #creating empty list to append the return value from the threads
        result=[]
        try:
            #creating thread object to trigger to the _ftpdownloads method
            thread_ftp=Thread(target = self._ftpdownload,args=[queue])
            thread_ftp.setName("Thread1")
            #to start the thread object
            thread_ftp.start()
            time.sleep(10)
            #creating thread object to trigger to the _smssendreceive method
            thread_sms=Thread(target = self._sendsms,args=[queue])
            thread_sms.setName("Thread2")
            #to start the thread object
            thread_sms.start()
        except Exception as inst:
            self._logger.info(inst)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         "Exception: %s" % inst)
            return Global.FAILURE,"Error"

        #appending the thread object into the empty list
        threads.append(thread_ftp)
        #appending the thread object into the empty list
        threads.append(thread_sms)
        for thread_id in threads:
            #to make the thread wait for the other thread does not complete
            thread_id.join()
            #appending the return value to the list
            result.append(queue.get())
        #checking the return value is SUCCESS
        if result[0]==Global.SUCCESS:
            self._logger("successfull")
            if result[1]==Global.SUCCESS:
                self._logger("succssfully")
                return Global.SUCCESS, "SUCCESSFULL"
            else:
                return Global.FAILURE," not send done."
        else:
            return Global.FAILURE,"not done"

#---------------------------------------------------------------------------------------------
    def _ftpdownload(self,queue):
        """
        Execute the web browsing
        """
        #creating the lock
        lock=threading.Lock()
        lock.acquire()
        try:
            self._logger.info("inside _ftpdownload function")
            #Thread no
            self._logger.info(current_thread())
            #checking the pdp status
            pdp_context_status = self._networking_api._get_pdp_context_status()
            if pdp_context_status in ("0", "2"):
                self._networking_api.activate_pdp_context()
            time.sleep(self._wait_btwn_cmd)
            if self._direction == "DL":
                self._logger.info("FTP transfer " +
                                  str(self._direction) +
                                  " for " + str(self._dlfilename) + "...")
                # Uecmd to do ftp and storing the result
                result = self._networking_api.\
                    ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                             self._server_ip_address,
                             self._username,
                             self._password,
                             self._dlfilename,
                             self._xfer_timeout,
                             self._device.multimedia_path)
            else:
                self._error.Code =Global.FAILURE
                self._error.Msg = "%s is not a known xfer direction" % self._direction
                self._logger.info(self._error.Msg)
                queue.put(self._error.Code)
        except Exception as inst:
            self._error.Code = Global.FAILURE
            queue.put(self._error.Code)
            self._logger.info(inst)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         "Exception : %s" % inst)
        finally:
            #lock is released
            lock.release()
            self.awake = True
        #giving the return value into queue
        queue.put(Global.SUCCESS)
#---------------------------------------------------------------------------------------------
    def _sendsms(self,queue):
        lock = threading.Lock()
        lock.acquire()
        try:
            result_verdict=Global.FAILURE
            self._logger.info("inside the lock of msg")
            #thread no
            self._logger.info(current_thread())
            #to delete all sms
            self._sms_api.delete_all_sms()
            self._sms_api2.delete_all_sms()
            time.sleep(5)
            sms_sent = SmsMessage(self._message, self._destination_number)
        # register on intent to receive incoming sms
            self._sms_api.register_for_sms_reception()
        # Send SMS to equipment using SMS parameters :
        # - SMS_TEXT
        # - DESTINATION_NUMBER
            self._sms_api2.send_sms(self._destination_number,self._message)
        # Get received sms
            sms_received = self._sms_api.wait_for_incoming_sms(self._sms_transfer_timeout)
        # Compare sent and received SMS (Text,
        # Destination number)
            self._logger.info(sms_received)
            (result_verdict, result_message) = \
                compute_sms_equals_dual_phone(sms_sent, sms_received)
            self._logger.info(result_message)

            queue.put(result_verdict)
        except Exception as inst:
            self._logger.info(inst)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         "Exception: %s" % inst)
        finally:
            #release of the lock
            lock.release()
            print(self.awake)
            if self.awake==True:
                result_verdict=Global.FAILURE
            #giving the return value into queue
            queue.put(result_verdict)
#--------------------------------------------------------------------------------------------------
    def _setDefaultSim(self):

        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        time.sleep(8)
        #To pop up the dialog box for selecting the sim
        statusintent=self._voicecall_api.get_sim_select(dialog=3,sim=0)
        self._logger.info(statusintent)
        #To check dailog is reached or not
        if statusintent == {}:
            err_msg=" The option dialog is not reached"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        #Waiting for the dailog box
        if self.d(resourceId='android:id/title_template').wait.exists(
                    timeout=5000):
            #clicking the default sim
            self.d(resourceId='android:id/button1').click()
            self._logger.info("Default sim is selected")
            return Global.SUCCESS,"Default sim is selected"
        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg


#-------------------------------------------------------------------------------------------------
    def _setDefaultSecondSim(self):

        self._device.run_cmd("adb shell input keyevent 82",self._test_timeout,
                                                  force_execution=True)
        time.sleep(8)
        #To pop up the dialog box for selecting the sim
        statusintent=self._voicecall_api.get_sim_select(dialog=3,sim=1)
        self._logger.info(statusintent)
        #To check dailog is reached or not
        if statusintent == {}:
            err_msg=" The option dialog is not reached"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg
        #Waiting for the dailog box
        if self.d(resourceId='android:id/title_template').wait.exists(
                    timeout=5000):
            #clicking the default sim
            self.d(resourceId='android:id/button1').click()
            self._logger.info("Default sim is selected")
            return Global.SUCCESS,"Default sim is selected"
        else:
            err_msg="Dialog is not opening"
            raise DeviceException(DeviceException.OPERATION_SET_ERROR, err_msg)
            return Global.FAILURE,err_msg

#-------------------------------------------------------------------------------------------------
    def _delete_file_if_exists(self, file_path, fail_on_error=True):
        """
        Deletes the given file on DUT if it exists.

        :type file_path: str
        :param file_path: the absolute path of the file to delete.

        :type fail_on_error: bool
        :param fail_on_error: [optional] a boolean indicating whether
            we want to raise an exception on error or not.
            Defaults to C{True}.
        """
        # Force file path to a str value
        file_path = str(file_path)
        rm_cmd = "adb shell rm %s" % file_path
        # We go on only if the file exists on the DUT
        if not self._phone_system_api.check_file_exist_from_shell(file_path):
            self._logger.debug("No such file to delete.")
            return
        # We try to delete the file
        try:
            # Run the command to remove the file, we give it 10 seconds to run
            self._logger.debug("Deleting file %s." % file_path)
            (exit_status, _output) = internal_shell_exec(rm_cmd, 10)
        except (KeyboardInterrupt, SystemExit):
            raise
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            exit_status = Global.FAILURE
            traceback.print_exc()

        # Check the status of the command execution
        if exit_status != Global.SUCCESS:
            # Build an error message
            error_message = "Command execution failed (command: '%s')." % rm_cmd
            # Handle the error case as expected
            if fail_on_error:
                # If we have to fail, raise an exception
                raise DeviceException(DeviceException.OPERATION_FAILED, error_message)
            else:
                # Otherwise simply log a warning
                self._logger.warning(error_message)

        # We double-check that the file has been deleted
        if self._phone_system_api.check_file_exist_from_shell(file_path):
            # Build an error message
            error_message = "File deletion failed (file: '%s')." % file_path
            # Handle the error case as expected
            if fail_on_error:
                # If we have to fail, raise an exception
                raise DeviceException(DeviceException.OPERATION_FAILED, error_message)
            else:
                # Otherwise simply log a warning
                self._logger.warning(error_message)
        else:
            self._logger.info("File %s deleted successfully." % file_path)