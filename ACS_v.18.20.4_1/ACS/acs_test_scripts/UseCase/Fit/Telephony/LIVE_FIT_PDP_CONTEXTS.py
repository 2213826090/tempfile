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
:summary: This file implements the Live Fit PDP contexts TC which allows to
check that multiple concurrent data network connectivity opens different PDP
contexts
:since: 22/02/2013
:author: jduran4x
"""
# Module name follows ACS conventions
# pylint: disable=C0103

import time
from acs_test_scripts.UseCase.Networking.LIVE_CELLULAR_BASE import LiveCellularBase
from acs_test_scripts.UseCase.Communication.Messaging.LIVE_MMS_LOOPBACK import LiveMmsLoopback
from UtilitiesFWK.Utilities import Global, internal_shell_exec
import traceback
from ErrorHandling.DeviceException import DeviceException


class LiveFitPdpContexts(LiveCellularBase):

    """
    Live Fit Pdp Contexts.
    """

    FTP_TRANSFER_IN_PROGRESS_STATES = ("connecting", "connected", "transferring")
    """
    String value representing FTP transfer states that shall be considered
    as I{running}.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """
        LiveCellularBase.__init__(self, tc_name, global_config)
        self._mms_test_case = None

        self._use_ftp_dl = (str(self._tc_parameters.
                            get_param_value("USE_FTP_DOWNLOAD")).upper() == "TRUE")
        self._use_tethering = (str(self._tc_parameters.
                               get_param_value("USE_TETHERING")).upper() == "TRUE")
        # USE_MMS parameter is used to know
        # if MMS are used to create a PDP context (=True)
        # if MMS sending and reception have to be checked (=CHECK)
        use_mms = str(self._tc_parameters.get_param_value("USE_MMS")).upper()
        self._check_mms = (use_mms == "CHECK")
        self._use_mms = (use_mms == "TRUE" or self._check_mms)
        # need to retain the initial number of PDP contexts
        self._initial_pdp_number = 0
        if self._use_mms:
            self._logger.info("")
            self._logger.info("Initialize MMS loopback test")
            # initialize the MMS loopback part
            self._mms_test_case = LiveMmsLoopback(tc_name, global_config)

        # for later use, this Test Case can also be used for tethering
        # this should use the same formalism as mms

        # if FTP download is used for this test, gather the needed informations
        # from the config file
        if self._use_ftp_dl:
            self.__ftp_start_output = None
            self._ftp_file = str(self._tc_parameters.
                                 get_param_value("FTP_FILE"))

            self._ftp_user = str(self._tc_parameters.
                                 get_param_value("FTP_USER"))
            # the get_param_value output is casted to a str
            # so None will be "None"
            if self._ftp_user in ["None", ""]:
                self._logger.info("FTP user not indicated, using 'anonymous'")
                self._ftp_user = str("anonymous")

            # if ftp password is not indicated, empty str is returned
            self._ftp_passwd = str(self._tc_parameters.
                                   get_param_value("FTP_PASSWD"))
            # the get_param_value output is casted to a str
            # so None will be "None"
            if self._ftp_passwd in ["None", ""]:
                self._logger.info("FTP user's password not indicated, using 'none'")
                self._ftp_passwd = str("none")

        # Read the DL_FILE value from UseCase xml Parameter
        self._ftp_timeout = int(self._tc_parameters.
                                get_param_value("FTP_TIMEOUT"))

        self._target_number = int(self._tc_parameters.
                                  get_param_value("TARGET"))

        self._period = str(self._tc_parameters.
                           get_param_value("REFRESH_PERIOD"))
        # the get_param_value output is casted to a str
        # so None will be "None"
        if self._period in [None, "None", ""]:
            self._period = str(2)

        #
        # Some attributes needed tom add robustness to FTP operations
        #

        # The base name of the file we download
        self._ftp_file_basename = None
        # The absolute file path on the DUT
        self._absolute_path_on_dut = None
        # The server name or ip address
        self._server_ip = None
        # A boolean indicating whether we have some processing
        # to do on FTP paths or not (intended for back-to-back)
        self._compute_paths = True

        # Get UECmdLayer
        self._watcher_api = self._device.get_uecmd("Watcher")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        LiveCellularBase.set_up(self)

        time.sleep(self._wait_btwn_cmd)

        # Process some information for FTP if needed
        if self._use_ftp_dl and self._compute_paths:
            # remove the protocol information
            self._ftp_file = self._ftp_file.replace("http://", "")
            self._ftp_file = self._ftp_file.replace("ftp://", "")
            # split the given ftp file into required objects
            ftp_file_split = self._ftp_file.split("/")
            self._ftp_file = "/".join(ftp_file_split[1:])
            self._server_ip = ftp_file_split[0]
            self._ftp_file_basename = ftp_file_split[-1]
            # Compute the file name on DUT
            self._absolute_path_on_dut = "/".join([
                self._device.get_ftpdir_path(),
                self._ftp_file_basename])
            # Say that we won't have to do this anymore in future
            # iterations.
            self._compute_paths = False

        # Clean any previous FTP artifacts to avoid side effects
        if self._use_ftp_dl:
            # Now we stop any previous still running
            # FTP transfer if any
            self._stop_ftp_transfer()
            # Delete the FTP dowloaded file on the DUT
            self._delete_file_if_exists(self._absolute_path_on_dut)

        if self._use_mms:
            self._logger.info("set up MMS loopback test")
            # if mms is not used, the object doesn't even exist
            try:
                # need to encapsulate MMS_LOOPBACK set_up
                # cause it verifies if MMS stock app is running.
                # if it does, it failes
                # but this has no incidence for our test
                self._mms_test_case.set_up()
            except (KeyboardInterrupt, SystemExit):
                raise
            # We allow broad-except here
            # pylint: disable=W0703
            except Exception as e:
                # if MMS sending is checked, this is prohibitive
                if self._check_mms:
                    raise
                else:
                    self._logger.warning(e)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        self._error.Code = Global.FAILURE
        self._error.Msg = "no error"

        LiveCellularBase.run_test(self)

        # start Pdp context watcher
        self._watcher_api.start_watch_pdp_contexts(int(self._period))

        # wait the watcher check period time
        time.sleep(float(self._period))
        # and retain the initial number of PDP contexts
        # _initial_pdp_number = n x <spare interfaces> + 1 x <active interface>
        self._initial_pdp_number = self._watcher_api.get_pdp_contexts_number()

        # if FTP DL is used, need to transform the provided configuration
        # values into used function inputs.
        # the configuration is provided through the variable FTP_FILE
        # The expected content of FPT_FILE is
        # [http://|ftp://]<server_ip>/<ftp_file>
        # userfriendly but not exactly what needed
        # for the start_ftp_xfer function server_ip and ftp_file are
        # required separately.
        if self._use_ftp_dl:
            self._logger.info("using FTP download")
            self.__ftp_start_output = self._networking_api.\
                start_ftp_xfer(self._uecmd_types.XFER_DIRECTIONS.DL,  # pylint: disable=E1101
                               self._server_ip,
                               self._ftp_user,
                               self._ftp_passwd,
                               self._ftp_file,
                               self._device.get_ftpdir_path(),
                               self._ftp_timeout)
            time.sleep(self._wait_btwn_cmd)
            # waiting for FTP Xfer start success
            end_time = time.time() + float(self._ftp_timeout)
            self._logger.info("Waiting until beginning of FTP transfer...")
            while time.time() <= end_time:
                status = self._networking_api.get_ftp_xfer_status()
                if status == "transferring":
                    break
                time.sleep(1)
            else:
                self._logger.error("FTP transfer beginning attempt has timed out")

        # if MMS is used for the test just use the provided MMS LOOPBACK test
        # as the only interesting fact is that MMS creates PDP context
        # (not emission/reception status), don't escaladate potential errors
        # if emission has not worked, PDP context wouldn't have been created
        # and this test case will fail.
        if self._use_mms:
            self._logger.info("Running MMS loopback test")
            # if mms is not used, the object doesn't even exist
            try:
                (mms_code, mms_msg) = self._mms_test_case.run_test()
                mms_msg += "\n"
            except (KeyboardInterrupt, SystemExit):
                raise
            # We allow bare-except here
            # pylint: disable=W0702
            except:
                # if MMS sending is checked, this is prohibitive
                if self._check_mms:
                    raise
                else:
                    self._logger.error("Running MMS loopback test failed")
            finally:
                # 'pdp_number' is never empty or null.
                # its value comes from an integer and is get by
                # _internal_exec. So potential error are kept at this level
                pdp_number = self._watcher_api.get_pdp_contexts_number()
                self._watcher_api.stop_watch_pdp_contexts()

        if not self._check_mms:
            # if mms sending is not checked,
            # we are not interested in mms results,
            mms_code = Global.SUCCESS
            mms_msg = ""
        # the pdp number we are interested in
        # is the number of simultaneous connections
        # excluding the spare interfaces
        pdp_number = pdp_number - self._initial_pdp_number + 1
        if pdp_number != self._target_number:
            self._error.Msg = mms_msg + "get %i simultaneous PDP contexts instead of %i" \
                % (pdp_number, self._target_number)
        else:
            self._logger.info("set SUCCESS status")
            self._error.Msg = mms_msg + "get %i simultaneous PDP contexts" % pdp_number
            self._error.Code = mms_code
        return self._error.Code, self._error.Msg

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        LiveCellularBase.tear_down(self)

        time.sleep(self._wait_btwn_cmd)

        # We stop any running FTP transfer if any
        if self._use_ftp_dl:
            self._stop_ftp_transfer()
            self._delete_file_if_exists(self._absolute_path_on_dut)

        if self._use_mms:
            self._logger.info("")
            self._logger.info("Tear Down MMS loopback test")
            try:
                self._mms_test_case.tear_down()
            except (KeyboardInterrupt, SystemExit):
                raise
            # We allow bare-except here
            # pylint: disable=W0702
            except:
                # if MMS sending is checked, this is prohibitive
                if self._check_mms:
                    raise
                else:
                    self._logger.error("Tear Down MMS loopback test")

        time.sleep(self._wait_btwn_cmd)

        self._watcher_api.stop_watch_pdp_contexts()
        return Global.SUCCESS, "No errors"

    def _is_ftp_still_running(self):
        """
        Returns a boolean indicating whether a FTP transfer is
        still running or not.

        :rtype: bool
        :return:
            - C{True} if a FTP transfer is still running
            - C{False} otherwise
        """
        ftp_running = False
        status = self._networking_api.get_ftp_xfer_status()
        if status in LiveFitPdpContexts.FTP_TRANSFER_IN_PROGRESS_STATES:
            ftp_running = True
        return ftp_running

    def _stop_ftp_transfer(self, fail_on_error=True):
        """
        Stop the currently running FTP transfer if any.

        The raise of an exception can be deactivated (this should
        allow a better reusability).

        :type fail_on_error: bool
        :param fail_on_error: [optional] a boolean indicating whether
            we want to raise an exception on error or not.
            Defaults to C{True}.

        :raise: Exception if C{fail_on_error} was set to true and
            an error occurred.
        """
        if self._is_ftp_still_running():
            self._logger.info("Trying to stop FTP transfer with id %s" % \
                str(self.__ftp_start_output))
            # We allow broad-except here
            # pylint: disable=W0703
            try:
                self._networking_api.stop_ftp_xfer(self.__ftp_start_output)
            except (KeyboardInterrupt, SystemExit):
                raise
            except Exception as ftp_except:
                if fail_on_error:
                    raise ftp_except
                else:
                    message = "Exception caught while trying to stop " \
                        "FTP transfer with id %s" % str(self.__ftp_start_output)
                    self._logger.warning(message)
        else:
            self._logger.debug("No running FTP transfer.")

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
