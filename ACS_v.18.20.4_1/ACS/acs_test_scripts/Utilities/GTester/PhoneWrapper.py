"""
:copyright: (c)Copyright 2012, Intel Corporation All Rights Reserved.
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
:summary: A class that aggregates actions on a DUT in a higher-level API
:author: asebbanx
:since: 14/11/2012
"""
import time
import traceback
from UtilitiesFWK.Utilities import Global, internal_shell_exec
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import AcsConfigException


class AndroidDeviceWrapper(object):

    """
    A class used to perform operations on a C{DUT}
    when such operation are not I{UE commands}.
    """

    DISABLE_METHOD_INDEX = 0
    """
    The index to be used when retrieving
    the I{disable} method for a given application path.
    """

    RESTORE_METHOD_INDEX = 1
    """
    The index to be used when retrieving
    the I{restore} method for a given application path.
    """

    WAIT_AFTER_KILL = 2
    """
    The number of seconds to wait after a process has
    been killed before checking that is has actually been
    killed.
    """

    CPDD_PATH = "/system/bin/cpdd"
    """
    The path to the C{cpdd} executable.
    """

    RILD_PATH = "/system/bin/rild"
    """
    The path to the C{rild} executable.
    """

    STMD_PATH = "/system/bin/stmd"
    """
    The path to the C{stmd} executable.
    """

    MMGR_PATH = "/system/bin/mmgr"
    """
    The path to the C{mmgr} executable.
    """

    __INSTANCE = None
    """
    The class attribute used to implement a pseudo-singleton
    mechanism via the C{get_instance} method.
    """

    def __init__(self, device, command_timeout="10"):
        """
        Initializes this instance.

        :param device: the I{DUT} instance
        :type device: AndroidDeviceBase

        :param command_timeout: the timeout (in seconds) to use
            when executing shell commands on the I{DUT}.
        :type command_timeout: int
        """
        self.__device = device
        self.__command_timeout = command_timeout
        self.__disabled_applications = []
        self.__path_and_method = {
            AndroidDeviceWrapper.RILD_PATH: (
                self.disable_rild,
                self.restore_rild),
            AndroidDeviceWrapper.STMD_PATH: (
                self.disable_stmd,
                self.restore_stmd),
            AndroidDeviceWrapper.MMGR_PATH: (
                self.disable_mmgr,
                self.restore_mmgr),
            AndroidDeviceWrapper.CPDD_PATH: (
                self.disable_cpdd,
                self.restore_cpdd)
        }
        self.__is_reboot_advised = False

    @classmethod
    def get_instance(cls, device):
        """
        Returns the C{AndroidDeviceWrapper} instance to use.

        :param device: the I{DUT} instance
        :type device: AndroidDeviceBase

        :rtype: AndroidDeviceWrapper
        :return: the C{AndroidDeviceWrapper} instance
        """
        if AndroidDeviceWrapper.__INSTANCE is None:
            AndroidDeviceWrapper.__INSTANCE = AndroidDeviceWrapper(device)
        return AndroidDeviceWrapper.__INSTANCE

    def is_reboot_advised(self):
        """
        Returns a boolean indicating whether a reboot
        is advised before running any tests.

        Some process killings or daemon executables
        renaming can be taken into account only after the
        I{DUT} has been rebooted.

        :rtype: bool
        :return: whether a reboot should be performed or not:
            - C{True} a a reboot should be done
            - C{False} otherwise
        """
        return self.__is_reboot_advised

    def get_logger(self):
        """
        Returns this device's logger instance.

        :rtype: logger
        :return: Logger
        """
        return self.__device.get_logger()

    def get_device(self):
        """
        Returns this object's I{DUT} instance.
        :rtype: AndroidDeviceBase
        """
        return self.__device

    def get_disabled_applications(self):
        """
        Return a copy of the list of all disabled application paths.

        Note that this method returns a copy of the command list
        so that the original cannot be modified outside of this class.

        :rtype: list
        :return: The list of all disabled application paths.
        """
        return list(self.__disabled_applications)

    def get_disable_method_for_path(self, path):
        """
        Returns the method object to use in order to disable
        the application for the given C{path}.

        :param path: the absolute path of the application to disable
        :type path: str

        :rtype: object
        :return: the method object to call
        """
        disable_method_object = None
        if path in self.__path_and_method:
            method_tuple = self.__path_and_method[path]
            disable_method_object = method_tuple[
                AndroidDeviceWrapper.DISABLE_METHOD_INDEX]
        return disable_method_object

    def get_restore_method_for_path(self, path):
        """
        Returns the method object to use in order to restore
        the application for the given C{path}.

        :param path: the absolute path of the application to restore
        :type path: str

        :rtype: object
        :return: the method object to call
        """
        restore_method_object = None
        if path in self.__path_and_method:
            method_tuple = self.__path_and_method[path]
            restore_method_object = method_tuple[
                AndroidDeviceWrapper.RESTORE_METHOD_INDEX]
        return restore_method_object

    def disable_application(self, application_path):
        """
        Disables the application with the given path.

        :param application_path: absolute path of the application
        :type application_path: str

        :raise AcsBaseException: if an error occured when trying
            to disable the application
        """
        # Initialize the return value
        (exit_status, message) = (Global.FAILURE, "Nothing executed.")
        # First retrieve the disable method
        disable_method = self.get_disable_method_for_path(application_path)
        # Check whether a disable method has been declared
        if disable_method is None:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                "Could not find any method to disable application'%s'." % (application_path,))
        # Try to do the job we were ask to
        try:
            # Check that the application has not already been disabled
            if application_path not in self.disabled_applications:
                (exit_status, message) = disable_method()
            # Else if the application was already disabled
            else:
                # Consider that the job has been done correctly
                exit_status = Global.SUCCESS
        # Intercept exceptions raised by utility methods
        except AcsBaseException as exception:
            # And simply raise them
            raise exception
        # Intercept any other errors
        except:
            # Raise a new AcsBaseException to be processed by
            # framework properly
            traceback.print_exc()
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Could not disable application '%s'." % application_path)
        # If no error occurred
        else:
            # Add the application path to the disabled application list
            if application_path not in self.disabled_applications:
                self.__disabled_applications.append(application_path)
        # Return the execution result(s)
        return exit_status, message

    def restore_application(self, application_path):
        """
        Restores the application with the given path.

        :param application_path: absolute path of the application
        :type application_path: str

        :raise AcsBaseException: if an error occured when trying
            to restore the application
        """
        # Initialize the return value
        (exit_status, message) = (Global.FAILURE, "Nothing executed.")
        # First retrieve the restore method
        restore_method = self.get_restore_method_for_path(application_path)
        # Check whether a restore method has been declared
        if restore_method is None:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                "Could not find any method to enable application'%s'." %
                application_path)
        try:
            # Check that the application has been disabled
            if application_path in self.disabled_applications:
                (exit_status, message) = restore_method()
            # If the application was never disabled
            else:
                # Consider that the job has been done correctly
                exit_status = Global.SUCCESS
        # Intercept exceptions raised by utility methods
        except AcsBaseException as exception:
            # And simply raise them
            raise exception
        # Intercept any other errors
        except:
            # Raise a new AcsBaseException to be processed by
            # framework properly
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Could not enable application '%s'." % application_path)
        # If no error occurred
        else:
            # Remove the application path from the disabled application list
            if application_path in self.disabled_applications:
                self.__disabled_applications.remove(application_path)
        # Return the execution result(s)
        return exit_status, message

    def restore_all_applications(self):
        """
        Restores all the previously disabled applications.

        :raise AcsBaseException: if an error occured when trying
            to restore one of the applications
        """
        for application in self.disabled_applications:
            self.restore_application(application)

    def disable_cpdd(self):
        """
        Disables the C{cpdd} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            disabling the application
        """
        # We compute the new file path
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.CPDD_PATH)
        # We rename the file
        self.__move_file(AndroidDeviceWrapper.CPDD_PATH, restore_file_path)
        # We try and stop the corresponding process
        try:
            # Licence to kill
            self.__kill_process(AndroidDeviceWrapper.CPDD_PATH)
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            # If the killing failed, indicate that we should
            # restart the board
            self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def restore_cpdd(self):
        """
        Restores the C{cpdd} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            restoring the application
        """
        # We simply rename the binary file
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.CPDD_PATH)
        # We rename the file
        self.__move_file(restore_file_path, AndroidDeviceWrapper.CPDD_PATH)
        # We indicate that the board should be rebooted
        self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def disable_rild(self):
        """
        Disables the C{rild} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            disabling the application
        """
        # We compute the new file path
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.RILD_PATH)
        # We rename the file
        self.logger.info("Renaming binary file: %s." %
                         AndroidDeviceWrapper.RILD_PATH)
        self.__move_file(AndroidDeviceWrapper.RILD_PATH, restore_file_path)
        # We try and stop the corresponding process
        try:
            # Licence to kill
            self.__kill_process(AndroidDeviceWrapper.RILD_PATH)
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            # If the killing failed, indicate that we should
            # restart the board
            self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def restore_rild(self):
        """
        Restores the C{rild} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            restoring the application
        """
        # We simply rename the binary file
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.RILD_PATH)
        # We rename the file
        self.logger.info("Renaming file: %s." %
                         restore_file_path)
        self.__move_file(restore_file_path, AndroidDeviceWrapper.RILD_PATH)
        # We indicate that the board should be rebooted
        self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def disable_stmd(self):
        """
        Disables the C{stmd} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            disabling the application
        """
        # We compute the new file path
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.STMD_PATH)
        # We rename the file
        self.logger.info("Renaming binary file: %s." %
                         AndroidDeviceWrapper.STMD_PATH)
        self.__move_file(AndroidDeviceWrapper.STMD_PATH, restore_file_path)
        # We try and stop the corresponding process
        try:
            # Licence to kill
            self.__kill_process(AndroidDeviceWrapper.STMD_PATH)
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            # If the killing failed, indicate that we should
            # restart the board
            self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def restore_stmd(self):
        """
        Restores the C{stmd} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            restoring the application
        """
        # We simply rename the binary file
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.STMD_PATH)
        # We rename the file
        self.logger.info("Renaming file: %s." %
                         restore_file_path)
        self.__move_file(restore_file_path, AndroidDeviceWrapper.STMD_PATH)
        # We indicate that the board should be rebooted
        self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def disable_mmgr(self):
        """
        Disables the C{mmgr} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        Note that if the board is restarted, C{mmgr} will
        probably be restarted as well. So that method call
        should occur B{after} any reboot.

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            disabling the application
        """
        # Should we try a more violent way to stop the process ?
        try_harder = False
        # We try and stop the corresponding process
        try:
            self.__stop_daemon(AndroidDeviceWrapper.MMGR_PATH)
        # We choose any name for the execption, starting with "_"
        # in order to avoid Pylint warnings.
        except AcsBaseException as _acs_ex:
            try_harder = True
        if not try_harder:
            # We return the operation status
            return Global.SUCCESS, "No error."

        # At this point we should try a more violent way
        # to stop the process.
        self.logger.debug(
            "The standard way to stop application failed."
            "Trying to rename the binary file.")
        # We compute the new file path
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.MMGR_PATH)
        # We rename the file
        self.logger.info("Renaming binary file: %s." %
                         AndroidDeviceWrapper.MMGR_PATH)
        self.__move_file(AndroidDeviceWrapper.MMGR_PATH, restore_file_path)
        # We try and stop the corresponding process
        try:
            # Licence to kill
            self.__kill_process(AndroidDeviceWrapper.MMGR_PATH)
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            # If the killing failed, indicate that we should
            # restart the board
            self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def restore_mmgr(self):
        """
        Restores the C{mmgr} and returns a C{tuple} containing
        useful information about the command execution:
        (<exit_status>, <stdout>), with:
            - <exit_status>: an C{int}:
                - either the exit status of the command used to disable
                    the application
                - either C{Global.FAILURE} or C{Global.SUCCESS}
            - <stdout>: a C{str}: the output of the command or any relevant piece
                of information as str

        :rtype: tuple
        :return: a C{tuple} containing (<exit_status>, <stdout>)

        :raise AcsBaseException: if an error occured while
            restoring the application
        """
        # Should we try a more violent way to stop the process ?
        try_harder = False
        # We try and start the corresponding process
        try:
            self.__start_daemon(AndroidDeviceWrapper.MMGR_PATH)
        # We choose any name for the execption, starting with "_"
        # in order to avoid Pylint warnings.
        except AcsBaseException as _acs_ex:
            try_harder = True
        if not try_harder:
            # We return the operation status
            return Global.SUCCESS, "No error."

        # At this point we should try a more violent way
        # to stop the process.
        self.logger.debug(
            "The standard way to start application failed."
            "Trying to rename the binary file.")
        # We simply rename the binary file
        restore_file_path = self.__get_restore_file_path(
            AndroidDeviceWrapper.MMGR_PATH)
        # We rename the file
        self.logger.info("Renaming file: %s." %
                         restore_file_path)
        self.__move_file(restore_file_path, AndroidDeviceWrapper.MMGR_PATH)
        # We indicate that the board should be rebooted
        self.__is_reboot_advised = True
        # We return the operation status
        return Global.SUCCESS, "No error."

    def __get_restore_file_path(self, path):
        """
        Returns a path that can be used to rename the file
        given by C{path}.

        :param path: the path of the file to rename.
        :type path: str

        :rtype: str
        :return: the path of the renamed file

        :raise AcsBaseException: if an invalid parameter has
            been provided.
        """
        if path in (None, ""):
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Invalid path given: %s." % path)
        else:
            return "%s.bak" % str(path)

    def __move_file(self, current_path, new_path):
        """
        Renames the file indicated by C{current_path} to a file
        with path indicated by C{new_path}.

        :param current_path: the path of the file to be renamed
        :type current_path: str

        :param new_path: the new file path
        :type new_path: str

        :raise AcsBaseException: if an error occured when trying
            to rename the file
        """
        # Compute the new path if not provided
        if not new_path:
            new_path = "%s.bak" % str(current_path)
        # Create the command
        command = "adb shell mv %s %s" % (current_path, new_path)
        # Execute the command
        self.__run_command(command)

    def __kill_process(self, path_or_name):
        """
        Kills all the process with the given name using the C{killall}
        command.

        C{path_or_name} can refer either to a process name or the absolute
        path of the executable. In that later case, the process name will
        be inferred from the path by taking the last path element.

        :raise AcsBaseException: if a process corresponding to C{path_or_name}
            is still running at the end of this method execution.
        """
        # Initialize the process name
        process_name = path_or_name
        # Check whether we were given a path or a name
        if "/" in path_or_name:
            # If we hold a path, we extract the last element
            index = path_or_name.rfind("/")
            process_name = path_or_name[index + 1:]
        # Check that the process name is not empty
        if process_name in ("", None):
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Invalid process name/path given: %s." % path_or_name)
        self.logger.info("Trying to kill process: %s." % process_name)
        # Check that there is at least one process with
        # that name running
        output = self.__run_command("adb shell ps")
        # If that is not the case, we have nothing to do
        if process_name not in output:
            return
        # Else we have to try and kill it
        command = "adb shell killall %s" % process_name
        self.__run_command(command)
        # Wait a few seconds
        time.sleep(AndroidDeviceWrapper.WAIT_AFTER_KILL)
        # Check that the killing actually happened
        output = self.__run_command("adb shell ps")
        # If that is not the case, raise an exception
        if process_name in output:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "It seems the process '%s' was not killed." % process_name)

    def __stop_daemon(self, binary_path):
        """
        :raise AcsBaseException: if a process corresponding to C{binary_path}
            is still running at the end of this method execution.
        """
        # Initialize the process name
        process_name = binary_path
        # Check whether we were given a path or a name
        if "/" in binary_path:
            # If we hold a path, we extract the last element
            index = binary_path.rfind("/")
            process_name = binary_path[index + 1:]
        # Check that the process name is not empty
        if process_name in ("", None):
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Invalid process name/path given: %s." % binary_path)
        # Try and stop the process
        command = "adb shell %s stop" % binary_path
        self.__run_command(command)
        # Wait a few seconds
        time.sleep(AndroidDeviceWrapper.WAIT_AFTER_KILL)
        # Check that the killing actually happened
        output = self.__run_command("adb shell ps")
        # If that is not the case, raise an exception
        if process_name in output:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "It seems the process '%s' was not stopped." % process_name)

    def make_result_directory(self):
        """
        Create the C{GTester} result directory on the I{DUT} if needed.
        :raise AcsBaseException: if the directory could not be created.
        """
        # Initialize temporary variables
        dir_name = "component_testing"
        dir_path = "/data/%s" % dir_name
        exist = False
        # Check whether the directory already exists or not
        output = self.__run_command("adb shell ls /data")
        if dir_name in output:
            exist = True
        if exist:
            # We return the operation status
            return Global.SUCCESS, "No error."
        # Create the directory if needed
        command = "adb shell mkdir %s" % dir_path
        self.__run_command(command)
        # Check that the directory has been created
        output = self.__run_command("adb shell ls /data")
        if dir_name in output:
            # We return the operation status
            return Global.SUCCESS, "No error."
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "The directory %s could not be created." % dir_path)

    def __start_daemon(self, binary_path):
        """
        :raise AcsBaseException: if a process corresponding to C{binary_path}
            is not running at the end of this method execution.
        """
        # Initialize the process name
        process_name = binary_path
        # Check whether we were given a path or a name
        if "/" in binary_path:
            # If we hold a path, we extract the last element
            index = binary_path.rfind("/")
            process_name = binary_path[index + 1:]
        # Check that the process name is not empty
        if process_name in ("", None):
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Invalid process name/path given: %s." % binary_path)
        # Try and stop the process
        command = "adb shell %s start" % binary_path
        self.__run_command(command)
        # Wait a few seconds (reuse the same value  already
        # used for killings)
        time.sleep(AndroidDeviceWrapper.WAIT_AFTER_KILL)
        # Check that the killing actually happened
        output = self.__run_command("adb shell ps")
        # If that is not the case, raise an exception
        if process_name not in output:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "It seems the process '%s' was not started." % process_name)

    def remount(self):
        """
        Remounts the device file-system (in R/W).

        :raise AcsBaseException: if the exit status of the command
            indicates an error
        :rtype: str
        :return: the command output
        """
        self.logger.info("Remounting filesystem.")
        return self.__run_command("adb remount")

    def __run_command(self, command):
        """
        Runs the given I{shell} command on the I{DUT} and raises
        an exception if the exit status indicates an error.

        :param command: the command to run
        :type command: str

        :raise AcsBaseException: if the exit status of the command
            indicates an error

        :rtype: str
        :return: the command output
        """
        exit_status = Global.FAILURE
        try:
            # We have no choice but call a protected method
            # so we disable the corresponding Pylint warning.
            # pylint: disable=W0212
            cmd = self.__device.format_cmd(command)
            (exit_status, output) = internal_shell_exec(
                cmd,
                int(self.__command_timeout))
        # We want to trap any kind of error/exception so
        # we use an empty exception clause and disable the
        # corresponding Pylint warning.
        # pylint: disable=W0702
        except:
            exit_status = Global.FAILURE
            traceback.print_exc()
        if exit_status != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Command execution failed (command: '%s')." % command)
        return output

    def reboot(self):
        """
        Reboots the board (in I{MOS} mode).

        :raise AcsBaseException: if the reboot failed.
        """
        # We try to reboot the board
        try:
            # Reboot the board
            self.__device.reboot()
        # Handle DeviceException
        except DeviceException as device_exception:
            # The reboot method could raise a DeviceException
            # that we want to handle properly
            raise device_exception
        # Handle all other errors
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Could not reboot the board.")
        # If no exception occurred
        else:
            # Indicate that we do not need to
            # reboot the board anymore
            self.__is_reboot_advised = False

    def pull(self, remote_file, local_file):
        """
        Retrieves the file pointed by C{remote_file} to
        the location pointed by C{local_file}.

        :param remote_file: the remote file path
        :type remote_file: str

        :param local_file: the local file path
        :type local_file: str

        :raise AcsBaseException: if the I{pull} operation failed.
        """
        # We try to retrieve the file
        try:
            # Pull the file with a 20 seconds timeout
            self.__device.pull(remote_file, local_file, 20, force_execution=True)
        # Handle AcsBaseException
        except AcsBaseException as acs_exception:
            # The pull method could raise a AcsBaseException
            # that we want to handle properly
            raise acs_exception
        # Handle all other errors
        except:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Could not pull the remote file from the board.")

    # Declare properties
    device = property(
        get_device,
        None,
        None,
        "This object's DUT instance.")

    disabled_applications = property(
        get_disabled_applications,
        None,
        None,
        "The list of all disabled application paths.")

    is_reboot_advised = property(
        is_reboot_advised,
        None,
        None,
        "Is a reboot advised before running any more tests ?")

    logger = property(
        get_logger,
        None,
        None,
        "This object's device logger.")
