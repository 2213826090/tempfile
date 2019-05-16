"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: SII on behalf of INTEL MCG PSI
:summary: This file implements miscellaneous phone UECmds
:since: 11 Feb 2014
:author: vgomberx
"""
from acs_test_scripts.Device.UECmd.Interface.Misc.IPhoneSystem import IPhoneSystem
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import is_number, str_to_bool_ex
from ErrorHandling.AcsConfigException import AcsConfigException
import os


class PhoneSystem(Base, IPhoneSystem):
    """
    Class that handles all miscellaneous system related operations.
    """

    def __init__(self, device):
        """
        Initializes this instance.
        """
        Base.__init__(self, device)
        IPhoneSystem.__init__(self, device)

    def display_on(self):
        """
        set device screen on
        :param: None
        :rtype: None
        """
        # Stimulate phone to wake it up
        # module_name, class_name = self._get_module_and_class_names("Display")
        # self._internal_uecmd_exec(module_name, class_name, "DisplaySwitch", "switch=on")
        self._logger.warning("[NOT WORKING on windows] PhoneSystem.display_on ")

    def display_off(self):
        """
        set device screen off
        :param: None
        :rtype: None
        """
        # Stimulate phone to wake it up
        # module_name, class_name = self._get_module_and_class_names("Display")
        # self._internal_uecmd_exec(module_name, class_name, "DisplaySwitch", "switch=off")

        # We disable this uecmd because on windows the board enter in the sleep mode
        # and we can't wake up the device
        self._logger.warning("[NOT WORKING on windows] PhoneSystem.display_off ")

    def wake_screen(self):
        """
        Wakes up the phone screen

        :return: None
        """
        self._logger.debug("Waking up phone screen")
        self._logger.warning("[NOT WORKING on windows] PhoneSystem.display_on ")
        # Stimulate phone to wake it up
        # module_name, class_name = self._get_module_and_class_names("Display")
        # self._internal_uecmd_exec(module_name, class_name, "DisplaySwitch", "switch=on")

    def get_date(self, seconds=False):
        """
        Return the phone date.

        :rtype: String
        :return: date
        """
        self._logger.error(self.__class__.__name__ + ".get_date() not implemented")
        return "9999 02-17 16:13:49"

    # -------------------------UECMD RELATED TO SCREEN SETTING-----------------------

    def get_backlight_level(self):
        """
        Gets the backlight level info

        :rtype: int
        :return: The backlight level
        """
        module_name, class_name = self._get_module_and_class_names("Display")
        output = self._internal_uecmd_exec(module_name, class_name, "getScreenBrightness")
        output = output["values"].get("output")
        if not is_number(output):
            tmp_txt = "get_backlight_level output returned a non integer value %s" % str(output)
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
        else:
            return int(output)

    def set_brightness_mode(self, mode):
        """
        Sets how the brightness is managed by the system.

        :type mode: str
        :param mode: mode to set, 'manual' or 'automatic'
        """
        self._logger.warning("[NOT IMPLEMENTED] PhoneSystem.set_brightness_mode")

    def set_display_brightness(self, brightness):
        """
        Sets the display brightness
        :type: brightness: int
        :param brightness: brightness to set, in percentage from 0 to 100

        :rtype: list
        :return: operation status & output log
        """
        if 100 <= brightness <= 0:
            tmp_txt = "set_display_brightness only take a percentage from [0:100] and not %s" % str(brightness)
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        module_name, class_name = self._get_module_and_class_names("Display")
        params = "percentage=%s" % str(brightness)
        output = self._internal_uecmd_exec(module_name, class_name, "SetScreenBrightness", params)
        print output

    def set_screen_timeout(self, timeout):
        """
        Sets the screen timeout to go in idle.

        :type timeout: int
        :param timeout: the screen timeout in seconds. Use 0 to set maximum delay

        :return: None
        """

        if timeout < 0:
            tmp_txt = "set_screen_timeout only take a positive value and not %s" % str(timeout)
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)

        module_name, class_name = self._get_module_and_class_names("Display")
        params = "timeout=%s" % str(timeout)
        output = self._internal_uecmd_exec(module_name, class_name, "setScreenTimeout", params)

    def get_screen_timeout(self):
        """
        Gets the time before screen goes to idle.

        :rtype: int
        :return: screen timeout in seconds
        """

        module_name, class_name = self._get_module_and_class_names("Display")
        output = self._internal_uecmd_exec(module_name, class_name, "getScreenTimeout")
        output = output["values"].get("output")
        if not is_number(output):
            tmp_txt = "get_screen_timeout output returned a non integer value %s" % str(output)
            self._logger.error(tmp_txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, tmp_txt)
        else:
            return int(output)

    def get_file_size(self, file_path):
        """
        Get the file size in bytes of the file given in parameter.
        Return size as long
        Return -1 if the file does not exist
        Return -2 if the filesize has not been successfully parsed

        :type file_path: str
        :param file_path: Full path of the file

        :rtype: long
        :return: size of the file in bytes. \
                 return -1 if file does not exist.
                 return -2 if the filesize has not been successfully parsed
        """
        self._logger.info("Getting size of %s" % file_path)
        module_name, class_name = self._get_module_and_class_names("DeviceSystem")
        params = "filename=%s" % file_path
        output = self._internal_uecmd_exec(module_name, class_name, "GetFileSize", params)
        size = output["values"].get("fileSize")
        if size is not None:
            return long(size)
        else:
            return -2

    def remove_file(self, filepath):
        """
        Remove file from device

        :param filepath: Name and path of the file to delete
        :type filename: str
        """
        cmd = "cmd.exe /C del %s" % (filepath)
        output = self._internal_exec(cmd)

    def create_custom_size_user_data(self, filename, size_ko, random_data=False, folder=None):
        """
        Create a file directly into the device (no file to upload) with a specific size.

        :param filename: Name of the file to create
        :type filename: str

        :param size_ko: Size of the file to create, in kilo-octet (integer only)
        :type size_ko: int

        :param random_data: generate a file with random data in it.
        Default to False because it takes time to generate a file with random data.
        :type random_data: bool

        :param folder: destination folder on DUT where the file will be created.
        By default, it creates new files in ACS ftpdir_path of the device.
        :type folder: str
        """
        if not filename or not isinstance(filename, (str, unicode)):
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect filename : unable to create a user data in the device")
        if not size_ko or size_ko <= 0:
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect file size : unable to create a user data in the device")
        if not isinstance(random_data, bool):
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect random data mode : "
                                  "unable to create a user data in the device")

        if folder is None:
            folder = self._device.get_ftpdir_path()
            output_file = os.path.normpath(folder + '\\' + filename).replace("/", "\\")
        else:
            output_file = os.path.normpath(filename).replace("/", "\\")

        if not isinstance(folder, (str, unicode)):
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect destination folder : "
                                  "unable to create a user data in the device")

        if not self.check_file_exist_from_shell(folder):
            cmd = "cmd.exe /C mkdir %s" % folder
            self._internal_exec(cmd)

        cmd = "fsutil file createnew %s %d" % (folder + "\\" + output_file, size_ko * 1024)
        output = self._internal_exec(cmd)
        return

    def check_directory_exist_from_shell(self, dir_path):
        """
        Check if the directory exists on the target file system from shell
        you need root access at least to do this.

        :type dir_path: str
        :param dir_path: Full path of the directory to be checked

        :rtype: boolean
        :return: True if exist , False otherwise
        """

        cmd = "cmd /C if exist %s\\*.* (echo 'directory exists') else (echo 'no such directory')" % dir_path
        output = self._internal_exec(cmd)
        if str(output).lower().find("no such directory") != -1:
            return False
        else:
            return True

    def create_directory_from_shell(self, dir_path):
        """
        Create the directory running adb shell command

        :type dir_path: str
        :param dir_path: Full path of the directory which needs to be created
        """

        # Check if the directory already exists
        if self.check_directory_exist_from_shell(dir_path):
            self._logger.debug("Directory already exists, no further actions needed")
        else:
            self._logger.debug("Create directory: %s" % (dir_path))
            # Create the directory
            cmd = "cmd /C mkdir %s" % (dir_path)
            output = self._internal_exec(cmd)
            # Raise an error if the command failed
            if str(output).lower().find("read-only file system") != -1:
                self._logger.warning("write_file error: no write permission")
                raise DeviceException(DeviceException.OPERATION_FAILED, "write_file error: no write permission")
            self._logger.debug("The directory '%s' has been created" % (dir_path))

    def set_phone_lock(self, mode):
        """
        When phone is in idle mode, force the phone to stay locked/unlocked.

        :type mode: int
        :param mode: phone lock state in idle mode. Can be:
            -{0}: unlocked
            -{1}: lock
        """

        lockEnabled = str_to_bool_ex(mode)
        if lockEnabled is None:
            self._logger.error("set_phone_lock : Parameter mode %s is not valid" % str(lockEnabled))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter 'mode' is not valid !")

        module_name, class_name = self._get_module_and_class_names("Display")
        if lockEnabled:
            lock_enabled = 1
        else:
            lock_enabled = 0
        params = "lockEnabled=%d" % lock_enabled
        output = self._internal_uecmd_exec(module_name, class_name, "setPhoneLock", params)
        print output

    def set_phone_screen_lock_on(self, lock_on):
        """
        set phone screen lock on
        :type lock_on: integer
        :param lock_on: phone screen light state in idle mode. Can be:
        -{0}: unlocked display always ON
        -{1}: lock display ON
        :rtype: None
        """
        if lock_on in ("on", "1", 1):
            lock_on = 1
            self._logger.info("Lock the phone screen ON.")

        elif lock_on in ("off", "0", 0):
            lock_on = 0
            self._logger.info("Unlock the phone screen ON (can now turn OFF).")

        else:
            self._logger.error("set_phone_screen_lock_on : Parameter lock_on %s is not valid" % str(lock_on))
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Parameter lock_on is not valid !")

        # Set the phone lock

        # function = "setLockScreenLightON"
        # cmd_args = "--ei lockOn %d" % lock_on
        # self._internal_exec_v2(module, function, cmd_args)
        # raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
        self._logger.warning("[NOT IMPLEMENTED] PhoneSystem.set_phone_screen_lock_on")

    def check_file_exist_from_shell(self, file_path):
        """
        Check if the file exists on the target file system from shell
        you need root access at least to do this.

        :type file_path: str
        :param file_path: Full path of the file to be checked

        :rtype: boolean
        :return: True if exist , False otherwise
        """
        cmd = "cmd /C if exist %s (echo 'file exists') else (echo 'file does not exist')" % file_path
        output = self._internal_exec(cmd)
        if str(output).lower().find("file does not exist") != -1:
            return False
        else:
            return True

    def home_page_menu(self):
        """
        Go to home page
        """
        module_name, class_name = self._get_module_and_class_names("UIAction")
        params = ""
        self._internal_uecmd_exec(module_name, class_name, "ShowMetroStartMenu", params)

    def set_stay_on_while_plugged_in(self, value, use_agent=True):
        """
        Set the value for Windows to stay awake when plugged to host

        :type value: int
        :param value: the value to set (between 0 to 3)
        """
        self._logger.debug("[NOT IMPLEMENTED] set_stay_on_while_plugged_in on Windows")

    def delete(self, filename, raise_exception_on_failure=True):
        """
        Deletes the file.

        :type filename: str
        :param filename: file to delete

        :type raise_exception_on_failure: Boolean
        :param raise_exception_on_failure: Raise exception on failure or not

        :rtype: boolean
        :return: True if the file has been well deleted, False otherwise
        """
        self._logger.info("delete file: %s " % filename)

        cmd = "cmd /C del %s " % filename
        output = self._internal_exec(cmd)

        return True
