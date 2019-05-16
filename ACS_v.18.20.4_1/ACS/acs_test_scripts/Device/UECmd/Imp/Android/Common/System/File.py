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
:summary: This file implements the File system interaction UEcmd for Android device
          This is for testing file system not doing simple W/R action.
:since: 10/28/2011
:author: wchen61, vgomberx
"""

import os
import re
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.System.IFile import IFile
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException
import UtilitiesFWK.Utilities as Utils
from UtilitiesFWK.Parsers.JsonParser import JsonParser
from ErrorHandling.AcsBaseException import AcsBaseException


class File(BaseV2, IFile):

    """
    :summary: File UEcommands operations for Android platforms
    """

    def __init__(self, device):
        """
        Constructor.
        """

        BaseV2.__init__(self, device)
        IFile.__init__(self, device)
        self._logger = device.get_logger()
        self._phone_system = device.get_uecmd("PhoneSystem")
        # this is a folder where script in cos or mos could be stored
        self.__sysfs_folder = "/data/"

    def operate(self, folder_level, folder_sub_number, folder_name_length,
                file_number, file_size, file_name_length, src_path, dst_path, copy):
        """
        Do the file operation with parameters, the file operation time out is according to folder level,
        sub folder number, file number and file size. First calculate the files size to be created,
        and div 512 get the timeout(assume 512kb/s is enough for file operate).

        :type folder_level: int
        :param folder_level: The folder level.
        :type folder_sub_number: int
        :param folder_sub_number: The sub folder number in each folder.
        :type folder_name_length: int
        :param folder_name_length: The folder name length, name will generate randomly.
        :type file_number: int
        :param file_number: The file number in each folder.
        :type file_size: int
        :param file_size: The file size, in KB.
        :type file_name_length: int
        :param file_name_length: The file name length, name will generate randomly.
        :type src_path: str
        :param src_path: The path of file or folder will be created.
        :type dst_path: str
        :param dst_path: The path of file or folder will be copied.

        :rtype: list
        :return: operation status & output log
        """
        module = "acscmd.filesystem.FileSystemModule"
        function = "operate"
        cmd_args = "--ei folder_level {0} --ei folder_sub_number {1} --ei folder_name_length {2} " \
                   "--ei file_number {3} --ei file_size {4} --ei file_name_length {5} --es src_path {6} " \
                   "--es dst_path {7} --ez copy {8}".format(folder_level, folder_sub_number, folder_name_length,
                                                            file_number, file_size, file_name_length, src_path,
                                                            dst_path, copy)

        if int(folder_level) == 0:
            total_size = int(file_number) * int(file_size)
        else:
            total_size = pow(int(folder_sub_number), int(folder_level) - 1) * int(file_number) * int(file_size)
        timeout = total_size
        if timeout < 50:
            timeout = 50

        result = self._internal_exec_v2(module, function, cmd_args, timeout)

        if not (result["output"] is None):
            return result["output"]
        else:
            return "No error"

    def operate_file_name(self, object_type, previous_name, following_name, operation_type):
        """
        Do the file name operation in emmc with parameters,
        including rename with ordinary/long name, create with multi-extension name.

        :type object_type: str
        :param object_type: The object type, file|folder.
        :type previous_name: str
        :param previous_name: The previous name of file or folder will be renamed or created.
        :type following_name: str
        :param following_name: The following name of file or folder will be renamed.
        :type operation_type: str
        :param operation_type: The operation type, rename|create.

        :rtype: list
        :return: operation status & output log
        """
        module = "acscmd.filesystem.FileSystemModule"
        function = "operateFileName"
        cmd_args = "--es object_type {0} --es previous_name {1} --es following_name {2} " \
                   "--es operation_type {3}".format(object_type, previous_name, following_name, operation_type)

        result = self._internal_exec_v2(module, function, cmd_args)

        if not (result["output"] is None):
            return result["output"]
        else:
            return "No error"

#--------------------------------------------------------------------------------

    def get_file_permissions(self, file_name, user_privilege):
        """
        get the right permission for a given file and user.
        will return a key word and the raw permission output.

        :type file_name: str
        :param file_name: name of the file (the absolute pat of the file)
        :type permissions: str
        :param permissions: the permissions to check ( ex: ".rwxrwxrwx")
        :param user_privilege: can take value "root" or  "user" indicate the right of whom we are checking

        :rtype: tuple (str,str)
        :return: (permission as string "READ_ONLY","WRITE_ONLY", "READ_WRITE" or "NO_READ_WRITE", raw permission)
        """
        cmd = "adb shell ls -l %s" % file_name
        # Sending the command.
        output = self._exec(cmd, force_execution=True).strip()

        if not self.is_shell_output_ok(output):
            if not "permission denied" in output.lower():
                msg = "error when reading file permission :" + output
                self._logger.error(msg)
                raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)
            else:
                result = "NO_READ_WRITE"
                raw_right = output
        else:
            output = filter(None, output.split(" "))
            raw_right = output[0][1:]
            owner = output[1].lower()

            user_privilege = user_privilege.lower()
            if user_privilege == owner:
                right = raw_right[0:2]

            # By default we consider on android that shell and root are never on the same group
            else:
                right = raw_right[6:]

            result = "NO_READ_WRITE"
            if "rw" in right:
                result = "READ_WRITE"
            elif "w" in right:
                result = "WRITE_ONLY"
            elif "r" in right:
                result = "READ_ONLY"

        return result, raw_right

    def rename(self, oldname, newname):
        """
        Rename a file.

        :type newname: str
        :param newname: the new name
        :type oldname: str
        :param oldname: the old name

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("rename file %s into %s" % (oldname, newname))
        cmd = "adb shell rename %s %s" % (oldname, newname)
        output = self._exec(cmd, force_execution=True).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def move(self, oldpath, newpath):
        """
        move a file.

        :type newpath: str
        :param newpath: the new name
        :type oldpath: str
        :param oldpath: the old name

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("rename file %s into %s" % (oldpath, newpath))
        cmd = "adb shell mv %s %s" % (oldpath, newpath)
        output = self._exec(cmd, force_execution=True).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def copy(self, source_file, destination_file):
        """
        Copy a file.

        :type source_file: str
        :param source_file: path of the file to copy , eg: /ect/hello

        :type destination_file: str
        :param destination_file: path of the copied file , eg: /ect/hellocopy

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("copy file %s into %s" % (source_file, destination_file))
        cmd = "adb shell cp -r %s %s" % (source_file, destination_file)

        output = self._exec(cmd, force_execution=True).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def find_files(self, root_path, match_pattern):
        """
        Find all files matching the pattern in the I{root_path}
        :type root_path: str
        :param root_path: the folder path where searching
        :type match_pattern: str
        :param match_pattern: the file pattern to find (case sensitive)

        :rtype tuple
        :return: (execution status, list of files)
        """
        target_file = self._device.get_device_os_path().join(root_path, match_pattern)
        self._logger.info("test if file %s exist" % target_file)
        cmd = "adb shell ls %s" % target_file

        output = self._exec(cmd).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def exist(self, target_file):
        """
        Test if a file exist.

        :type target_file: str
        :param target_file: path of the checked file, eg: /ect/hello

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("test if file %s exist" % target_file)
        cmd = "adb shell ls %s" % target_file

        output = self._exec(cmd, force_execution=True, raise_error=False).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def size(self, target):
        # Use ls -l to get the file size. If someone has a better command,
        #  please upgrade this function.
        cmd = "adb shell ls -l " + target
        output = self._exec(cmd).strip()
        if not self.is_shell_output_ok(output):
            raise DeviceException(DeviceException.PHONE_OUTPUT_ERROR, "Got this response when requesting file size: " + output)

        # An ls output looks like this:
        #  -rwxrwxrwx root     root       652566 2014-01-15 11:11 file_name
        return long(output.split()[3])

    def write_file(self, target_file, text, mode='overwrite'):
        """
        Writes given text into given file( can create the file if doesnt exist)

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello
        :type text: str
        :param text: text to write into the file
        :type mode: str
        :param mode: Optional open mode :
                     "overwrite" to erase or create contains and add text
                     "add" to add text to current contain

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("write '%s' into file %s " %
                         (text, target_file))
        modified_text = ""
        text = str(text)
        if text.find("\n") != -1:
            text = text.splitlines()
            for line in text:
                modified_text += line + "\\n"
        else:
            modified_text = text

        if mode == 'add':
            cmd = "adb shell \"echo -e '%s' >> %s\"" % (modified_text, target_file)
        else:
            cmd = "adb shell \"echo -e '%s' > %s\"" % (modified_text, target_file)

        output = self._exec(cmd).strip()
        result = self.is_shell_output_ok(output)

        return result, output

    def read_file(self, target_file):
        """
        Returns the contain of given file.

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("read  file %s" % target_file)

        cmd = "adb shell cat %s" % (target_file)
        output = self._exec(cmd).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def delete(self, filename, make_secure_copy=False):
        """
        Deletes the file.

        :type filename: str
        :param filename: file to delete

        :type make_secure_copy: bool
        :param make_secure_copy: Do a backup of file before delete

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("delete file: %s " % filename)
        new_path = ""
        if make_secure_copy:
            new_path = self._task_folder + "/" + os.path.basename(filename)
            self._logger.info("making a secure copy of your file at : %s " % new_path)
            self.copy(filename, new_path)

        cmd = "adb shell rm -rf %s " % filename

        output = self._exec(cmd).strip()
        result = self.is_shell_output_ok(output)

        if make_secure_copy:
            if result:
                self._logger.info("restore deleted file from : %s " % new_path)
                works, _ = self.copy(new_path, filename)
            # delete secure copy only if the copy works.
            if works:
                self._exec("adb shell rm -rf %s " % new_path)

        return result, output

    def is_equal(self, file1, file2):
        """
        compare the checksum of 2 files to see is they are equals

        :type file1: str
        :param file1: first file to compare from

        :type file2: str
        :param file2: second file to compare with

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.info("comparing if file %s is equal to %s" % (file1, file2))

        cmd = "adb shell md5 %s" % file1
        output = self._exec(cmd).strip()
        if self.is_shell_output_ok(output):
            sum1 = output.split(" ")[0].strip()
        else:
            msg = "error when generating md5sum value :" + output
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        cmd = "adb shell md5 %s" % file2
        output = self._exec(cmd).strip()
        if self.is_shell_output_ok(output):
            sum2 = output.split(" ")[0].strip()
        else:
            msg = "error when generating md5sum value :" + output
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        if sum1 == sum2:
            msg = "Files %s and %s are identical" % (file1, file2)
        else:
            msg = "Files %s and %s are different" % (file1, file2)

        return sum1 == sum2, msg

    def remount(self):
        """
        remount file system
        """
        self._exec("adb remount", force_execution=True)

    def set_file_permission(self, target_file, right):
        """
        wrapper for function chmod, can also take a value like from ls -l minus the first '-'
        e.g : rwxr--r--  , first 3 character is for owner ,then group and finally other.

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello

        :type right: str
        :param right: right to apply , like 777 or rw-r--r--

        :type user_privilege: str
        :param user_privilege: can take value "root" or "user" to force the
                            execution of the command as user

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        if not right.isdigit():
            # consider we are in presence of a output from ls -l
            right = right.replace("r", "4").replace("w", "2").replace("x", "1").replace("-", "0").strip()
            result = ""
            mode_code = 0
            itero = 0
            for element in right:
                itero += 1
                mode_code += int(element.strip())
                # pack value every 3 occuration
                if itero == 3:
                    result += str(mode_code)
                    mode_code = 0
                    itero = 0

            if itero > 0:
                # if the last element is not complete
                result += str(mode_code)
            right = result

        self._logger.info("Setting file %s permissions to %s" % (target_file, right))
        cmd = "adb shell chmod %s %s" % (right, target_file)
        output = self._exec(cmd, force_execution=True).strip()
        result = self.is_shell_output_ok(output)
        return result, output

    def __get_user_privilege(self):
        result = "UNKNOWN"
        raw_result = self._exec("adb shell id", force_execution=True)
        if  "root" in raw_result:
            result = "root"
        elif  "shell" in raw_result:
            result = "shell"
        return result

    def change_user_privilege(self, mode):
        """
        change the user privilege.
        take care, as this will stop adb for a while and may
        cause adb connection to crash

        :type mode: str
        :param mode: can take value "root" or "user" to force the
                            execution of the command as user
        """
        mode = str(mode).upper()
        self._logger.info("try to change user privelege to " + mode)
        if mode == "ROOT":
            if  self.__get_user_privilege() != "root":
                self._exec("adb root", force_execution=True, wait_for_response=False)
            else:
                self._logger.info("user privelege is already " + mode)

        elif mode == "USER":
            if self.__get_user_privilege() != "shell":
                script_name = "%sSCRIPT_UNROOT.sh" % (self.__sysfs_folder)
                if not self.is_shell_output_ok(self._exec("adb shell ls -l " + script_name)):
                    # check that the script exist else create it
                    cmd_core = 'stop adbd && setprop service.adb.root 0 && start adbd'
                    # dirty hardcoded path
                    cmd = "adb shell echo \'%s\' > %s" % (cmd_core, script_name)
                    self._exec(cmd, force_execution=True)
                    # Set execution permissions
                    cmd = "adb shell chmod 777 %s" % script_name
                    self._exec(cmd, force_execution=True)

                # Run script detached on DUT
                cmd = "adb shell nohup sh %s" % (script_name)
                self._exec(cmd, force_execution=True, wait_for_response=False)
            else:
                self._logger.info("user privelege is already " + mode)
        else:
            msg = "unknown user mode %s, can only be ROOT or USER" % mode
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

    def get_file_owner(self, file_name):
        """
        get the file owner and groupe.

        :type file_name: str
        :param file_name: name of the file (the absolute pat of the file)

        :rtype: tuple (str,str)
        :return: (owner, owner_group) , these value can be equal to None if failed to be found
        """
        cmd = "adb shell ls -l %s" % file_name
        # Sending the command.
        output = self._exec(cmd, force_execution=True).strip()

        if not self.is_shell_output_ok(output):
            msg = "error when trying to get file owner :" + output
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # this parsing is based on linux ls return style
        # -rw-rw---- u0_a46   u0_a46        229 2015-02-11 15:46 com.google.android.GoogleCamera_media.xml
        # there is case with extra value
        # -rw-rw---- 1 u0_a39 u0_a39 1261 2016-01-08 22:42 /data/data/com.android.camera2/shared_prefs/com.android.camera2_preferences.xml
        # remove the file name
        output = output.rstrip(file_name)
        output = filter(None, output.split(" "))

        owner = None
        owner_group = None
        # in some case, get owner got more element
        if len(output) > 6:
            if len(output) > 3:
                owner = output[2].strip()
            if len(output) > 4:
                owner_group = output[3].strip()

        elif len(output) > 5:
            if len(output) > 2:
                owner = output[1].strip()
            if len(output) > 3:
                owner_group = output[2].strip()

        return owner, owner_group

    def set_file_owner(self, file_name, owner, owner_group=None):
        """
        Set the file owner and groupe.

        :type file_name: str
        :param file_name: name of the file (the absolute pat of the file)
        :type owner: str
        :param owner: the owner to set
        :type owner_group: str
        :param owner_group: the owner group

        :rtype: tuple (str,str)
        :return: (owner, owner_group), these value can be equal to None if failed to be found
        """
        if owner_group is None:
            cmd = "adb shell chown %s %s" % (owner, file_name)
        else:
            cmd = "adb shell chown %s:%s %s" % (owner, owner_group, file_name)
        # Sending the command.
        output = self._exec(cmd, force_execution=True).strip()

        if not self.is_shell_output_ok(output):
            msg = "error when try to change file owner :" + output
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

#--------------------------------------------------------------------------------

    def retrieve_size_from_filename(self, filename):
        """
        Try to retrieve file size from the filename.
        Filename must follow a specific format:
        [name][size (integer)][unit (k, ko, kB, m, mo, mB, g, go, gB)][extension (optional)]
        example : put500MB.zip

        :return: Size in kilo-Byte
        :rtype: int
        """
        known_units = ("ko", "kb", "mo", "mb", "go", "gb", "k", "m", "g")
        coef = {"ko": 1, "k": 1, "kb": 1,
                "mo": 1024, "m": 1024, "mb": 1024,
                "go": (1024 * 1024), "g": (1024 * 1024), "gb": (1024 * 1024)}

        # retrieve only filename
        filename = os.path.basename(filename)
        # retrieve all integers and unit from the filename
        self._logger.debug("Filename is '%s'" % str(filename))

        regex_search = re.search("^\D*(\d*)(\w).*$", str(filename))

        if regex_search is not None:
            size = str(regex_search.group(1))
            unit = str(regex_search.group(2)).lower()
            self._logger.debug("File size will be '%s'" % str(size))
            self._logger.debug("File unit will be '%s'" % str(unit))

            if unit in known_units:
                # convert size in kilobyte
                return int(int(size) * coef[unit])
            else:
                self._logger.warning("Unkown unit '%s'" % str(unit))

        raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                               "Filename doesn't follow the specific format : "
                               "[name][size (integer)][unit %s]"
                               "[extension (optional)]" % str(known_units))

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

        output_file = os.path.normpath(folder + '/' + filename).replace("\\", "/")

        if not isinstance(folder, (str, unicode)):
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect destination folder : "
                                  "unable to create a user data in the device")

        # create output folder if needed
        if folder not in ("/", "", " "):
            self._exec("adb shell mkdir -p " + folder, 1)

        cmd = ""
        if random_data:
            # this command will create size_mo blocks of 1024 bytes (1kB)
            # Each block will contain random data (if=/dev/urandom)
            cmd = "adb shell dd if=/dev/urandom of=%s bs=1024 count=%d" % (output_file, size_ko)
        else:
            # this command will create ONE block of ONE byte, at the
            # last byte of the file (seek to size_mo mega-bytes - 1 byte)
            cmd = "adb shell dd if=/dev/zero of=%s bs=1 count=1 seek=$((%d*1024 -1))" % (output_file, size_ko)
        self._exec(cmd, self._uecmd_default_timeout)

    def create_file_if_needed(self, filename, size_ko, dest_folder=None):
        """
        Create a file in the DUT, if it doesn't exist or if it has an incorrect
         size.

        :param filename: name of the file to create
        :type filename: str

        :param size_ko: Size in kilo-octet of the wanted file, as an integer
        strictly higher that 0
        :type size_ko: int

        :param dest_folder: Optionnal parameter if the wanted file must be
        outside of the ACS ftp directory
        :type dest_folder: str

        """
        self._logger.info(
            "Checking file existence in the device, creating it if needed")
        # check parameters
        if not filename:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Incorrect filename : cannot create file on device")
        if not isinstance(size_ko, int) or size_ko <= 0:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Incorrect file size : cannot create file on device")

        if dest_folder is None:
            dest_folder = self._device.multimedia_path
        else:
            filename = os.path.basename(filename)

        file_path = os.path.normpath(dest_folder + "/" + filename).replace("\\", "/")

        self._logger.debug("File to check : " + file_path)
        # check if file exists
        file_exists = self._phone_system.check_file_exist_from_shell(file_path)

        if file_exists:
            self._logger.debug("File already existing: checking size")
            # check size
            file_size = self._phone_system.get_file_size(file_path)
            # convert file size in kilobyte
            file_size = int(file_size / (1024))

            if file_size == size_ko:
                self._logger.debug("File has the correct size. Nothing to be done")
                return
            else:
                self._logger.debug("File has an incorrect size (%d) : it will "
                                   "be replaced by a new one with the wanted size."
                                   % file_size)

        # create or replace file
        self._logger.debug("Create file %s of %d MB" % (file_path, size_ko))
        self.create_custom_size_user_data(file_path, size_ko, folder="")

    def unzip(self, file_name):
        """
        Unzip a file on the DUT.
        :type file_name: str
        :param file_name: name of the file (the absolute path of the file on DUT)

        :rtype: str
        :return: return path+name of unzipped file if successful.  Throws AcsToolException.OPERATION_FAILED if there's an error.
        """

        # Find out size of the file to use as a basis for calculating a timeout for the unzip command.  ls -l should put that in column 4.
        cmd = "adb shell stat -c %%s %s " % file_name
        (return_code, output) = self._device.run_cmd(cmd, timeout=10)
        if return_code != Utils.Global.SUCCESS:
            msg = "uecmd unzip: Error '%s' when trying to list the file '%s'" % (output, file_name)
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # Trusting that 'ls -l' provided file size in column 4.  If this does not have a number, then 'int' will throw an exception.
        file_size = int(output)
        timeout = int(file_size / 4096)
        if timeout < 100000:
            # Set a 100s minimum unzip timeout for small files
            timeout = 100000

        simple_filename = os.path.basename(file_name)
        device_path = os.path.dirname(file_name)
        cmd = "adb shell cd " + device_path + "; gunzip -f " + simple_filename

        if simple_filename[-4:].lower() == ".tgz":
            unzipped_name = file_name[:-4] + ".tar"
        elif simple_filename[-3:].lower() == ".gz":
            unzipped_name = file_name[:-3]
        else:
            msg = "uecmd unzip: unsupported filename extension in '%s'.  Expected a .gz or .tgz file." % simple_filename
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        (return_code, output_msg) = self._device.run_cmd(cmd, timeout=timeout)
        if return_code != Utils.Global.SUCCESS:
            msg = "uecmd unzip: Error '%s' when running '%s'" % (output_msg, cmd)
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        return unzipped_name

    def untar(self, file_name, destination=None, unzip=False):
        """
        Untar a file on the DUT.  Throws AcsToolException.OPERATION_FAILED if there's an error.
        :type file_name: str
        :param file_name: name of the file (the absolute path of the file on DUT)
        :type destination: str
        :param destination: folder where to unzip the file (the absolute path of the folder on DUT).
        :type unzip: boolean
        :param unzip: if True archive will be unzipped before being extracted
        """

        # Find out size of the file to use as a basis for calculating a timeout for the unzip command.  ls -l should put that in column 4.
        cmd = "adb shell stat -c %%s %s" % file_name
        (return_code, output) = self._device.run_cmd(cmd, timeout=10)
        if return_code != Utils.Global.SUCCESS:
            msg = "uecmd untar: Error '%s' when trying to list the file '%s'" % (output, file_name)
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # Trusting that 'ls -l' provided file size in column 4.  If this does not have a number, then 'int' will throw an exception.
        file_size = int(output)
        timeout = int(file_size / 4096)
        if timeout < 100000:
            # Set a 100s minimum unzip timeout for small files
            timeout = 100000

        device_path = os.path.dirname(file_name)
        options = "-xzf" if unzip else "-xf"
        cmd = "adb shell cd " + device_path + "; tar " + options + " " + file_name
        if destination:
            # add option to command
            cmd += " -C " + destination
            # create output folder if needed
            _, dest_exists = self.exist(destination)
            if not dest_exists:
                self._exec("adb shell mkdir -p " + destination, 1)

        (return_code, output_msg) = self._device.run_cmd(cmd, timeout=timeout)
        if return_code != Utils.Global.SUCCESS:
            msg = "uecmd untar: Error '%s' when running '%s'" % (output_msg, cmd)
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

    def is_sdcard_present(self):
        """
        Check whether SD card block is present or not by listing mmc blocks under /dev/block
        :rtype: boolean
        :return: return True if sd card exists.
        """

        cmd = "adb shell 'ls -l /dev/block | grep mmcblk1p1 &>/dev/null'"
        (return_code, output) = self._device.run_cmd(cmd, timeout=10)

        if return_code != Utils.Global.SUCCESS:
            msg = "uecmd is_sdcard_present: Error '%s' when trying to list /dev/block" % (output)
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        if output == "mmcblk1p1":
            return True
        else:
            return False

    def set_json_preferences(self, filename, preferences):
        """
        Configure the mandatory preferences of chrome browser

        :type preferences: dict
        :param preferences: dictionnary of preferences to set
        """
        pref_dir = self._device.get_device_os_path().dirname(filename)
        basename = self._device.get_device_os_path().basename(filename)

        res, msg = self._device.run_cmd("adb pull %s %s" %
                                        (filename, basename), 30)

        if res == Utils.Global.FAILURE:
            self._device.run_cmd("adb shell mkdir -p %s" % pref_dir, 5)
            with open(basename, "w+") as tmp:
                tmp.write("{}")
            tmp.close()

        parser = JsonParser(basename)
        config = parser.get_parsed_content()
        config.update(preferences)

        content = "{!r}".format(preferences).replace("'", "\"")

        with open(basename, "w") as f:
            f.write(content)

        status, msg = self._device.run_cmd("adb push %s %s" %
                                           (basename, filename), 30)

        if os.path.isfile(basename):
            os.remove(basename)

        if status != Utils.Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Cannot write preferences : %s" % msg)
