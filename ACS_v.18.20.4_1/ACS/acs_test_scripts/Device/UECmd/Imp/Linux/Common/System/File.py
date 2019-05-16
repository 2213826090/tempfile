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

:organization: INTEL NDG SW
:summary: This file implements the File system interaction UEcmd for Linux device
          This is for testing file system not doing simple W/R action.
:since: 04/28/2014
:author: jreynaux
"""

import time
import os
import binascii
import re
from acs_test_scripts.Device.UECmd.Imp.Linux.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IFile import IFile
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class File(Base, IFile):

    """
    :summary: File UEcommands operations for Android platforms
    """

    def __init__(self, device):
        """
        Constructor.
        """
        Base.__init__(self, device)
        IFile.__init__(self, device)
        self._logger = device.get_logger()

        self.__sdcard_device_file = device.get_config("sdcard_dev", "/dev/mmcblk1p1")
        self.__usbdrive_device_file = device.get_config("usbdrive_dev", "/dev/sda1")
        self._phone_system_api = device.get_uecmd("PhoneSystem")

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
        # Check if sdcard requested, if requested, mount it
        self._auto_mount_external_drive([source_file, destination_file])

        cmd = "cp -r %s %s" % (str(source_file), str(destination_file))
        return self._internal_exec(cmd)

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
        if make_secure_copy:
            self._logger.warning("make secure copy : feature not implemented")

        self._logger.debug("Delete file %s ..." % filename)
        cmd = "rm -rf %s" % str(filename)
        return self._internal_exec(cmd, timeout=self._uecmd_default_timeout)

    def exist(self, target_file):
        """
        Test if a file exist.

        :type target_file: str
        :param target_file: path of the checked file, eg: /ect/hello

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self._logger.debug("Checking if %s exists ..." % target_file)
        exist = False
        command_ok, output = self._internal_exec("file %s" % target_file)
        if command_ok:
            if output is not None and "no such file or directory" not in output.lower():
                exist = True

        if exist:
            output = "%s exists !" % target_file
            self._logger.debug(output)
        else:
            output = "%s does not exist" % target_file
            self._logger.warning(output)

        return exist, output

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
        output = "File %s and %s are NOT equal !" % (file1, file2)
        status = False

        command_ok, exec_output = self._internal_exec("md5sum %s" % file1)
        if command_ok:
            md5sum_file1 = exec_output.split()[0]

        command_ok, exec_output = self._internal_exec("md5sum %s" % file2)
        if command_ok:
            md5sum_file2 = exec_output.split()[0]

        if md5sum_file1 in md5sum_file2:
            output = "File %s and %s are identical" %(file1, file2)
            status = True

        return status, output

    def move(self, oldpath, newpath, user_privilege=None):
        """
        move a file.

        :type newpath: str
        :param newpath: the new name
        :type oldpath: str
        :param oldpath: the old name
        :type user_privilege: str
        :param user_privilege: can take value "root" or "user" to force
                            the execution of the command as user

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        cmd = "mv %s %s" % (str(oldpath), str(newpath))
        return self._internal_exec(cmd)

    def rename(self, oldname, newname, user_privilege=None):
        """
        Rename a file.

        :type new_name: str
        :param new_name: the new name
        :type old_name: str
        :param old_name: the old name
        :type user_privilege: str
        :param user_privilege: can take value "root" or "user" to force
                            the execution of the command as user

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        self.move(oldname, newname, user_privilege)

    def get_file_size(self, target):
        """
        Get file size.

        :type target: str
        :param target: the file path to get size

        :rtype: int
        :return: the file size in block size 1

        :raise AcsBaseException in case of wrong usage
        """
        size = None
        cmd = "ls -lrt %s | awk '{print $5}'" % target
        status, output = self._internal_exec(cmd)
        if status:
            # in case of someone use this method to get folder size, raise ex
            if len(str(output).split()) > 1:
                msg = "get_file_size method should be used to get FILE size, check input parameter"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            size = int(output.split()[0])

        return size

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
        # Due to bad previous implementation of UC, need to cast each parameters ... -_-'
        folder_level = int(folder_level)
        folder_sub_number = int(folder_sub_number)
        file_number = int(file_number)
        file_size = int(file_size)
        file_name_length = int(file_name_length)
        folder_name_length = int(folder_name_length)

        # userdataPath (/home/root/)
        emmc_path = self._device.multimedia_path
        # sdcard_ext (/sdcard/)
        sdcard_path = self._device.get_sdcard_path()

        if "emmc" in src_path:
            path = emmc_path
        elif "mnt_sdcard" in src_path:
            # Should be mouted upper
            path = sdcard_path
        else:
            msg = "Wrong given src path. Should be in emmc|mnt_sdcard|mnt_sdcard_ext"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self._logger.debug("emmc_path = %s, sdcard_path = %s, path given = %s" % (emmc_path, sdcard_path, src_path))

        status = False
        if folder_level == 0 and folder_sub_number == 0 and file_number >= 1:
            self._logger.info("Performing file operation with %d file(s) of %d KB, named with %d "
                              "character(s) length and created on %s"
                              % (file_number, file_size, file_name_length, path))

            status = self._do_file_operation(file_number, file_name_length, file_size, path)
        # Mean folder creation
        elif folder_level >= 1 and folder_sub_number == 0 and file_number == 0:
            self._logger.info("Performing folder operation with %d folder(s), named with %d "
                              "character(s) length and created on %s"
                              % (folder_level, folder_name_length, path))

            status = self._do_folder_operation(folder_level, folder_name_length, path)
        else:
            msg = "Given parameters does not match a valid operation !"
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # umount partition is necessary
        time.sleep(10)
        if "sdcard" in str(src_path) or "sdcard" in str(dst_path):
            self._phone_system_api.unmount_device(self._device.get_sdcard_path())

        if not status:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, "Unable to perform operation")

        # If operations succeed, return output message
        return "Operation succeed"

    def _do_file_operation(self, file_number, file_name_length, file_size, src_path):
        """
        Do the file operation with parameters.

        :type file_number: int
        :param file_number: The file number in each folder.
        :type file_size: int
        :param file_size: The file size, in KB.
        :type file_name_length: int
        :param file_name_length: The file name length, name will generate randomly.
        :type src_path: str
        :param src_path: The path of file or folder will be created.
        :type dst_path: The path of file or folder will be copied.

        :rtype: list
        :return: operation status & output log
        """
        # FILE CREATION TEST eg 500 Kb file on EMMC
        # The path of file or folder will be created
        status = False

        # Generate file name(s)
        file_name = binascii.b2a_hex(os.urandom(file_name_length / 2))

        file_name = src_path + file_name

        # create file(s)
        self._logger.info("Create file %s ..." % file_name)
        s, o = self.create_custom_size_user_data(file_name, file_size, folder="")
        if not s:
            msg = "Unable to create user data file"
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        self._logger.info("Check file creation ...")
        s, o = self.exist(file_name)
        if not s:
            msg = "File %s does not exists !" % file_name
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # Verify file size
        self._logger.info("Check file size ...")
        size = self.get_file_size(file_name)
        # size in block size, file_size in KB
        if size != file_size * 1024:
            msg = "The created file size is not correct expected: %d, found: %d" \
                  % (file_size, size)
            self._logger.error(msg)
            status = False
        else:
            msg = "The created file size is correct !"
            self._logger.info(msg)
            status = True

        # delete file
        self.delete(file_name)

        if not status:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # If operations succeed, return True
        return status

    def _do_folder_operation(self, folder_level, folder_name_length, src_path):
        """
        Do the file operation with parameters.

        :type folder_level: int
        :param folder_level: The folder level.
        :type folder_sub_number: int
        :param folder_sub_number: The sub folder number in each folder.
        :type folder_name_length: int
        :param folder_name_length: The folder name length, name will generate randomly.
        :type src_path: str
        :param src_path: The path of file or folder will be created.

        :rtype: list
        :return: operation status & output log
        """
        # FOLDER CREATION TEST eg toto file on EMMC
        # The path of file or folder will be created, emmc|mnt_sdcard|mnt_sdcard_ext
        status = False

        # Generate file name(s)
        folder_name = binascii.b2a_hex(os.urandom(folder_name_length / 2))
        folder_name = src_path + folder_name

        # Check if sdcard requested, if requested, mount it
        self._auto_mount_external_drive(folder_name)

        # create folder(s)
        self._logger.info("Create folder %s ..." % folder_name)
        self.create_folder(folder_name)

        self._logger.info("Check folder creation ...")
        status, output = self.exist(folder_name)
        if not status:
            msg = "Folder %s does not exists !" % folder_name
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        # delete folder
        self.delete(folder_name)

        if not status:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, output)

        # If operations succeed, return True
        return status

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

        :param folder: destination support on DUT where the file will be created.
        By default, the support is eMMC
        :type folder: str
        """
        self._logger.debug("create_custom_size_user_data : Filename : %s" % filename)

        if not filename or not isinstance(filename, (str, unicode)):
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect filename : unable to create a user data in the device")
        if not size_ko or size_ko <= 0 or not str(size_ko).isdigit():
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect file size : unable to create a user data in the device")
        if not isinstance(random_data, bool):
            raise DeviceException(DeviceException.INVALID_PARAMETER, "incorrect random data mode : "
                                                                     "unable to create a user data in the device")

        if folder is None:
            folder = self._device.multimedia_path

        output_file = os.path.normpath(folder + '/' + filename).replace("\\", "/")

        if not isinstance(folder, (str, unicode)):
            raise DeviceException(DeviceException.INVALID_PARAMETER, "incorrect destination folder : "
                                                                     "unable to create a user data in the device")

        # Check if sdcard or usbdrive requested, if requested, mount it
        self._auto_mount_external_drive(output_file)

        if random_data:
            # this command will create size_mo blocks of 1024 bytes
            # Each block will contain random data (if=/dev/urandom)
            cmd = "dd if=/dev/urandom of=%s bs=1024 count=%d" % (output_file, size_ko)
        else:
            # this command will create ONE block of ONE byte, at the
            # last byte of the file (seek to size_mo mega-bytes - 1 byte)
            cmd = "dd if=/dev/zero of=%s bs=1 count=1 seek=$((%d*1024 -1))" % (output_file, size_ko)
        return self._internal_exec(cmd, self._uecmd_default_timeout)

    def create_folder(self, target):
        """
        Create a folder on target path.

        :type target: str
        :param target: the folder path to create

        :rtype: None
        """
        cmd = "mkdir -p %s" % target
        self._internal_exec(cmd)

    def set_file_permission(self, target_file, right):
        """
        wrapper for function chmod, can also take a value like from ls -l minus the first '-'
        e.g : rwxr--r--  , first 3 character is for owner ,then group and finally other.

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello

        :type right: str
        :param right: right to apply , like 777 or rw-r--r--

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        cmd = "chmod -R {0} {1}".format(right, target_file)
        self._internal_exec(cmd)

    def _auto_mount_external_drive(self, paths):
        """
        - Check if sdcard is present in given paths, if yes, mount sdcard.
        - Check if usbdrive is present in given paths, if yes, mount usbdrive.
        - If none of sdcard or usbdrive, it is not relative to external device,
          no partition needs to be mounted, just return

        :type paths: str
        :param paths: paths (eg. "/home/root/file1.txt", "/sdcard/file2.txt"
        :return: None
        """
        if self._device.get_sdcard_path() in str(paths):
            self._phone_system_api.mount_device(self.__sdcard_device_file, self._device.get_sdcard_path())

        if self._device.get_usbdrive_path() in str(paths):
            self._phone_system_api.mount_device(self.__usbdrive_device_file, self._device.get_usbdrive_path())

    def find_files(self, root_path, match_patern):
        """
        Find all files matching the patern in the I{root_path}
        :type root_path: str
        :param root_path: the folder path where searching
        :type match_patern: str
        :param match_patern: the file patern to find (case sensitive)

        :rtype tuple
        :return: (execution status, list of files)
        """
        cmd = "find {0} -maxdepth 1 -name \"{1}\"".format(root_path, match_patern)
        return self._internal_exec(cmd)

    def get_last_sd_insert(self):
        """
        parse the dmesg to extract the last pluged sdx device.
        :rtype: str
        :return: path to the block device node (ie. : /dev/sdx )
        """
        ret_sd = ""
        regex = re.compile('\[.*\d*\.\d*\]\s*(sd[a-z]):(?: sd[a-z][1-9])*')
        result, msg = self._internal_exec("dmesg|tail -n20", use_uart=True)
        if not result:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to get the dmesg : %s" % msg)
        res = regex.search(msg, re.MULTILINE)
        # get the last occurence if multiple
        if res is not None:
            ret_sd = "/dev/{0}".format(res.group(res.lastindex))
            # check if the device allways plugged
            if not self.exist(ret_sd):
                ret_sd = ""
        return ret_sd

    def create_partition(self, block_device, type="primary", start=0, end=1024):
        """
        create a partition on the I{block_device} starting at I{start} ending at I{end}.
        :type block_device: str
        :param block_device: device of type block where to create the partition.
        :type type: str
        :param type: type of the partition. possible values are "primary", "logical" or "extended"
        :type start: int
        :param start: start index of the partition in MB
        :type end: int
        :param end: end index of the partition in MB

        :raise INTERNAL_EXEC_ERROR: if command execution on the DUT fail.

        :rtype: int
        :return: partition number
        """
        if type not in ("primary", "logical", "extended"):
            raise DeviceException(DeviceException.INVALID_PARAMETER, "incorrect partition type")
        cmd = "parted -s {0} -a optimal mkpart {1} {2} {3}".format(block_device, type, start, end)
        res, msg = self._internal_exec(cmd, use_uart=True)
        if not res:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to create partition : %s" % msg)
        cmd = "parted -s -m {0} print".format(block_device)
        res, msg = self._internal_exec(cmd, use_uart=True)
        if not res:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to list partitions : %s" % msg)
        msg = msg.rstrip()
        out = msg.split('\n')
        part = out.pop()
        return int(part.split(':')[0])

    def delete_partition(self, block_device, partition_number=0):
        """
        delete the partition number I{partition_number} of the specified I{block_device}
        a partition number of 0 will erase all the partition

        :type block_device: str
        :param block_device: device of type block where to delete the partition.
        :type partition_number: int
        :param partition_number: the partition number to delete. 0 will erase all partition

        :raise INTERNAL_EXEC_ERROR: if command execution on the DUT fail.

	:rtype: None
        """
        if partition_number == 0:
            cmd = "dd if=/dev/zero of={0} bs=512 count=1".format(block_device)
            res, msg = self._internal_exec(cmd, use_uart=True)
            if not res:
                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to erase MBR : %s" % msg)
            cmd = "echo w | fdisk {0}".format(block_device)
            res, msg = self._internal_exec(cmd, use_uart=True)
            if not res:
                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to write new MBR : %s" % msg)
        else:
            cmd = "parted -s {0} rm {1}".format(block_device, partition_number)
            res, msg = self._internal_exec(cmd, use_uart=True)
            if not res:
                raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to remove partition : %s" % msg)

    def format_partition(self, block_device, partition_number, fs_type):
        """
        format the I{partition_number} of the I{block_device} with the I{fs_type}

        :type block_device: str
        :param block_device: device of type block where to format the partition
        :type partition_number: int
        :param partition_number: partition number to format
        :type fs_type: str
        :param fs_type: file system type

	:rtype: None
        """
        fsdict = {"ext2": "mkfs.ext2 -q",
                  "ext3": "mkfs.ext3 -q",
                  "ext4": "mkfs.ext4 -q",
                  "msdos": "mkfs.msdos",
                  "vfat": "mkfs.vfat -F 32"}
        if not fsdict.has_key(fs_type):
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Unknown file system type")
        cmd = "{0} {1}{2}".format(fsdict[fs_type], block_device, partition_number)
        res, msg = self._internal_exec(cmd, use_uart=True)
        if not res:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to create partition : %s" % msg)
        cmd = "sync"
        res, msg = self._internal_exec(cmd, use_uart=True)
        if not res:
            raise DeviceException(DeviceException.INTERNAL_EXEC_ERROR, "Unable to execute sync : %s" % msg)
