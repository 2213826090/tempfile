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
:summary: This script implements the interface to unitary actions for
miscellaneous features.
:since: 08/08/2010
:author: szhen11
"""
from ErrorHandling.DeviceException import DeviceException

# pylint: disable=W0613


class IFile():

    """
    Abstract class that defines the interface to be implemented
    by device system operations handling sub classes.

    All method that shall be redefined in sub-classes raise a
    I{UECommanddException} error.
    """

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        pass

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def rename(self, oldname, newname):
        """
        Rename a file.

        :type new_name: str
        :param new_name: the new name
        :type old_name: str
        :param old_name: the old name

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def exist(self, target_file):
        """
        Test if a file exist.

        :type target_file: str
        :param target_file: path of the checked file, eg: /ect/hello

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def read_file(self, target_file):
        """
        Returns the contain of given file.

        :type target_file: str
        :param target_file: path of the file , eg: /ect/hello

        :rtype: tuple
        :return: (boolean operation success, error msg)
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def remount(self):
        """
        remount file system
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_file_owner(self, file_name):
        """
        get the file owner and groupe.

        :type file_name: str
        :param file_name: name of the file (the absolute pat of the file)

        :rtype: tuple (str,str)
        :return: (owner, owner_group) , these value can be equal to None if failed to be found
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_file_size(self, target):
        """
        Get file size.

        :type target: str
        :param target: the file path to get size

        :rtype: int
        :return: the file size in block size 1

        :raise AcsBaseException in case of wrong usage
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def create_folder(self, target):
        """
        Create a folder on target path.

        :type target: str
        :param target: the folder path to create

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def get_last_sd_insert(self):
        """
        parse the dmesg to extract the last pluged sdx device.
        :rtype: str
        :return: path to the block device node (ie. : /dev/sdx )
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def create_partition(self, block_device, type="primary", start=0, end=1024):
        """
        create a partition on the I{block_device} starting at I{start} ending at I{end}.
        :type block_device: str
        :block_devicedevice: block_device of type block where to create the partition.
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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def delete_partition(self, device, partition_number=0):
        """
        delete the partition number I{partition_number} of the specified I{device}
        a partition number of 0 will erase all the partition

        :type device: str
        :param device: device of type block where to delete the partition.
        :type partition_number: int
        :param partition_number: the partition number to delete. 0 will erase all partition

        :raise INTERNAL_EXEC_ERROR: if command execution on the DUT fail.

        :rtype: None
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def retrieve_size_from_filename(self, filename):
        """
        Try to retrieve file size from the filename.
        Filename must follow a specific format:
        [name][size (integer)][unit (k, ko, kB, m, mo, mB, g, go, gB)][extension (optional)]
        example : put500MB.zip

        :return: Size in kilo-Byte
        :rtype: int
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

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
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
