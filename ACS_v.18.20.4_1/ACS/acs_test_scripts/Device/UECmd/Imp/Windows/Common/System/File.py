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
:summary: This file implements the File system interaction UEcmd for Windows device
          This is for testing file system not doing simple W/R action.
:since: 10/28/2011
:author: wchen61, vgomberx
"""

import os
import re
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.Device.UECmd.Imp.Windows.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.System.IFile import IFile
from ErrorHandling.DeviceException import DeviceException


class File(Base, IFile):

    """
    :summary: File UEcommands operations for Windows platforms
    """

    def __init__(self, device):
        """
        Constructor.
        """

        Base.__init__(self, device)
        IFile.__init__(self, device)
        self._phone_system = device.get_uecmd("PhoneSystem")

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
        else:
            output_file = os.path.normpath(filename).replace("\\", "/")

        if not isinstance(folder, (str, unicode)):
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                  "incorrect destination folder : "
                                  "unable to create a user data in the device")

        # create output folder if needed
        if folder not in ("/", "", " "):
            self.device._internal_exec("mkdir " + folder)

        # this command will create ONE block of ONE byte, at the
        # last byte of the file (seek to size_mo mega-bytes - 1 byte)
        cmd = "fsutil file createnew %s %s" % (output_file, size_ko)
        self._internal_exec(cmd)

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
                self._logger.debug(
                    "File has an incorrect size (%d) : it will "
                    "be replaced by a new one with the wanted size."
                    % file_size)

        # create or replace file
        self._logger.debug("Create file %s of %d MB" % (file_path, size_ko))
        self.create_custom_size_user_data(file_path, size_ko * 1024, folder="")
