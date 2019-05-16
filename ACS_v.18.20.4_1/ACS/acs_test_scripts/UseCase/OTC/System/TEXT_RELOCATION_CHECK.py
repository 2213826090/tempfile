"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:since: 2014-06-05
:author: agdobrex
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.PathManager import Folders
from subprocess import check_output, CalledProcessError
import subprocess
import os, errno
import shutil
import re


class TextRelocationCheck(UseCaseBase):

    """
        This test checks the libraries from /system/lib of the DUT for eventual security risks, using pax-utils.
         Test case steps:
            1. adb pull /system/libs/
            2. scans all the libs using the script:
                "scanelf -qT -R out/target/product/gmin/system/lib/* | grep -v libdvm.so | grep -v libart.so |
                 grep -v libc_malloc_debug_leak.so | grep -v libc_malloc_debug_qemu.so | grep -v libhoudini.so |
                  grep -v libgcam.so | grep -v libvorbisidec.so | grep -v libcrypto.so | grep -v libaudioresample.so"
            3. if all the libs founded by the script are in the white list, the test case is 'pass'; otherwise 'fail'
                - this is checked using the norm that if the above script produces an output, the test case is fail
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._log_dir = "TEMP"
        self._logger.info("The log directory returned is: " + self._log_dir)
        self._temp_folder_name = 'temporary_libs'
        self._temp_folder_path = os.path.join(Folders.REPORTS, self._log_dir, self._temp_folder_name)
        self._logger.info("The temporary directory is: " + str(self._temp_folder_path))
        self._timeout = 60000
        self._white_list_library = ['libdvm.so', 'libart.so', 'libc_malloc_debug_leak.so', 'libc_malloc_debug_qemu.so',
                              'libhoudini.so', 'libgcam.so', 'libvorbisidec.so', 'libcrypto.so', 'libaudioresample.so']

        #get the white list libraries
        _white_list_library_parameter = self._tc_parameters.get_param_value("white_list_libraries")
        self._white_list_library = filter(None, re.split('\s+', _white_list_library_parameter))

        self._scanelf_path = ""
        self._pre_reboot_device = False
        self._post_reboot_device = False
        self._post_reboot_nok_device = False
        if self._device.get_config("DisableTcReboot", False, default_cast_type='str_to_bool'):
            self._pre_reboot_device = self._tc_parameters.get_param_value("PRE_REBOOT", default_value="False",
                                                                          default_cast_type="str_to_bool")
            # Reboot the board after test execution
            # AKA "POST_REBOOT" in ATF
            self._post_reboot_device = self._tc_parameters.get_param_value("POST_REBOOT", default_value="False",
                                                                           default_cast_type="str_to_bool")
            # Reboot the board after test execution if test verdict is failure
            # AKA "POST_REBOOT_NOK" in ATF
            self._post_reboot_nok_device = self._tc_parameters.get_param_value("POST_REBOOT_NOK", default_value="False",
                                                                               default_cast_type="str_to_bool")


    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)

        """
            Check to see if the report directory was created and also create a temporary directory.
        """
        if not os.path.exists(self._temp_folder_path):
            try:
                os.makedirs(self._temp_folder_path)
            except OSError as exception:
                if exception.errno != errno.EEXIST:
                    raise AcsConfigException("Could not create a temporary directory under " + self._temp_folder_path)

        """
            Pull all the files from /system/lib from the DUT to the newly created folder.
        """
        command = "adb pull /system/lib " + self._temp_folder_path
        self._logger.info("Pulling systems libraries from DUT.")
        self._device.pull("/system/lib", self._temp_folder_path, timeout=self._timeout)

        """
            Check if all the files have been pulled.
        """
        exists = False
        for (dirpath, dirname, files) in os.walk(self._temp_folder_path):
            for file in files:
                if file.find(".so") != -1:
                    exists = True
                    break
        if not exists:
            return Global.FAILURE, "System libs could not be pulled from the device."

        """
            Check to see if pax-utils is installed on the linux machine.
        """
        output = ""
        try:
            output = check_output(["which", "scanelf"], stderr=subprocess.STDOUT)
        except CalledProcessError:
            raise AcsConfigException("Missing module pax-utils. Please install this package and retry test.")

        if not "scanelf" in output:
            raise AcsConfigException("Missing module pax-utils. Please install this package and retry test.")
        self._scanelf_path = output.rstrip('\n')


        return Global.SUCCESS, ""

    def run_test(self):
        """
            Here we run the command specified on the files downloaded.
        """

        path = self._temp_folder_path + "/*"    # add star to end of path in order to get all files

        #first do the
        print self._scanelf_path + ' -qT -R ' + path

        ps = subprocess.Popen(self._scanelf_path + " -qT -R " + path, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              shell=True)
        output = ps.communicate()[0]

        # the above shell command is similar to this line of code
        output_lines = output.split('\n')
        output_lines = filter(None, output_lines)
        output_lines_aux = []

        for line in output_lines:
            white_listed = False
            for lib in self._white_list_library:
                if lib in line:
                    white_listed = True
                    break
            if not white_listed:
                output_lines_aux.append(line)

        if output_lines_aux:
            return Global.FAILURE, "The command produces the following output: " + str(output_lines_aux)

        return Global.SUCCESS, ""

    def tear_down(self):
        """
            Here we delete the temporary folder.
        """

        if os.path.exists(self._temp_folder_path):
            shutil.rmtree(self._temp_folder_path)

        return Global.SUCCESS, ""
