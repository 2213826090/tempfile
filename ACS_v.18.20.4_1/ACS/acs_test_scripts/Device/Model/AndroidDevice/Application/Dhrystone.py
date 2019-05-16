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
:summary: This script implements dhrystone benchmark
:since: 16/04/2013
:author: pbluniex
"""
import os
import re
from acs_test_scripts.Device.Model.AndroidDevice.Application.IBinary import IBinary
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
import hashlib


class Dhrystone(IBinary):

    """
    Dhrystone benchmark implementation
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IBinary.__init__(self, device)
        self.is_lower_better = False
        self._path_ref = os.path.join("BENCHMARKS", "DHRYSTONE")
        self._results = {"score": []}
        self.__threads_nb = 1

        self.__verif_md5 = {"6f9c2297ece9880a58c37c4c4627ad02":
                                "dhrystone_2-2-64b-E_GNU-Linux-x86-64_icc-ICC-15-0-0-20140723-opts-O3-mP3OPT-bair-3-xatom-sse4-2-DOPTSTRCPY-1-DOPTSTRCMP-4-DVERBOSE-1-DOPT-DIV",
                            "4c7050f80cbab3934cee28a02e0f852b":
                                "dhrystone_2-2-32b-E_GNU-Linux-x86-64_icc-ICC-14-0-0-20130529-opts-Ofast-mP3OPT-bair-3-static-xatom-sse4-2-DOPTSTRCPY-1-DOPTSTRCMP-4-DVERBOSE-1-DOPT-DIV",
                            "c23dac2f5ab60ad5b43e6f6102f5793d":
                                "FT/pnp/Applications\BENCHMARKS\DHRYSTONE\dhrystone"}

    def _fetch_result(self):
        """
        Get score for dhrystone
        """
        result = self._fetch_file()

        pattern = "DMIPS for main thread:\s+(?P<score>[0-9\.]+)"
        match = re.finditer(pattern, result)
        if match:
            scores = [float(item.group("score")) for item in match]
            self._results["score"].append(min(scores) * self.__threads_nb)
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can not fetch Dhrystone result")

    def __md5sum(self, filepath):
        """
        Returns md5sum
        """
        BLOCKSIZE = 65536
        hasher = hashlib.md5()
        with open(filepath, 'rb') as afile:
            buf = afile.read(BLOCKSIZE)
            while len(buf) > 0:
                hasher.update(buf)
                buf = afile.read(BLOCKSIZE)
        return hasher.hexdigest()

    def __verify_binary_version(self):
        """
        """
        self._logger.debug("Check MD5 of binary file")

        md5_res = self.__md5sum(self._application_uri)

        self._logger.debug("Binary file md5: %s" % md5_res)
        if md5_res in self.__verif_md5.keys():
            file_version = self.__verif_md5[md5_res]
            self._logger.debug("The file version is: %s" % str(file_version))
        else:
            string = "The binary file %s does not correspond to a valid " % self._application_uri \
                + "Dhrystone file version. Please check."
            raise AcsConfigException(AcsConfigException.INVALID_TEST_CASE_FILE, string)

    def post_install(self):
        """
        Post installation configurations
        """
        self._logger.debug("Dhrystone - post_install begin")

        self.__verify_binary_version()

        self.__threads_nb = int(self._phonesystem.get_cpu_info(["siblings"])[0])
        self._command = "./%s & " % self._benchmark_name
        for _ in range(1, self.__threads_nb):
            self._command += "./%s & " % self._benchmark_name

        IBinary.post_install(self)
