"""

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

@summary: This file implements a Test Step to create a random binary file
@since 8 July 2014
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV
"""

import os
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsBaseException import AcsBaseException
import random
import struct
import tempfile


class CreateRandomBinaryFileHost(TestStepBase):
    """
    Install a file on a host
    """
    def run(self, context):
        """
        Create a random file

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        destination_path = self._pars.destination_path # Destination path
        if destination_path == '[TEMP]':
            destination_path = tempfile.mkdtemp()
        if self._pars.rand_file_size_min is not None:
            if self._pars.rand_file_size_min >= self._pars.rand_file_size:
                self._logger.error("{0}:  RAND_FILE_SIZE_MIN of {1} cannot be <= to RAND_FILE_SIZE, which is = {2}".format(self._pars.id, self._pars.rand_file_size_min, self._pars.rand_file_size))
                msg = "{0}:  RAND_FILE_SIZE_MIN of {1} cannot be <= to RAND_FILE_SIZE, which is = {2}".format(self._pars.id, self._pars.rand_file_size_min, self._pars.rand_file_size)
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, msg)
            else:
                rand_file_size_min = self._pars.rand_file_size_min
        else:
            rand_file_size_min = 0
        if rand_file_size_min > 0:
            rand_file_size = int(random.randrange(rand_file_size_min,self._pars.rand_file_size + 1))
        else:
            rand_file_size = self._pars.rand_file_size
        rand_file_seed = self._pars.rand_file_seed
        rand_file_name = self._pars.rand_file_name

        self._logger.debug("%s:  rand_file_size = %d, rand_file_seed = %d, rand_file_name = %s "
                           % (self._pars.id, rand_file_size, rand_file_seed, rand_file_name))

        file_path = os.path.join(destination_path, rand_file_name)

        try:
            # Create the file
            self.create_rand_bin_file(file_path=file_path, file_size_mb=rand_file_size, seed=rand_file_seed)
        except Exception as e:
            raise AcsToolException(AcsToolException.DEFAULT_ERROR_CODE, str(e))


        # Check the file was created
        if os.path.exists(file_path):
            self._logger.info("{0}:  Created {1} to {2}".format(self._pars.id, rand_file_name, destination_path))
            context.set_info(self._pars.ctx_destination_path, file_path)
        else:
            msg = "{0}:  Failed to create {1} to {2}".format(self._pars.id, rand_file_name, destination_path)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    @staticmethod
    def create_rand_bin_file(file_path, file_size_mb, seed=None):
        """
           Creates a file filled with random data at file_path with size file_size. If seed
           is provided, that seed will be used to create the random data to allow
           repeatability. Returns the seed used to create the data.

           :param file_path: The path to where the random data file should be created.
           :param file_size_mb: The size of the file to create in megabytes.
           :param seed: A seed to use for generating the random data. If no seed is
               provided, a random seed will be created and returned.
        """

        # If no seed was provided create a seed with 50 random characters
        if seed is None:
            seed_ascii_bucket = range(ord('A'),ord('Z')+1)
            seed_ascii_bucket.extend(range(ord('0'),ord('9')+1))
            seed = ""
            for i in range(50):
                seed += chr(random.choice(seed_ascii_bucket))
        random.seed(seed)
        rand_file = open(file_path, 'wb')
        # It would take way too long to make the entire file random, considering
        #  it takes ~10 seconds per MB. Lets create 1 MB of random data and repeat it.
        rand_data = ""
        for i in range(250*1024):
            rand_data += struct.pack('I', random.randint(0,0xFFFFFFFF))
        # Repeatedly write our data to the file
        for i in range(file_size_mb):
            rand_file.write(rand_data)
        rand_file.close()

