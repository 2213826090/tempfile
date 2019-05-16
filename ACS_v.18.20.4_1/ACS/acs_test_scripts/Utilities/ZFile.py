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
:summary: ZFile, a class to handle zip and unzip
:author: Xiaofei Wan
"""

import zipfile
import os.path

# pylint: disable=missing-docstring, invalid-name


class ZFile(object):

    def __init__(self, filename, zipmode='r'):
        self.zfile = zipfile.ZipFile(filename, zipmode, zipfile.ZIP_DEFLATED)

    def close(self):
        self.zfile.close()

    def extract_to(self, path):
        for p in self.zfile.namelist():
            self.extract(p, path)

    def extract(self, filename, path):
        if not filename.endswith('/'):
            newName = os.path.join(path, filename)
            directory = os.path.dirname(newName)
            if not os.path.exists(directory):
                os.makedirs(directory)
            file(newName, 'wb').write(self.zfile.read(filename))

    def compress(self, filename):
        if os.path.isfile(filename):
            self.zfile.write(filename)
        elif os.path.isdir(filename):
            path = filename
            for root, _dirs, files in os.walk(path):
                for f in files:
                    self.zfile.write(os.path.join(root, f), os.path.join(root, f).replace(path + os.sep, ''))
