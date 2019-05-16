# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

"""
@summary: SpecialActionsImpl class
@since: 06/12/2015
@author: Zhao, Xiangyi
"""

from testlib.util.common import g_common_obj
import hashlib
import sys

class Md5Impl(object):

    """Md5Impl"""

    def __init__(self):
        self._device = g_common_obj.get_device()
        super.__init__

        self.actions = []

    def setup(self):
        """resource setup"""

    def make_md5(self, filename):
        """make md5 for file
        """
        print "make md5 file"
        if len(sys.argv) != 2:
            sys.exit('Usage: %s file' % sys.argv[0])

        m = hashlib.md5()
        with open(filename, 'rb') as fp:
            while True:
                blk = fp.read(4096)  # 4KB per block
                if not blk: break
                m.update(blk)
            print m.hexdigest(), filename
        return m.hexdigest()

    def make_md5_jumpbyte(self, byte, filename):
        """make md5 for file and jump byte at begin
        """
        print "make md5 file and jump byte at begin"
        if len(sys.argv) != 2:
            exit
#             sys.exit('Usage: %s file' % sys.argv[0])

        m = hashlib.md5()
        with open(filename, 'rb') as fp:
            fp.seek(byte)
            while True:
                blk = fp.read(4096)  # 4KB per block
                if not blk: break
                m.update(blk)
            print m.hexdigest(), filename
        return m.hexdigest()

make_md5 = Md5Impl()
