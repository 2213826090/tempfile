#-------------------------------------------------------------------------------
# @copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.

# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.

# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.

# @organization: INTEL MCG PSI
# @summary: <enter_summary_details_here>
# @since: 5/28/14
# @author: nbrissox
#-------------------------------------------------------------------------------
import os
import sys

path = os.path

__DIR__ = path.abspath(path.dirname(__file__))
__LOGICAL_TOOLS_DIR__ = path.abspath(path.join(__DIR__, '_Tools'))

if not __LOGICAL_TOOLS_DIR__ in sys.path:
    sys.path.append(__LOGICAL_TOOLS_DIR__)

# noinspection PyUnresolvedReferences
from launcher import main as launch


if __name__ == '__main__':

    # All Css images are imported from relative path, thus we need to be in the very same conditions
    # as running it from its logical location!
    os.chdir(path.join(__LOGICAL_TOOLS_DIR__, 'launcher'))
    # Running
    sys.exit(launch.main())
