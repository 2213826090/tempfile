""" PUPDR_common Basic functions for PUPDR tests implementation.
Offers a set of custom functions for PUPDR testing team.

"""

"""
==================================================================================================

(c)Copyright 2013, Intel Corporation All Rights Reserved.

The source code contained or described here in and all documents related to the source code
("Material") are owned by Intel Corporation or its suppliers or licensors. Title to the Material
remains with Intel Corporation or its suppliers and licensors. The Material contains trade secrets
and proprietary and confidential information of Intel or its suppliers and licensors. The Material
is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the
Material may be used, copied, reproduced, modified, published, uploaded, posted, transmitted,
distributed, or disclosed in any way without Intels prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted
to or conferred upon you by disclosure or delivery of the Materials, either expressly, by
implication, inducement, estoppel or otherwise. Any license under such intellectual property rights
must be express and approved by Intel in writing.


====================================================================================================
"""

import os
from Core.PathManager import Paths
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.AcsConfigException import AcsConfigException

LIBRARY_PATH_LIST = ""
exec_script_path = os.environ.get("MY_PATH")

possible_paths = [Paths.EXECUTION_CONFIG + "/ExtraLibs/Lib/pupdr"]
possible_paths.append(os.path.dirname(__file__))

LOGGER_TEST_SCRIPT.debug("PUPDR LIBRARY LOADER")

for p in possible_paths:
    p = os.path.abspath(p)
    if os.path.isdir(p) and "pupdr_common.py" in os.listdir(p):
        LIBRARY_PATH_LIST = p
        break

if LIBRARY_PATH_LIST == "":
    LOGGER_TEST_SCRIPT.error("No pupdr library found in this list of paths: {0}".format(possible_paths))
    raise AcsConfigException("No PUPDR library found")
else:
    pupdr_common = os.path.join(LIBRARY_PATH_LIST, "pupdr_common.py")
    LOGGER_TEST_SCRIPT.debug("Loading the following library file: {0}".format(pupdr_common))
    execfile(pupdr_common)
