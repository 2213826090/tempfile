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
:summary: Compute checksum of files in folder reports
:author: nprecigx

"""
# pylint: disable=W0621
import os
from Core.Report.ACSLogging import LOGGER_FWK
from Core.PathManager import ACS_SRC_DIR
from UtilitiesFWK.Utilities import compute_checksum

CHECK_SUM_FILES = "reports_compute"
FEATURE_FILE_ACTIVATION = ".secure_file"


class ComputeReportChecksum:

    """
    Compute checksum
    """
    @staticmethod
    def compute(campaign_report_path):
        """
        Compute checksum for all files in the given path and save it in a file
        :type campaign_report_path: str
        :param campaign_report_path: The path to the folder to compute checksum.
        """
        # Compute checksum only if FEATURE_FILE_ACTIVATION is present
        if ComputeReportChecksum.compute_checksum_activated():
            try:
                hash_code = compute_checksum(campaign_report_path)
            except Exception as error:
                LOGGER_FWK.error("Error while compute checksum (%s)" % str(error))
            try:
                if hash_code:
                    with open(os.path.join(campaign_report_path, CHECK_SUM_FILES), 'w') as file_:
                        file_.write(str(hash_code))
                else:
                    LOGGER_FWK.error("No checksum computed")
            except Exception as error:
                LOGGER_FWK.error("Error while write in file (%s)" % str(error))

    @staticmethod
    def compute_checksum_activated():
        """
        Check if the file FEATURE_FILE_ACTIVATION is present
        :rtype bool
        :param True if the file is present
        """
        if os.path.isfile(os.path.join(ACS_SRC_DIR, FEATURE_FILE_ACTIVATION)):
            return True
        else:
            return False
