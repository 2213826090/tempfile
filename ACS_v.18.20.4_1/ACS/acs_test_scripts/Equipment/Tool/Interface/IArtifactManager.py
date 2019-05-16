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
:summary: This file implements a ArtifactManager class to manage all binaries
:since:24/02/2014
:author: kturban
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException


class IArtifactManager(object):
    """
        Class IArtifactManager: virtual interface for artifact manager
    """
    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_artifact(self, artifact_name, artifact_source="", transfer_timeout=60 * 10):
        """
            Retrieve an artifact on the local host

            :param artifact_name: the name of artifact
            :type  artifact_name: str.

            :param artifact_source: the uri of artifact source
            :type  artifact_source: str.

            :param transfer_timeout: timeout to transfer the artifact on local host
            :type  transfer_timeout: int.

            :return: path to the downloaded artifact
            :rtype: str
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
