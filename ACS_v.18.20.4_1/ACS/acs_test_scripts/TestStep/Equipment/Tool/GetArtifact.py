"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements a Test Step to get a local/distant artifact into local cache directory
:since:04/03/2014
:author: kturban
"""

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global


class GetArtifact(EquipmentTestStepBase):

    """
    Retrieve a local/distant artifact and store it in artifact cache folder
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        EquipmentTestStepBase.run(self, context)

        try:
            artifact_manager = self._equipment_manager.get_artifact_manager(self._pars.eqt)
            local_artifact = artifact_manager.get_artifact(artifact_name=self._pars.artifact,
                                                           artifact_root_uri=self._pars.artifact_source,
                                                           transfer_timeout=self._pars.transfer_timeout)
        except AcsConfigException:
            raise

        if not local_artifact:
            error_msg = "Cannot get the file on the local host"
            raise AcsToolException(AcsToolException.OPERATION_FAILED, error_msg)
        else:
            context.set_info(self._pars.stored_file_path, local_artifact)
            self.ts_verdict_msg = "File successfully stored in cache as {0}".format(local_artifact)
