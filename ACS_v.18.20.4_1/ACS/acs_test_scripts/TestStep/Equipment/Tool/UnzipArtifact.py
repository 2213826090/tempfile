"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC ANDROID QA
:description: TestStep to un-zip downloaded artifacts before pushing them onto the device.
:since: 6/3/14
:author: mmaracix
"""
import os

from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException



class UnzipArtifact(EquipmentTestStepBase):
    """
    Unzip files previously downloaded from the Artifactory
    via the ArtifactManager class methods.
    This TestStep is based on the unzip_artifact method from ArtifacManager.py
    It takes as parameters the zip file location as a CONTEXT variable from GET_ARTIFACT and the output location
    for the unzipped files.
    The output is the same name as the Archive name, with a "/" at the end. This can be modified.
    The output is stored as a CONTEXT variable that can be passed to an INSTALL_FILE TestStep.
    """

    def run(self, context):
        EquipmentTestStepBase.run(self, context)

        try:
            artifact_manager = self._equipment_manager.get_artifact_manager(self._pars.eqt)
            if os.path.exists(self._pars.extract_location):
                artifact_unziped_path = artifact_manager.unzip_artifact(self._pars.stored_file_path, extract_to=self._pars.extract_location)
            else:
                artifact_unziped_path = artifact_manager.unzip_artifact(self._pars.stored_file_path)
        except AcsConfigException:
            raise

        if not artifact_unziped_path:
            err_msg = "Could not extract the contents of the archive"
            raise AcsToolException(AcsToolException.OPERATION_FAILED, err_msg)
        else:
            context.set_info(self._pars.extract_location, artifact_unziped_path)
            self.ts_verdict_msg = "File successfully unzipped in cache as {0}".format(artifact_unziped_path)

