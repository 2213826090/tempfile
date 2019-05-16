# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 25/02/14
:author: kturban
"""
from mock import patch
from mock import Mock
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Equipment.Tool.GetArtifact import GetArtifact
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException


class GetArtifactTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._eq_man_patch = patch("acs_test_scripts.TestStep.Equipment.Tool.GetArtifact.EquipmentTestStepBase.run")
        self._eq_man_patch.start()
        self._osmakedir_patch = patch("acs_test_scripts.Utilities.HttpDownloaderUtil.os.makedirs")
        self._osmakedir_patch.start()

    def tearDown(self):
        UTestTestStepBase.tearDown(self)
        self._osmakedir_patch.stop()
        self._eq_man_patch.stop()

    def test_put_in_cache_ko(self):
        """
            Check that unexisting app path will raise error
        """
        teststep = GetArtifact(None, None, None, Mock())
        teststep._equipment_manager = Mock(spec=EquipmentManager)
        teststep._pars.eqt = "ARTIFACT_MANAGER"
        teststep._pars.artifact_source = "source_artifact"
        teststep._pars.artifact = "my_artifact"
        teststep._pars.transfer_timeout = 10
        mock_get_artifact = teststep._equipment_manager.get_artifact_manager
        mock_get_artifact.return_value.get_artifact.return_value = ""

        with self.assertRaises(AcsToolException):
            teststep.run(self._context)
        mock_get_artifact.return_value.get_artifact.assert_called_once_with(artifact_name="my_artifact",
                                                                            artifact_root_uri="source_artifact",
                                                                            transfer_timeout=10)

    @patch.object(EquipmentManager, "get_artifact_manager")
    def test_put_in_cache_ko_blocked(self, mock_get_artifact):
        """
            Check that unexisting app path will raise error
        """
        teststep = GetArtifact(None, None, None, Mock())
        teststep._equipment_manager = Mock(spec=EquipmentManager)
        teststep._pars.artifact_source = "source_artifact"
        teststep._pars.artifact = "my_artifact"
        teststep._pars.eqt = "ARTIFACT_MANAGER"
        teststep._pars.transfer_timeout = 10
        mock_get_artifact = teststep._equipment_manager.get_artifact_manager

        mock_get_artifact.return_value.get_artifact.side_effect = \
            AcsConfigException(AcsConfigException.INVALID_PARAMETER, "wrong params!")
        with self.assertRaises(AcsConfigException):
            teststep.run(self._context)
            mock_get_artifact.return_value.get_artifact.assert_called_once_with(artifact_name="my_artifact",
                                                                                artifact_root_uri="source_artifact",
                                                                                transfer_timeout=10)

    @patch.object(EquipmentManager, "get_artifact_manager")
    def test_put_in_cache_ok(self, mock_get_artifact):
        """
            Check that unexisting app path will raise error
        """
        teststep = GetArtifact(None, None, None, Mock())
        teststep._equipment_manager = Mock(spec=EquipmentManager)
        teststep._pars.artifact_source = "source_artifact"
        teststep._pars.artifact = "my_artifact"
        teststep._pars.eqt = "ARTIFACT_MANAGER"
        teststep._pars.transfer_timeout = 10
        mock_get_artifact = teststep._equipment_manager.get_artifact_manager
        mock_get_artifact.return_value.get_artifact.return_value = "/path/to/my/artifact"
        teststep.run(self._context)
        mock_get_artifact.return_value.get_artifact.assert_called_once_with(artifact_name="my_artifact",
                                                                            artifact_root_uri="source_artifact",
                                                                            transfer_timeout=10)
