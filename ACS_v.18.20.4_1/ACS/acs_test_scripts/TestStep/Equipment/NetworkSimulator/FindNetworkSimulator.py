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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step for getting an available network simulator
:author: jduran4x
:since 12/09/2014
:organization: INTEL QCTV
"""

from Core.TestStep.TestStepBase import TestStepBase
from Core.PathManager import Paths, Folders
from ErrorHandling.AcsConfigException import AcsConfigException
from lxml import etree as ET
import os


class FindNetworkSimulator(TestStepBase):
    CATALOG_FILE_PATH = os.path.join(Paths.TEST_SCRIPTS, Folders.CATALOGS,
                                     "Equipment", "Equipment_Catalog.xml")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        TestStepBase.run(self, context)
        self._logger.info("Search a Network Simulator with %s feature enabled" % self._pars.feature)
        self._logger.info("It will be stored in context key %s" % self._pars.eqt_name)
        # !!!!! WARNING  !!!!!
        # Test step Context not reset work around (to be removed when context will be reset)
        # Check if eqt name is already in context, if yes return as equipment already exist
        if self._pars.eqt_name in context.ref_dict():
            self._logger.warning(" %s equipment is already in context" % (self._pars.eqt_name))
            return
        # we get the values in the context that can be a network simulator name.
        # the network simulator name is a string
        context_values = [x for x in context.ref_dict().values() if type(x).__name__ == "str"]
        self._factory.create_equipment_manager()

        # open equipment catalog
        # get all equipments that embed the desired feature
        # perform a xpath query to get the list of nodes corresponding to the feature
        # the equipment name is the parent of the parent of the resulting nodes (strongly linked to catalog tree)
        eqt_cat = ET.parse(self.CATALOG_FILE_PATH)
        eqts = eqt_cat.xpath("//Parameter[@name=\"%s\" and @value=\"enable\"]" % self._pars.feature)
        if len(eqts) == 0:
            raise Exception("feature %s not supporter" % self._pars.feature)
        eqt_names = [x.getparent().getparent().attrib["name"] for x in eqts]
        benchconfig = self._global_conf.benchConfig.get_dict()
        # get the NS which embed the feature and is not already in use
        for ns in benchconfig.keys():
            eqt_param = self._global_conf.benchConfig.get_parameters(ns)
            if eqt_param.has_parameter("Model"):
                eqt_model = eqt_param.get_param_value("Model").replace("_VISA", "")
                eqt_name = eqt_param.get_name()
                if eqt_model in eqt_names and eqt_name not in context_values:
                    # found 1 matching equipment not already in use
                    context.set_info(self._pars.eqt_name, eqt_name)
                    self._logger.info("Found %s (%s)"
                                      % (eqt_model, eqt_name))
                    break
        else:
            raise AcsConfigException(AcsConfigException.UNKNOWN_EQT)
