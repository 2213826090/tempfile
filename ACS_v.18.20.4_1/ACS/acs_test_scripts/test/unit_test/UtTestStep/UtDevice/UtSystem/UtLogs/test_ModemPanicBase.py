# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the base for unit tests related to modem panics.
:since: 2014-10-23
:author: emarchan

"""

from acs_test_scripts.test.unit_test.UtTestStep.UtDevice.UtSystem.UtLogs.test_SysDebugBase import test_SysDebugBase
from Core.TestStep.TestStepContext import TestStepContext
from lxml import etree


class test_ModemPanicBase(test_SysDebugBase):

    def setUp(self):
        test_SysDebugBase.setUp(self)


    def _create_fake_syslog_with_panic(self):
        """
        Simulates a real system log output with a modem panic
        """
        xmltree = self._create_fake_syslog_with_no_panic()
        xmlpanic = etree.Element("mpanic")
        xmlpanic.attrib["errno"] = "DUMMY_ERR"
        xmltree.append(xmlpanic)
        return xmltree

    def _create_fake_syslog_with_no_panic(self):
        """
        Simulates a real system log output without modem panic
        """
        xmltree = etree.Element("ModemPanics")
        return xmltree
