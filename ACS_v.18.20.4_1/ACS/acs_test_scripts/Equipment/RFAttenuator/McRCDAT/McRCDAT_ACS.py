"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL ACP
:summary: This file implements the class that integrates McRCDAT into ACS
:since: 2015-07-03
:author: emarchan
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.RFAttenuator.Interface.IRFAttenuator import IRFAttenuator
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.RFAttenuator.Scenario import LogicDataDebug

from McRCDAT import McRCDAT


class McRCDAT_ACS(EquipmentBase, McRCDAT, IRFAttenuator):

    def __init__(self, bench_name, eqt_model, eqt_dic, bench_dic):
        EquipmentBase.__init__(self, bench_name, eqt_model, eqt_dic)
        self._host = bench_dic.get_param_value("IP")
        McRCDAT.__init__(self, self._host)
        self.set_logger(self._logger)

    def _log_and_raise_failure(self, msg=""):
        """
        Logs an error and raises an exception in case of failure.

        :type msg: string
        :param msg: Error message for the logging.
        """
        self._log.error(msg)
        raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
