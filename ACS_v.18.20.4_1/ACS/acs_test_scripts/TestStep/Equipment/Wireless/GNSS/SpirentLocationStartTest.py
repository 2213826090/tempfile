"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements a Test Step to run the TTman Client for GNSS tests with Spirent Equipment
@since 21/08/2014
@author: obouzain
"""

from acs_test_scripts.TestStep.Equipment.Wireless.GNSS.SpirentBase import SpirentBase
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
import os
import re
from ErrorHandling.DeviceException import DeviceException
import shutil

class SpirentLocationStartTest(SpirentBase):
    """
    Implements the Spirent test step for GNSS
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        SpirentBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.local_computer = None
        self._em = EquipmentManager()

    def run(self, context):

        """
        Runs the test step
        @type context: TestStepContext
        @param context: test case context
        """
        SpirentBase.run(self, context)
        # Get parameters
        computer = str(self._pars.computer)
        self.local_computer = self._em.get_computer(computer)
        host = str(self._pars.host).lower()
        port = str(self._pars.port)
        local_address = self.local_computer.get_host_on_test_network()
        local_port = self._pars.local_port
        project = str(self._pars.project)
        campaign_file = str(self._pars.campaign_file)
        test_module = str(self._pars.test_module)
        test_id = str(self._pars.test_id)

        # Launch the client
        jar_file = os.path.normpath(os.path.join(os.getcwd(), "..//", "..//", "acs_test_scripts//", "Lib//", "TTmanClient.jar"))
        cmd_line = 'java -jar "%s" "%s" "%s" "10.237.164.9" "%s" %s %s "%s" "%s"' % (jar_file, host, port, local_port, project, campaign_file, test_module, test_id)
        self._logger.info("Running %s" % cmd_line)

        # Stock all the result of the execution of the command in a variable result
        result = str(self.local_computer.run_cmd(cmd_line))
        self._logger.info (result)
        verdict = re.search('FINISHED verdict pass', result, re.MULTILINE)

        log_path = self._device.get_report_tree().get_report_path()
        path = os.getcwd()
        self._logger.debug("path %s" % path)
        for cur_file in os.listdir(path):
            if  cur_file.endswith(".tlz"):
                local_log = cur_file
                # copy
                created = log_path + "\GNSS_Report"
                os.mkdir(created, 0755);
                self._logger.info("log file created %s" % created)
                shutil.move(local_log, created)

        # If the pass in not found in the result the test is failed
        if verdict is None:
            msg = "Test Failed"
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

