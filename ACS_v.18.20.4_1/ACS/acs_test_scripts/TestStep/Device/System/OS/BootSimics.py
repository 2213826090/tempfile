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

"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import os

class BootSimics(DeviceTestStepBase):
    """
    Boots VP Simics image
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._em = self._factory.create_equipment_manager()
        self._computer = self._em.get_computer("COMPUTER1")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        self._logger.info("BootSimics: Run")
        DeviceTestStepBase.run(self, context)

        simics_exec = self._pars.simics_exec_path
        simics_model = self._pars.simics_model_path
        boot_gui = self._pars.boot_with_gui

        self._logger.info("executable path: {0}".format(simics_exec))
        self._logger.info("script path: {0}".format(simics_model))
        self._logger.info("boot_with_gui set to: %r" %boot_gui)

        # Check if simics executable and model are valid
        if not os.path.isfile(simics_exec):
            msg = "Simics executable not found: " + simics_exec
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not os.path.isfile(simics_model):
            msg = "Simics model not found: " + simics_model
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        cmd = "'" + r"%s" %simics_exec + "' '" + r"%s" %simics_model + "'" + " -e run"
        if not boot_gui:
            cmd = cmd + " -no-gui"

        self._logger.info("booting simics with following cmd ...")
        self._logger.info(cmd)
        ret_val = self._computer.run_cmd(cmd, timeout=-1)
        self._logger.debug("ret_code: {0} ".format(ret_val["ret"]))
        if ret_val["ret"] is not None:
            self._logger.debug("status: {0} ".format(ret_val["std"]))
            self._logger.debug("err_msg:{0} ".format(ret_val["err"]))
            msg = "Error launching simics"
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info("BootSimics: Done")
