import json
import re
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class RemoveGoogleAccount(DeviceTestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        command = "adb shell ls /data/system/users/*/accounts.db"

        _, output = self._device.run_cmd(command, timeout=5)

        if output:
            # Run test
            command = "adb shell rm -r /data/system/users/*/accounts.db"

            res, output = self._device.run_cmd(command, timeout=5)

            if res != 0:
                raise Exception('adb command failed', {'command': command, 'output': output})
            else:
                self._device.reboot()