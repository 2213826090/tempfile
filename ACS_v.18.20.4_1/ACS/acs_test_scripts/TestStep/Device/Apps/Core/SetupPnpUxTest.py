import json
import re
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException

class SetupPnpUxTest(DeviceTestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        response = {}

        # Run test
        command = "adb shell am instrument -w -e apps '[]' -e account %s:%s com.intel.droidbot/.SetupRequest" % (self._pars.email, self._pars.passwd)

        _, output = self._device.run_cmd(command, timeout=300)

        # check for proper behaviour
        if 'INSTRUMENTATION_CODE: ' not in output:
            raise Exception('adb command failed', {'command': command, 'output': output})

        # read result

        for match in re.finditer(r'INSTRUMENTATION_RESULT: (\S+)=(.*)', output, re.MULTILINE):
            key = match.group(1).strip().lower()
            value = match.group(2).strip()
            try:
                response[key] = json.loads(value)
            except ValueError:
                response[key] = value
            # loop += 1

        # check for request DroidbotError
        if 'INSTRUMENTATION_CODE: -1' not in output:
            self._logger.error(str(response))
            raise DeviceException('request failed', response)