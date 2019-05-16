"""
@summary:
Set /system partition to read/write mode on SUT .

@since 25 May 2015
@author: Aamir Khowaja
@organization: INTEL TMT-TTD-AN
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Utilities import AcsConstants, Global

class MakeSystemPartitionRW(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        self._logger.info("MakeSystemPartitionRW: Run")

        self._device.set_filesystem_rw()
