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

@organization: PSTV
@summary:  This script implements Fishtank
@since: 30/07/2014
@author: sammathx
"""
import os
import re
import time
import subprocess
import signal
from acs_test_scripts.Device.Model.AndroidDevice.Application.IBrowsing import IBrowsing
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication
from ErrorHandling.DeviceException import DeviceException

process_id=None
TIMEOUT=10

class Fishtanknew(IBrowsing):
    """
    Implementation
    """
    def __init__(self, device):
        """
        Initializes this instance.
        @type device: Device
        @param device: The DUT
        """
        IBrowsing.__init__(self, device)
        self.__device = device
        self._results = {"score": []}
        self._scores=[]
        self._sum=0
        self.is_lower_better = False

    def generateLogcat(self):
        global process_id
        time.sleep(0.5)
        self.__device.run_cmd('adb shell logcat -c ', TIMEOUT)
        time.sleep(1)
        a=subprocess.Popen("adb shell \"su -c logcat -v time > /storage/sdcard0/logcatfishtank.log \"", shell=True)
        time.sleep(3)
        process_id=a.pid
        self._logger.info("Process id for the logcat is :%s "%process_id)

    def __get_screenshot(self):
        """
        Capture the screenshot at end of the test
        """
        report_dir = self.__device.get_report_tree()
        self.__browser_mark_report_path = os.path.join(report_dir.get_report_path(), "fishtanknew_result")
        filename = self.__browser_mark_report_path+ "/" + "Fishtank%s.png" % time.ctime().replace(':', '-')
        self.__screenshot = self.__device.screenshot(filename=filename)

    def _fetch_result(self):
        """
        Return the score of Browsermark run
        """
        self.__get_screenshot()

    def wait(self, timeout):
        """
        Wait until the end of the run

        @type timeout: integer
        @param timeout: Time in second beyond the application should end
        """
        self._logger.info("Wait until the test complete")

        #Killing the logcat current process
        os.kill(process_id, signal.SIGKILL)
        self._logger.info("Process with id %s has been killed!!"%process_id)
        time.sleep(5)
        overall_score=self.__device.run_cmd('adb shell cat /storage/sdcard0/logcatfishtank.log | grep "score" ', TIMEOUT)
        if overall_score[1] is '' :
            self._logger.info("Unable to fetch result from logcat,Taking screenshot")
        else :
            a=overall_score[1]
            b=re.search("Result : (\d+)?",a)
            score=b.group(1)
            #Checking for the score available from logcat
            if "score" in overall_score[1] :
                       self._logger.info("Overall Score obtained is %s"%score)
                       self._scores.append(b.group(1))
                       self._results["score"].append(int(score))
            else :
                       self._logger.info("Unable to fetch score")
                       raise DeviceException(DeviceException.TIMEOUT_REACHED,"Timeout while browsing")

        #Removing the logcat message
        self._logger.info('adb shell \"su -c rm -rf /storage/sdcard0/logcatfishtank.log \"', TIMEOUT)
        self._logger.info("Logs has been successfully removed")

    def drive(self):
        """
        Drive the application
        """
        self._logger.info("Drive : Test start executing")
        time.sleep(110)

    def start(self):
        """
        Start application
        """
        self._logger.info("Wait 10 secs before running benchmark")
        time.sleep(TIMEOUT)
        IBrowsing.start(self)
        time.sleep(3)
        self.generateLogcat()

    def post_install(self):
        """
        Post-installation actions
        """
        IBrowsing.post_install(self)
        self.clear_cache()
    def get_score(self, stat_type="MEDIAN"):

        IApplication.get_score(self, stat_type)
        raise DeviceException(DeviceException.OPERATION_SET_ERROR, "Empty operation set is defined")
