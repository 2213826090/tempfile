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

:organization: OTC ANDROID
:summary: Run CTS test plan in ACS
:since: 2016-02-10
:author: hsolver
"""

import __builtin__
import os
import shutil
import tempfile
import subprocess
import sys
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import Verdict
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from Core.Report.Live.LiveReporting import LiveReporting

SysErrors = __builtin__.OSError, __builtin__.IOError,

class ITS(UseCaseBase):

    """
    Execute complete ITS on a device.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

    def initialize(self):
        """
        Process the **<Initialize>** section of the XML file and execute defined test steps.
        """
        UseCaseBase.initialize(self)
        result, output = Global.SUCCESS, ""

        if self._device._android_version == 'M':
            self._its_package_name = self._tc_parameters.get_param_value("ITS_PACKAGE_M")
        elif self._device._android_version == 'N':
            self._its_package_name = self._tc_parameters.get_param_value("ITS_PACKAGE_N")
        else:
            return Global.FAILURE, 'Unsupported Android Version'

        self._artifactory_uri = self._tc_parameters.get_param_value("ARTIFACTORY_URI")
        self._test_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=float)

        #self._scenes = ['scene0','scene1','scene2','scene3']
        self._scenes = ['scene0', 'scene1'] #TODO: Scenes 2 and 3 is not supported yet.
        self._tests = {}
        self.devnull = file(os.devnull, "w")
        return result, output

    def set_up(self):
        """
        Initialize the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        verdict = Global.SUCCESS
        msg = ""

        UseCaseBase.set_up(self)

        if self._its_package_name is None:
            return (Global.FAILURE, "You need to specify a ITS_NAME value.")

        if self._artifactory_uri is None:
            self._logger.info("Default artifactory path will be used")

        if self._test_timeout is None:
            self._test_timeout = 360000
            self._logger.info("Default test timeout will be used: %s.", self._test_timeout)

        art_mgr = EquipmentManager().get_artifact_manager("ARTIFACT_MANAGER")

        # Download ITS package
        self._ITS_path = art_mgr.get_artifact(self._its_package_name, self._artifactory_uri)

        # Check if zipfile or local cache
        fileName, fileExtension = os.path.splitext(self._ITS_path)
        if fileExtension.lower() == ".zip":
            # Unzip CTS package
            self._ITS_path = art_mgr.unzip_artifact(self._ITS_path)
            self._logger.debug("Unzip with ArtifactoryManager in: %s", self._ITS_path)

        #Set python path for the subprocesses created for the actual tests
        os.environ['PYTHONPATH'] = os.path.join(self._ITS_path, 'android-cts-verifier/CameraITS/pymodules')
        #Set the path for this python script 
        sys.path.append(os.path.join(self._ITS_path, 'android-cts-verifier/CameraITS/pymodules'))

        #Install CTSVerifier and set permissions.
        self._logger.info("Installing CtsVerifier.apk")
        cmd = ['adb', 'install', os.path.join(self._ITS_path, 'android-cts-verifier/CtsVerifier.apk')]
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)
        self._logger.info("Granting permissions for CtsVerifier")
        cmd = ['adb', 'shell', 'pm', 'grant', 'com.android.cts.verifier android.permission.CAMERA']
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)
        self._logger.info("starting CtsVerifier")
        cmd = ['adb', 'shell', 'am', 'start', 'com.android.cts.verifier/.camera.its.ItsTestActivity']
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)
        
        #turn on stay awake on the device
        cmd = ['adb', 'shell', 'svc', 'power', 'stayon', 'usb']
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)
        

        #Find out how many cams the device has
        import its.device
        its_session = its.device.ItsSession()
        self.camids = its_session.get_camera_ids()

        #build a dictionary with lists of tests.
        self.testroot = os.path.join(self._ITS_path, 'android-cts-verifier/CameraITS/tests')
        for s in self._scenes:
            self._tests[s] = []
            files = os.listdir(os.path.join(self.testroot, s))
            for f in files:
                if f[-3:] == '.py':
                    self._tests[s].append(f)

        return verdict, msg


    def run_test(self):
        """
        Execute the CTS test
        Compute result based on CTS xml result output

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.run_test(self)
        verdict = Global.SUCCESS
        tc_verdict = Verdict.BLOCKED
        failcount = 0
        msg = ""

        live_report = LiveReporting.instance()
        self._work_dir = tempfile.mkdtemp(prefix='ITS_')
        outfile = file(os.path.join(self._work_dir, 'stdout.txt') ,'w')
        errfile = file(os.path.join(self._work_dir, 'stderr.txt') ,'w')
        '''
        tc_order = 2
        for cam in self.camids:
            for s in self._scenes:
                for t in self._tests[s]:
                    live_report.send_create_tc_info(t[0:-3], 'bla', tc_order)
                    tc_order += 1
        '''

        for cam in self.camids:
            camoption = 'camera=' + str(cam)
            outdir = os.path.join(self._work_dir, str(cam))
            os.mkdir(outdir)
            for s in self._scenes:
                self._logger.debug('Running ITS test for ' + s + ' ' + camoption)
                for t in self._tests[s]:
                    live_report.send_start_tc_info('cam' + str(cam)+ ' ' +t[0:-3],iteration=True)

                    cmd = ['python', os.path.join(self.testroot, s, t), camoption]
                    result = subprocess.call(cmd, stdout=outfile, stderr=errfile, cwd=outdir)

                    debug_str = 'ITS: '
                    if result == 0:
                        debug_str += ' PASS '
                        tc_verdict = Verdict.PASS
                    elif result == 1:
                        debug_str += ' FAIL '
                        tc_verdict = Verdict.FAIL
                        failcount += 1
                    elif result == 101:
                        debug_str += ' SKIP '
                        tc_verdict = Verdict.INVALID
                    else:
                        debug_str += ' UNKNOWN '
                        tc_verdict = Verdict.INCONCLUSIVE

                    self._logger.debug(debug_str + t[0:-3])

                    live_report.send_stop_tc_info(verdict=tc_verdict,
                                                  execution_nb=1,
                                                  success_counter=1,
                                                  max_attempt=1,
                                                  acceptance_nb=1,
                                                  tc_comments="",
                                                  iteration=1)

        if failcount > 0:
            verdict = Global.FAILURE
            msg = 'ITS ' + str(failcount) + ' tests failed'
        return verdict, msg


    def tear_down(self):
        """
        End and dispose the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        #turn off stay awake on the device
        cmd = ['adb', 'shell', 'svc', 'power', 'stayon', 'false']
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)

        UseCaseBase.tear_down(self)
        shutil.rmtree(self._work_dir)
        shutil.rmtree(self._ITS_path)

        return Global.SUCCESS, "No errors"
