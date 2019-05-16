#!/usr/bin/env python
"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL SSG OTC
:summary: Media Instrument use case aims to run existing OTC test cases originally
          targeted to be run within cts.
:since: 11/17/2015
:author: bozhoux
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException, AcsConfigException
import subprocess
import tempfile
from xml.dom import minidom
import Queue

from Core.PathManager import Paths
import os
import time

class mocksuite(object):
    def __init__(self, suite):
        self.suite = suite

class MediaInstrument(UseCaseBase):

    """
    Generic use case that run python unittest class and extract the verdict
    from the unittest result
    """
    DEFAULT_INSTRUMENT_ROOT = "default root"

    def __init__(self, tc_name, global_config):

        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        self.media_instrument_root = os.environ.get("MEDIA_INSTRUMENT_ROOT") or MediaInstrument.DEFAULT_INSTRUMENT_ROOT
        if not self.media_instrument_root:
            raise AcsConfigException("Get media instrument root path failed! Actually got %s"%self.media_instrument_root)
        self.cts_tradefed = os.path.join(self.media_instrument_root, "android-cts", "tools", "cts-tradefed")
        self.report_folder = os.path.join(self.media_instrument_root, "android-cts", "repository", "results")
        if not os.path.exists(self.cts_tradefed):
            raise AcsConfigException("Can't get cts-tradefed binary in %s"%self.media_instrument_root)
        self._instantlogger = self._em.get_instant_logger("INSTANTLogger")

    def initialize(self):
        UseCaseBase.initialize(self)
        self.case_name = self._tc_parameters.get_param_value("TEST_CASE", default_cast_type=str)
        class_name = '.'.join(self.case_name.split(".")[:-1])
        method_name = self.case_name.split(".")[-1]
        #(class_name, method_name) = self.case_name.split("#")
        self.run_cmd = "exec bash %s run cts -c %s -m %s --disable-reboot"%(self.cts_tradefed, class_name, method_name)
        self.suite = mocksuite(self.case_name)
        self._instantlogger.initContext(self.suite, '%s.%s' % (class_name, method_name))

        self.case_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=int)
        self._logger.info("Media instrument root: %s"%self.media_instrument_root)
        self._logger.info("Case name: %s"%self.case_name)
        self._logger.info("Case timeout: %d"%self.case_timeout)
        self._logger.info("Run cmd: %s"%self.run_cmd)

        return Global.SUCCESS, "SUCCESS"

    def set_up(self):
        UseCaseBase.set_up(self)
        if os.path.exists(self.report_folder):
            map(lambda x: os.system("rm -r %s"%x), [os.path.join(self.report_folder, _) for _ in os.listdir(self.report_folder)])
        if (self._instantlogger.total_tc_executed == 0):
            self._instantlogger.begin()
            self._instantlogger.startTest(self.suite)
        return Global.SUCCESS, "SUCCESS"

    def _run_instrument_case(self):
        """run a instrument case"""
        self._logger.debug("_run_instrument_case")
        complete = False
        output = None
        p = subprocess.Popen(self.run_cmd, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        start = time.time()
        queue = Queue.Queue()
        try:
            while time.time() - start < self.case_timeout:
                line = p.stdout.readline()
                if line:
                    queue.put(line.strip())
                    print line.strip()
                    if "All done" in line:
                        time.sleep(5)
                        self._logger.info( "Media Instrument Complete!")
                        complete = True
                        break
            self._logger.debug("Queue size: %d"%queue.qsize())

            p.terminate()
            terminal_log = os.path.join(self._instantlogger.ctx.user_log_dir, "terminal.txt")
            self._logger.debug("Terminal log saved to %s"%terminal_log)
            with open(terminal_log, "w+") as f:
                for i in range(queue.qsize()):
                    f.writelines(queue.get()+"\n")
        except Exception,e:
            self._logger.error(str(e))
            raise AcsConfigException(str(e))
        output = "Media instrument complete" if complete else "Media instrument timeout %d"%self.case_timeout
        return complete, output

    def _parse_instrument_result(self):
        """Parse media instrumentation case result and return"""
        XML_STR = "testResult.xml"
        result_xml = None
        self._logger.debug("_parse_instrument_result")
        for x,y,z in os.walk(self.report_folder):
            if XML_STR in z:
                result_xml = os.path.join(x,XML_STR)

        if not result_xml:
            log = '"testResult.xml" not genorated on result folder %s'%self.report_folder
            self._instantlogger.addError(self.suite, log)
            return Global.FAILURE, log

        result = None
        log = None
        xml_root = minidom.parse(result_xml).documentElement
        for node in xml_root.getElementsByTagName("Test"):
            result = node.getAttribute("result")
            log = node.getElementsByTagName("FailedScene")[0].getAttribute("message") if result == "fail" else ""
        if result is None:
            log = 'Result unknown, Actually None'
            self._instantlogger.addError(self.suite, log)
            return Global.FAILURE, log
        elif result == "pass":
            self._instantlogger.addSuccess(self.suite)
            return Global.SUCCESS, "SUCCESS"
        else:
            self._instantlogger.addFailure(self.suite, log)
            return Global.FAILURE, log

    def run_test(self):
        '''
        Execute test case
        '''
        output = None
        self._logger.info("run_test")
        complete, output = self._run_instrument_case()
        if not complete:
            self._logger.error(output)
            self._instantlogger.addError(self.suite, output)
            return Global.FAILURE, output
        result, output = self._parse_instrument_result()
        return result, output

    def tear_down(self):
        UseCaseBase.tear_down(self)
        if os.path.exists(self.report_folder):
            map(lambda x: os.system("rm -r %s"%x), [os.path.join(self.report_folder, _) for _ in os.listdir(self.report_folder)])
        self._instantlogger.stopTest(self.suite)
        self._instantlogger.total_tc_executed += 1
        return Global.SUCCESS, "SUCCESS"

    def finalize(self):
        UseCaseBase.finalize(self)
        if (self._instantlogger.total_tc_executed == self._instantlogger.total_tc_count):
            self._instantlogger.finalize(self.suite)
        return Global.SUCCESS, "SUCCESS"
