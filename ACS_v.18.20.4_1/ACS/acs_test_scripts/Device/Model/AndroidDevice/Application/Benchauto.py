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

:organization: INTEL MCG PSI
:summary: This script implements the Benchauto benchmark runner for performance measurement
:since: 30/06/2014
:author: stasson

For more information regarding benchauto : see benchmator project page @
https://sharepoint.ger.ith.intel.com/sites/si-perf-opt/SitePages/Benchmator.aspx

"""
import time
import json
import re

from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IApplication
from ErrorHandling.DeviceException import DeviceException


class Benchauto(IApplication):
    """
    Implements benchauto.jar uiautomation test runner
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IApplication.__init__(self, device)
        self._benchauto_status = {}

    def install(self, appuri, additionnals=None, arguments=None, destination=None):
        """
        on top of apk install, must make sure language is english and push the 8.jar files
        """
        IApplication.install(self, appuri, additionnals, arguments, destination)

        # check if language switch to en if it is not :
        language = self.adb_shell("getprop persist.sys.language", 5)
        if "en" not in language:
            self._phonesystem.change_language_country_informations("en", "US")

    def run(self, tc_parameters):
        """
        build the uiautomator command line and populate json status in the _result dictionary.
        """
        # read test case parametters
        self.is_lower_better = tc_parameters.get_param_value("LOWER_IS_BETTER", "False", "str_to_bool")
        timeout = tc_parameters.get_param_value("TIMEOUT", 60, int)
        measure_nb = tc_parameters.get_param_value("MEASURE_NB", 1, int)
        benchauto_test = tc_parameters.get_param_value("BENCHAUTO_TEST")
        benchauto_args = tc_parameters.get_param_value("BENCHAUTO_ARGS", "")

        # build the command line
        cmd = "uiautomator runtest /sdcard/benchauto/benchauto.jar /sdcard/benchauto/benchlibs.jar -s "
        cmd += "-c %s " % benchauto_test
        cmd += "-e niter %s " % str(measure_nb)
        cmd += "-e timeout %s " % str(timeout)

        cmd += benchauto_args

        # run the test
        self._logger.info("cmd= %s" % cmd)
        output = self.adb_shell(cmd, measure_nb * (timeout + 200))

        # parse output for status and populate results
        jsonMatch = re.search("INSTRUMENTATION_STATUS: json=(.*)", output)
        if jsonMatch:
            jsonResult = jsonMatch.group(1)
            status = json.loads(jsonResult)
            self._logger.debug("status: " + str(json.dumps(status, indent=4)))
            scorenames = tc_parameters.get_param_value("SCORE_REPORT", "")
            self._populate_results(status, scorenames)
        else:
            self._logger.info("output= %s" % output)
            raise DeviceException(DeviceException.OPERATION_FAILED, "No Results found")

    def _populate_results(self, status, scorenames):
        """
        populate _results from status entries:
        results are renamed as per scorenames parametters if any:
            result-name=entry-name, ...
        """
        for entry in status['entries']:

            key = entry.strip().replace(" ", "_")

            # check if need to rename
            if key in scorenames:
                for match in scorenames.split(","):
                    if key == match.split('=')[1].strip():
                        key = match.split('=')[0].strip()

            self._results[key] = status[entry]['Runs']
