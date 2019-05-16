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
:summary: This script implements the 0xBench benchmark for performance measurement
:since: 19/06/2013
:author: pbluniex
"""
import re
import os
from lxml import etree
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class OxBench(IAndroidPackage):

    """
    Implementation of caffeinemark as Application
    """

    BENCHMARKS = {
        "LINPACK_1T": ("math", "Linpack(1thread(s))"),
        "LINPACK_2T": ("math", "Linpack(2thread(s))"),
        "LINPACK_4T": ("math", "Linpack(4thread(s))"),
        "SCIMARK2_COMPOSITE": ("math", "Scimark2:COMPOSITE"),
        "SCIMARK2_FTT": ("math", "Scimark2:FTT"),
        "SCIMARK2_SOR": ("math", "Scimark2:SOR"),
        "SCIMARK2_MONTECARLO": ("math", "Scimark2:MONTECARLO"),
        "SCIMARK2_SPARSEMATMULT": ("math", "Scimark2:SPARSEMATMULT"),
        "SCIMARK2_LU": ("math", "Scimark2:LU"),
        "DRAWCANVAS": ("2D", "DrawCanvas"),
        "DRAWCIRCLE2": ("2D", "DrawCanvas"),
        "DRAWRECT": ("2D", "DrawRect"),
        "DRAWARC": ("2D", "DrawArc"),
        "DRAWIMAGE": ("2D", "DrawImage"),
        "DRAWTEXT": ("2D", "DrawText"),
        "OPENGLCUBE": ("3D", "OpenGLCube"),
        "OPENGLBLENDING": ("3D", "OpenGLBlending"),
        "OPENGLFOG": ("3D", "OpenGLFog"),
        "FLYINGTEAPOT": ("3D", "FlyingTeapot"),
        "GARBAGECOLLECTION": ("VM", "GarbageCollection")
    }

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "0XBENCH")
        self.is_lower_better = False

        self.__pattern = "Benchmark: XML:"
        self._results = {"score": []}

    def _get_xml_result(self):
        """
        Get the result dictionary of the run

        :rtype: dictionary
        :return: The dictionary of the result
        """
        res = self._get_device_logger().get_message_triggered_status(self.__pattern)
        if res is None or len(res) == 0:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't get result")

        matchobj = re.search(r"%s (?P<result>.*)$" % self.__pattern, res[0])

        if matchobj is None:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't get result for '%s'" % self._arguments)

        str_result = matchobj.group("result")
        xml_root = etree.XML(str_result)
        return xml_root

    def _fetch_result(self):
        """
        Return the score of caffeinemark run
        """
        result = self._get_xml_result()
        xpath = "/result/scenario[@benchmark = '%s']" % \
                OxBench.BENCHMARKS[self._arguments][1]

        scenario = result.xpath(xpath)
        if scenario:
            self._results["score"].append(float(scenario[0].text))
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "There is no scenario called '%s' in XML stream" %
                                  OxBench.BENCHMARKS[self._arguments][1])

    def wait(self, timeout):
        """
        Wait until the end of the run

        :type timeout: integer
        :param timeout: Time in second beyond the application should end
        """
        if not self._get_device_logger().\
                is_message_received(self.__pattern, timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Execution test timeout has been reached")

    def start(self):
        """
        Start the application
        """
        run_apk_cmd = "am start "
        run_apk_cmd += "-a android.intent.action.LAUNCHER "
        run_apk_cmd += "-n " + self._get_application_name()
        run_apk_cmd += "/" + self._get_launcher()

        if self._arguments not in OxBench.BENCHMARKS:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is no test named '%s'" % self._arguments)

        category = OxBench.BENCHMARKS[self._arguments][0]
        run_apk_cmd += " --ez %s true --ez autorun true" % category

        self._get_device_logger().add_trigger_message(self.__pattern)

        self.adb_shell(run_apk_cmd, 3)

    def stop(self):
        """
        Stop the application
        """
        IAndroidPackage.stop(self)
        self._get_device_logger().remove_trigger_message(self.__pattern)

    def post_install(self):
        """
        Pre-installation actions
        """
        self.disable_jni()
