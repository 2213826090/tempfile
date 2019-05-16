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
:summary: This script implements GLBenchmark 2.7 application
:since: 24/05/2013
:author: pbluniex
"""
import re
import time
import os
from lxml import etree
from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class GlBenchmark(IAndroidPackage):

    """
    GLBenchmark 2.1, 2.5 and 2.7 implementation
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IAndroidPackage.__init__(self, device)
        self._path_ref = os.path.join("BENCHMARKS", "GLBENCHMARK")
        self.is_lower_better = False

        self._run_no = 0
        self._results = {"score": []}

        self.__data_path = None
        self.__drive_version = None
        self.__version = None
        self.__selected_tests_xml = None
        self.__main_menu = None
        self.__result_file = None

        self.__tests = {}
        self.__report_path = None
        self.__glb_start_command = ["move_home", "dpad_center", "dpad_right", "enter"]
        self.__regex_test_result = "FinishCurrentTest.*\(.*frames\s(?P<result>[\d]*).*FPS"
        self.__test_name = None
        self.__test_dic = {"glbenchmark21": {"EgyptStandard": "Egypt_Standard",
                                             "EgyptOffScreen": "Egypt_OffScreen",
                                             "ProStandard": "Pro_Standard",
                                             "ProOffScreen": "Pro_OffScreen"},
                           "glbenchmark25": {"EgyptHD": "EgyptHD_C24Z16_ETC1",
                                             "EgyptHDOffScreen": "EgyptHD_C24Z16_ETC1_OffScreen",
                                             "EgyptStandard": "EgyptHD_C16Z16_ETC1",
                                             "EgyptOffScreen": "EgyptHD_C16Z16_ETC1_OffScreen"},
                           "glbenchmark27": {"TRexHD": "TRexHD_C24Z16_ETC1_OnScreen",
                                             "TRexHDOffScreen": "TRexHD_C24Z16_ETC1_OffScreen",
                                             "EgyptHD": "EgyptHD_C24Z16_OnScreen",
                                             "EgyptHDOffScreen": "EgyptHD_C24Z16_OffScreen"}}

    def __configure25(self):
        """
        Configure application in version 2.5
        """
        self.__tests = ["FillRate_C24Z16_888",
                        "TriangleThroughput_C24Z16_888",
                        "TriangleThroughput_C24Z16_VertexLit_888",
                        "TriangleThroughput_C24Z16_FragmentLit_888",
                        "EgyptHD_C24Z16_ETC1",
                        "EgyptHD_C24Z16_PVRTC4",
                        "EgyptHD_C24Z16_DXT1",
                        "EgyptHD_C24Z16_ETC1->565",
                        "EgyptHD_C24Z24MS4_ETC1",
                        "EgyptHD_C24Z16_Fuxed_Timestep_ETC1",
                        "EgyptHD_C16Z16_ETC1"]

        self.__result_file = "last_results_2.5.1"

    def __configure27(self):
        """
        Configure application in version 2.7
        """
        self.__tests = ["FillRate_C24Z16_888_OnScreen",
                        "FillRate_C24Z16_888_OffScreen"
                        "TriangleThroughput_C24Z16_888_OnScreen",
                        "TriangleThroughput_C24Z16_888_OffScreen",
                        "TriangleThroughput_C24Z16_VertexLit_888_OnScreen",
                        "TriangleThroughput_C24Z16_VertexLit_888_OffScreen",
                        "TriangleThroughput_C24Z16_FragmentLit_888_OnScreen",
                        "TriangleThroughput_C24Z16_FragmentLit_888_OffScreen",
                        "TRexHD_C24Z16_OnScreen",
                        "TRexHD_C24Z16_OffScreen",
                        "TRexHD_C24Z16_ETC1_OnScreen",
                        "TRexHD_C24Z16_ETC1_OffScreen",
                        "TRexHD_C24Z16_DXT1_OnScreen",
                        "TRexHD_C24Z16_DXT1_OffScreen",
                        "TRexHD_C24Z16_PVRTC4_OnScreen",
                        "TRexHD_C24Z16_PVRTC4_OffScreen",
                        "TRexHD_C24Z16_ETC1to565_OnScreen",
                        "TRexHD_C24Z16_ETC1to565_OffScreen",
                        "TRexHD_C24Z24MS4_OnScreen",
                        "TRexHD_C24Z24MS4_OffScreen",
                        "TRexHD_C24Z16_Fixed_TimeStep_OnScreen",
                        "TRexHD_C24Z16_Fixed_TimeStep_OffScreen",
                        "EgyptHD_C24Z16_OnScreen",
                        "EgyptHD_C24Z16_OffScreen",
                        "EgyptHD_C24Z24MS4_OnScreen",
                        "EgyptHD_C24Z24MS4_OffScreen",
                        "EgyptHD_C24Z16_FixedTime_OnScreen",
                        "EgyptHD_C24Z16_FixedTime_OffScreen"]

    def __configure21(self):
        """
        Configure application in version 2.7
        """
        self.__tests = ["Egypt_Standard",
                        "Egypt_High",
                        "Egypt_Fixed_TimeStep",
                        "Egypt_Offscreen",
                        "Pro_Standard",
                        "Pro_High",
                        "Pro_Fixed_TimeStep",
                        "Pro_Offscreen",
                        "SwapBuffer",
                        "Trigonometric_Vertex_Weighted",
                        "Trigonometric_Fragment_Weighted",
                        "Trigonometric_Balanced_Weighted",
                        "Exponential_Vertex_Weighted",
                        "Exponential_Fragment_Weighted",
                        "Exponential_Balanced_Weighted",
                        "Common_Vertex_Weighted",
                        "Common_Fragment_Weighted",
                        "Common_Balanced_Weighted",
                        "Geometric_Vertex_Weighted",
                        "Geometric_Fragment_Weighted",
                        "Geometric_Balanced_Weighted",
                        "Loop_Vertex_Weighted",
                        "Loop_Fragment_Weighted",
                        "Loop_Balanced_Weighted",
                        "Branch_Vertex_Weighted",
                        "Branch_Fragment_Weighted",
                        "Branch_Balanced_Weighted",
                        "Uniform_Array",
                        "Fill",
                        "Triangle",
                        "Triangle_Texture",
                        "Triangle_Texture_Vertex_Lit",
                        "Triangle_Texture_Fragment_Lit"]

    def _fetch_result(self):
        """
        Get score for GlBenchmark
        """

        res_line = self._get_device_logger().\
            get_message_triggered_status("regex:" + self.__regex_test_result)

        self._get_device_logger().remove_trigger_message("regex:" + self.__regex_test_result)
        if not res_line:
            return
        self._logger.info(res_line)
        match = re.search(self.__regex_test_result, res_line[0])
        self._logger.info(match.group("result"))
        if match:
            self._results["score"].append(float(match.group("result")))
        else:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can't find score for %s" % self._arguments)

    def post_install(self):
        """
        Post install actions
        """
        self.__version = self._get_application_name().split(".")[2]

        if self.__version == "glbenchmark21":
            #remove existing configuration file
            command = "rm -rf /sdcard/GLBenchmarkData"
            self.adb_shell(command, 3)
            self.__configure21()
        elif self.__version == "glbenchmark27":
            #remove existing configuration file
            command = "rm -rf /sdcard/GLBenchmarkData"
            self.adb_shell(command, 3)
            self.__configure27()
        elif self.__version == "glbenchmark25":
            self.__configure25()
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s version is not supported in ACS" % self.__version)

    def wait(self, timeout):
        """
        Wait until the end of the benchmark

        :type timeout: integer
        :param timeout: The time in second the benchmark should end
        """
        self._run_no += 1
        logger = self._get_device_logger()
        if not logger.is_message_received("regex:" + self.__regex_test_result,
                                          timeout):
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Timeout reached while running %s" %
                                  self._arguments)

        # Wait before GLBenchmark finish to write result file
        time.sleep(1)

    def start(self):
        """
        Start application
        """
        test_dic = self.__test_dic[self.__version]
        if self._arguments not in test_dic.keys():
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Test %s not in dictionnary for %s" %
                                     (self._arguments, self.__version))

        self.__test_name = test_dic[self._arguments]
        if self.__version == "glbenchmark21":
            self.__drive_version = self.__drive21
        elif self.__version == "glbenchmark27":
            self.__drive_version = self.__drive27
        elif self.__version == "glbenchmark25":
            self.__drive_version = self.__drive25
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "%s version is not supported in ACS" % self.__version)

        if "pro" in self.__test_name.lower():
            self.__regex_test_result = "Pro:\s*\d*\s*frames\s*\((?P<result>[\d*.]*)\s*FPS\)"

        IAndroidPackage.start(self)

    def drive(self):
        """
        Drive the application
        """
        time.sleep(10)
        command = ["MOVE_HOME", "ENTER"]
        self._keyevent.scenario(command)

        command = ["DPAD_UP", "MOVE_HOME"]
        self._keyevent.scenario(command)

        self.__drive_version()

        self._get_device_logger().add_trigger_message("regex:" + self.__regex_test_result)

    def __drive21(self):
        """
        Drive the application 2.5 version
        """
        regex = self.__test_name.split("_")[0]
        self.__regex_test_result = "%s:\s*\d*\s*frames\s*\((?P<result>[\d*.]*)\s*FPS\)" % regex
        test_found = False
        for test in self.__tests:

            if test.lower() == self.__test_name.lower():
                test_found = True
                if self._run_no is 0:
                    command = ["ENTER"]
                    self._keyevent.scenario(command)
                self._logger.info("test %s selected" % test)
            command = ["DPAD_DOWN"]
            self._keyevent.scenario(command)

        if not test_found:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is no test named %s" % self._arguments)

        command = ["ENTER"]
        self._keyevent.scenario(command)

    def __drive27(self):
        """
        Drive the application 2.5 version
        """
        test_found = False
        for test in self.__tests:
            command = ["DPAD_DOWN"]
            self._keyevent.scenario(command)
            if test.lower() == self.__test_name.lower():
                test_found = True
                if self._run_no is 0:
                    command = ["ENTER"]
                    self._keyevent.scenario(command)
                self._logger.info("test %s selected" % test)

        if not test_found:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is no test named %s" % self._arguments)

        command = ["DPAD_RIGHT", "ENTER"]
        self._keyevent.scenario(command)

    def __drive25(self):
        """
        Drive the application 2.5 version
        """
        test_found = False
        time.sleep(10)
        for test in self.__tests:
            if test.lower() in self.__test_name.lower():
                test_found = True
                if self._run_no is 0:
                    command = ["ENTER"]
                    self._keyevent.scenario(command)
                self._logger.info("test %s selected" % test)
            command = ["DPAD_DOWN"]
            self._keyevent.scenario(command)
        if not test_found:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "There is no test named %s" % self._arguments)
        if self._run_no is 0:
            if "offscreen" in self._arguments.lower():
                command = ["DPAD_LEFT", "ENTER", "DPAD_RIGHT"]
                self._keyevent.scenario(command)
            command = ["DPAD_RIGHT", "DPAD_RIGHT", "ENTER"]
            self._keyevent.scenario(command)
        else:
            if "offscreen" in self._arguments.lower():
                 command = ["DPAD_RIGHT", "DPAD_RIGHT", "ENTER"]
                 self._keyevent.scenario(command)
