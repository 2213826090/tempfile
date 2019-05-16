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
:summary: This file implements a Test Step to parse the camera config file and return the width and height expected
:since: 28/01/2016
:author: droseyx
"""

from Core.TestStep.DeviceTestStepBase import TestStepBase
import xml.etree.ElementTree as ET

class CheckPreviewSize(TestStepBase):
    """
    Parse the camera config file and return in the context the width and height of the preview size
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        :type tc_conf: :py:class:`~src.Core.TCParameters`
        :param tc_conf: test case parameters
        :type  global_conf: object
        :param global_conf: global configuration data
        :type  ts_conf: object
        :param ts_conf: test steps parameters
        :type  factory: object
        :param factory: it's responsible for creating ACS objects needed by the test step
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._tree = None
        self._root = None
        self._size = ""

    def run(self, context):
        """
        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)
        self._logger.info("Retrieving %s from %s config file" % (self._pars.attribute, self._pars.host_config_file))
        try:
            self._tree = ET.parse(self._pars.host_config_file)
            self._logger.info("Config file %s parsed" % self._pars.host_config_file)
        except:
            self._logger.warning("Unable to parse XML config file %s" % self._pars.host_config_file)
        if self._tree is not None:
            try:
                self._root = self._tree.getroot()
                self._size = self._root.find('.//string[@name="%s"]' % self._pars.attribute).text
                self._logger.info("The expected %s is: %s"  % (self._pars.attribute, self._size))
            except:
                self._logger.warning("Unable to get the %s from the XML config file"  % self._pars.attribute)
            if self._size and 'x' in self._size:
                context.set_info(self._pars.preview_width, self._size.split('x')[0])
                context.set_info(self._pars.preview_heigth, self._size.split('x')[1])