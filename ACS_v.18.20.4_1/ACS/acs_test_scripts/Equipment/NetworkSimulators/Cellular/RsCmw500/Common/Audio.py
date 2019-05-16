"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: Common Audio (2G, 3G & 4G) implementation for CMW500
:since: 28/03/2014
:author: fbelvezx
"""
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject


class Audio(VisaObject):

    """
    Common Audio (2G, 3G & 4G) implementation for CMW500
    """

    def __init__(self, visa):  # pylint: disable=W0231
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)

    def set_audio_scenario(self, master_application):
        """
        Sets audio measurement scenario to External Analog Speech Analysis, with the corresponding signalling unit

        :type master_application: str
        :parameter master_application: Name of the signalling unit to be used in conjunction with the audio interface
        of the Rs CMW500
        """
        self.get_logger().info("Set audio measurement scenario to External Analog Speech Analysis")
        self._visa.send_command("ROUTe:AUDio:SCENario:EASPeech \"%s\"" % master_application)

    def calibrate_analog_audio_level(self, input_calib_level, output_calib_level):
        """
        Calibrates the input/output peak level of AF connectors

        :type input_calib_level
        :param input_calib_level
        :type output_calib_level
        :param output_calib_level
        """
        self.get_logger().info("Set analog peak level for AF1 IN connector to %s V" % input_calib_level)
        self._visa.send_command("CONFigure:AUDio:SPEech:ANALog:ILEVel %s" % input_calib_level)

        self.get_logger().info("Set analog peak level for AF1 OUT connector to %s V" % output_calib_level)
        self._visa.send_command("CONFigure:AUDio:SPEech:ANALog:OLEVel %s" % output_calib_level)
