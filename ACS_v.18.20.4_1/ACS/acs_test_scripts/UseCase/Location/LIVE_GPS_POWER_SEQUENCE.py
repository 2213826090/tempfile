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
:summary: This file implements the LIVE GPS POWER SEQUENCE SPEC MODE
:author: marouenx
:since:14/11/2013
"""

import time
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool_ex


class LiveGpsPowerSequence(UseCaseBase):

    """
    Live GPS Turn on off test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Read GPS_POWER_SEQUENCE from test case xml file
        self._turn_gps_sequence = str(self._tc_parameters.get_param_value("GPS_POWER_SEQUENCE"))

        # Get UECmdLayer
        self._location_api = self._device.get_uecmd("Location")

        self._initial_gps_power_status = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Initialize the test
        # First store the initial gps power status
        self._initial_gps_power_status = self._location_api.get_gps_power_status()
        time.sleep(self._wait_btwn_cmd)

        # Get the first element of the sequence
        first_sequence_element = str_to_bool_ex(self._turn_gps_sequence.strip().split()[0].lower())

        # Check the size of the sequence
        if len(self._turn_gps_sequence.strip().split()) < 2 :
            msg = \
                "input wrong sequence , failed ." \
                + "add more elements to the sequence in test case xml file"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if first_sequence_element is None:
            msg = \
                "input wrong sequence , failed ." \
                + "alter your sequence in test case xml file"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Check if the first element of the sequence is equal to
        # the actual state of gps
        if bool(self._initial_gps_power_status) == first_sequence_element:
            # Put gps power status unlike the first element
            # of the sequence
            self._logger.info("set gps initial status : " + str(not first_sequence_element))
            self._location_api.set_gps_power(not first_sequence_element)
            time.sleep(self._wait_btwn_cmd)


        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # begin gps turn on off sequence
        self._logger.info("gps power sequence is :" + self._turn_gps_sequence)

        seqlist = self._turn_gps_sequence.strip().split()
        for switch in seqlist:
            str_switch = switch
            switch = str_to_bool_ex(switch)
            if switch is None:
                msg = "input wrong sequence, unrecognized value : %s" % str_switch \
                + " , alter your sequence in test case xml file"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            self._location_api.set_gps_power(switch)
            time.sleep(self._wait_btwn_cmd)
            check = self._location_api.get_gps_power_status()
            time.sleep(self._wait_btwn_cmd)
            if switch != bool(check):
                msg = "set gps %s failure" % str(str_switch)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"


    #---------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Put the gps power status at its initial statuss
        if self._initial_gps_power_status is not None:
            self._location_api.set_gps_power(self._initial_gps_power_status)
            time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
