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

:organization: INTEL QCTV
:summary: Configure micro circuit USB-4SPDT-A18 RF matrix
:since: 19/01/2015
:author: mbrisbax
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LabRfMatrix(UseCaseBase):

    """
    Lab GRPS Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC parameters
        # Get DUT port on RF matrix
        if "RFMatrixPort" in self._dut_config:
            self._dut_port = int(self._dut_config.get("RFMatrixPort"))
        else:
            self._dut_port = 1

        # Init RF matrix
        self._matrix = self._em.get_rf_matrix("RF_MATRIX")

# ------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Init RF matrix
        self._matrix.init()

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)
        # Switch RF matrix port to desired port
        self._matrix.switch_to_rf_port(self._dut_port)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        # Release RF matrix
        self._matrix.release()

        return Global.SUCCESS, "No errors"
