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
:summary: This file implements the Modem firmware Flash Use Case
:since: 11/05/2015
:author: amurarux
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.System.LAB_MODEM_FLASHING_BASE import LabModemFlashingBase


class LabModemFwFlashing(LabModemFlashingBase):

    """
    Lab Modem Flash with DUT in I{MOS} (Main OS) Use Case Class.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        LabModemFlashingBase.__init__(self, tc_name, global_config)

        # Instantiate generic UECmd for modem
        self._modem_api = self._device.get_uecmd("Modem")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Run UC base run_test
        LabModemFlashingBase.run_test(self)

        # Check that device camps on the network
        self._logger.debug("Check network registration status is registered on DUT")
        self._modem_api.check_cdk_state_bfor_timeout("registered",
                                                    300)

        # Return the verdict of the flash
        return Global.SUCCESS, "Modem FW flashing OK and registration reached"