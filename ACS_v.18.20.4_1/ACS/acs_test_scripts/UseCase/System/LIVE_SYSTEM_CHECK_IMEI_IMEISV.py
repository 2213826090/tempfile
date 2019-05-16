"""
:copyright: (c)Copyright 2012, Intel Corporation All Rights Reserved.
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
:summary: This file is check the IMEI and IMEISV info of the device
:since: 14/01/2013
:author: Hbianx
"""

import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveSystemCheckImeiImeisv(UseCaseBase):

    """
    Class check the IMEI and IMEISV
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        # Get TC Parameters
        self._imei = self._tc_parameters.get_param_value("IMEI")
        self._imeisv = self._tc_parameters.get_param_value("IMEISV")
        # Get UECmdLayer
        self._modem_api = self._device.get_uecmd("Modem")

        self._test_result = Global.SUCCESS

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # The Imei is a mandatory value to check
        if self._imei.isdigit() and (len(self._imei) == 15):
            self._logger.info("The expected imei value is %s " % self._imei)
        else:
            self._logger.info("self._imei: %s : %d" % (self._imei, len(self._imei)))

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "imei value should be 15 decimal digit")

        # IMEISV value is optional, if the value is empty, we won't check it
        if self._imeisv in [None, ""]:
            self._logger.info("The imeisv number will not be checked ")
            self._imeisv = None
        else:
            if  self._imeisv.isdigit() and \
                    (len(self._imeisv) == 16 or len(self._imeisv) == 2):
                self._logger.info("The expected imeisv value is %s " %
                                  self._imeisv)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                       "imeisv value should be 2 or 16 decimal digit")

        return self._test_result, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call UseCase base Run function
        UseCaseBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        self._device_imei = self._modem_api.get_imei()

        # Check at first imei value
        if self._device_imei == self._imei:
            self._logger.info("The device imei %s is the same as the expected "
                              "imei %s" % (self._device_imei, self._imei))
            # If imei value check success, we will check imeisv
            if self._imeisv is not None:
                self._device_imeisv = self._modem_api.get_imeisv()

                if self._device_imeisv == self._imeisv:
                    self._logger.info("The device imeisv %s is the same as the "
                                      "expected imeisv %s \n" % (self._device_imeisv,
                                                                 self._imeisv))
                else:
                    self._error.Msg = "The device imeisv %s is "\
                        "different the expected imeisv %s" % \
                        (self._device_imeisv, self._imeisv)
                    self._test_result = Global.FAILURE
        else:
            self._error.Msg = "The device imei %s is different the expected "\
                "imei %s" % (self._device_imei, self._imei)
            self._test_result = Global.FAILURE

        return self._test_result, self._error.Msg
