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

:organization: INTEL MCG PSI
:summary: This file implements Networking UECmds for Android M device
:since: 2015-11-02
:author: mariussx
"""

import time

from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.Imp.Android.LLP.Networking.Networking import Networking as NetworkingLLP
from acs_test_scripts.Utilities.RegistrationUtilities import ImsRegistrationStatus

from ErrorHandling.DeviceException import DeviceException

class Networking(NetworkingLLP):

    """
    Class that handle networking operations for Android M
    """

    IMS_M_REGISTRATION_STATES = {
    "INVALID_VALUE" : -1,
    "STATE_UNKNOWN" : 0,
    "STATE_DISABLED" : 1,
    "STATE_OFFLINE" : 2,
    "STATE_REGISTERING" : 3,
    "STATE_REGISTERED" : 4,
    "STATE_DEREGISTERING" : 5,
    "STATE_SUSPENDED" : 6,
    "STATE_RESTRICTED" : 7,
    "STATE_EMERGENCY" : 16}

    IMS_REG_STATE_OUT_OF_SERVICE = 1
    IMS_REG_STATE_IN_SERVICE = 0

    def __init__(self, phone):
        """
        Constructor
        """
        NetworkingLLP.__init__(self, phone)


    def get_ims_registration_status(self):
        """
        Returns an integer description the IMS registration
        state of the device.
        :return: an integer describing the IMS registration status
            (-1 if a non-integer value has been returned by the API).
        :rtype: int
        """

        method = "getImsRegStatus"
        output = self._internal_exec_v2(self._cellular_networking_module, method, is_system=True)
        ims_registration_status_str = output["ims_registration_status"]
        ims_registration_status = -1
        if ims_registration_status_str and ims_registration_status_str.isdigit():
            ims_registration_status = int(ims_registration_status_str)

            if ims_registration_status == self.IMS_M_REGISTRATION_STATES["STATE_REGISTERED"]:
                ims_registration_status = self.IMS_REG_STATE_IN_SERVICE
            else:
                ims_registration_status = self.IMS_REG_STATE_OUT_OF_SERVICE

        return ims_registration_status


    def check_ims_registration_before_timeout(self, timeout=30):
        """
        Waits a maximum of the given timeout (in seconds) for IMS registration.

        If the IMS registration does not occur before the timeout, a
        DeviceException is raised.
        """
        start = time.time()
        ims_registered_state = ImsRegistrationStatus.in_service()
        current_status = None
        registration_time = 0
        while time.time() - start < timeout:
            time.sleep(1)
            current_status = ImsRegistrationStatus(
                self.get_ims_registration_status())
            if current_status == ims_registered_state:
                registration_time = time.time()
                break
        if current_status is None or current_status != ims_registered_state:
            return_msg = "The device could not register to IMS before " \
                "%d seconds." % timeout
            raise DeviceException(DeviceException.TIMEOUT_REACHED, return_msg)
        else:
            # Update the time needed for registration
            registration_time -= start
            # Log the computed time
            self._logger.info("IMS registration complete after %ds"
                % int(registration_time))
