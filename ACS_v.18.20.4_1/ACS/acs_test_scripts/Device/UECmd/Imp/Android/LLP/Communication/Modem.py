"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the Modem UEcmd for Android devices
:since: 2015-07-28
:author: mariussx

"""
from acs_test_scripts.Device.UECmd.Imp.Android.KK.Communication.Modem import Modem as ModemKK
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
import time


class Modem(ModemKK):
    """
    :summary: Modem UEcommands for Android platform
    """

    @need('modem')
    def __init__(self, device):
        ModemKK.__init__(self, device)
        self._modem_trace_module = "acscmd.system.ModemTraceModule"

    def activate_modem_trace_amtl(self, option, timeout=60):
        """
        Activates the modem trace using AMTL application

        :param option : Configuration which must be activated
        :type option: string
        :param timeout : Timeout for activation
        :type timeout: int

        :raise DeviceException.TIMEOUT_REACHED if timeout has been reached
        :raise DeviceException.OPERATION_FAILED if activation procedure has failed
        """

        function = "activateModemTrace"
        function_args = "--es modem_trace_option \"%s\"" % (option)

        self._internal_exec_v2(self._modem_trace_module,
                               function,
                               function_args,
                               is_system=True)

        time_count = 0
        while time_count <= timeout:
            (state, current_option) = self._get_modem_trace_status_amtl()
            if (state == "activated") and (option == current_option):
                break
            elif (state == "deactivated") and (option == current_option):
                msg = "Modem trace deactivated during activation procedure."
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            time.sleep(1)
            time_count += 1
        else:
            msg = "The modem trace not activated in %s seconds..." % str(timeout)
            self._logger.error(msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

        self._logger.info("The modem trace has been successfully activated for option: " + option)


    def deactivate_modem_trace_amtl(self, option, timeout=60):
        """
        Deactivates the modem trace using AMTL application

        :param config : Configuration which must be deactivated
        :type config: string
        :param timeout : Timeout for deactivation
        :type timeout: int

        :raise DeviceException.TIMEOUT_REACHED if timeout has been reached
        :raise DeviceException.OPERATION_FAILED if de-activation procedure has failed
        """

        self._logger.info("Deactivate modem trace with configuration: " + option)
        function = "deactivateModemTrace"
        function_args = "--es modem_trace_option \"%s\"" % (option)

        self._internal_exec_v2(self._modem_trace_module,
                               function,
                               function_args,
                               is_system=True)

        time_count = 0

        while time_count <= timeout:
            (state, current_option) = self._get_modem_trace_status_amtl()
            if (state == "deactivated") and (option == current_option):
                break
            elif (state == "activated") and (option == current_option):
                msg = "Modem trace activated during de-activation procedure."
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            time.sleep(1)
            time_count += 1
        else:
            msg = "The modem trace not deactivated in %s seconds..." % str(timeout)
            self._logger.error(msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

        self._logger.info("The modem trace has been successfully deactivated for option: " + option)


    def _get_modem_trace_status_amtl(self):
        """
        Retrieve the modem trace status

        :rtype: (String, String)
        :return: The status and the configuration for modem trace
        """

        function = "getStatusModemTrace"
        output = self._internal_exec_v2(self._modem_trace_module,
                                        function,
                                        is_system=True)

        status = str(output["modem_trace_status"])
        current_option = str(output["modem_trace_config"])

        return (status, current_option)


    def get_modem_trace_configuration_amtl(self, timeout=30):
        """
        Retrieve the modem trace configurations from AMLT

        :rtype: list
        :return: The list with the modem trace configuration

        :raise DeviceException.TIMEOUT_REACHED if timeout has been reached
        """
        # Sends the command to retrieve the modem trace configuration to AMTL
        send_function = "sendGetAmtlConfiguration"
        self._internal_exec_v2(self._modem_trace_module,
                               send_function,
                               is_system=True)

        time_count = 0
        amtl_config = "null"
        get_function = "retrieveAmtlConfiguration"

        # Wait for the modem trace configuration
        while time_count <= timeout:
            output = self._internal_exec_v2(self._modem_trace_module,
                                            get_function,
                                            is_system=True)
            amtl_config = str(output["amtl_config"])
            if amtl_config != "null":
                break
            time.sleep(1)
            time_count += 1
        else:
            msg = "Cannot retrieve the modem trace configuration in %s seconds..." % str(timeout)
            self._logger.error(msg)
            raise DeviceException(DeviceException.TIMEOUT_REACHED, msg)

        self._logger.info("The modem trace configuration has been successfully retrieve.")
        amtl_config_list = amtl_config.split(";")
        # Remove the last in the list as it is not valid
        amtl_config_list = amtl_config_list[:-1]

        return amtl_config_list

    def close_amtl_app(self):
        """
        Close the AMTL application

        :rtype: None
        :return: None
        """
        function = "closeAmtlApplication"
        self._internal_exec_v2(self._modem_trace_module,
                               function,
                               is_system=True)
