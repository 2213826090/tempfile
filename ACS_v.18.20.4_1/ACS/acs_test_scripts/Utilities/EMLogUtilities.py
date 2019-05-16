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
:summary: This file implements tools for EM logging
:since: 28/10/2014
:author: vgombert
"""

from UtilitiesFWK.Utilities import Singleton
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT_STATS
import traceback


class EMLogUtilities(object):
    __metaclass__ = Singleton
    # mandatory key
    EVENT = "event"
    # specific event key
    SUBEVENT = "subevent"
    STATUS = "status"

    # optional
    BUILD = "build"
    DEVICE_TYPE = "device_type"
    CAMPAIGN = "campaign"
    TC_NAME = "tc_name"
    CALLER = "function"

    def __init__(self):
        """
        this class is a singleton which mean that it wont be instantiate twice during
        a testcase execution.
        This allow em module to have access to data gathered by this class from the em usecase base for example.
        """
        self.__constant_values = {}

    def add_constant_values(self, constant_info):
        """
        Add constant values to the logger.
        constant values are sent each time you log something.
        Beware not to add constant values specific to a module as this instance
        may be use by other classes than the given modules.

        :type constant_info: list of tuple or dict
        :param constant_info: list of information to return
        """
        if type(constant_info) is dict:
            list_to_parse = constant_info.items()
        else:
            list_to_parse = constant_info

        for key, value in list_to_parse:
            if value not in ["", None]:
                self.__constant_values[key] = str(value)

    def dynamic_log(self, dynamic_info, log_constant=True, log_caller=True):
        """"
        create a dynamic log message with all info from the input list plus
        the info from constant log and the caller name

        :type dynamic_info: list of tuple or dict
        :param dynamic_info: list of information to return

        :type log_constant: boolean
        :param log_constant: if set to True, constant value stored in this class will be added to the log message

        :type log_caller: boolean
        :param log_caller: if set to True, the function name that call dynamic_log will be added to the log message
        """
        msg = ""
        # add dynamic info first
        if type(dynamic_info) is dict:
            list_to_parse = dynamic_info.items()
        else:
            list_to_parse = dynamic_info

        for key, value in list_to_parse:
            msg += "%s=%s; " % (key, str(value))

        # then add constant info
        if log_constant:
            for key, value in self.__constant_values.items():
                if value not in ["", None]:
                    msg += "%s=%s; " % (key, str(value))

        if msg != "":
            # add caller name
            if log_caller:
                msg += "%s=%s" % (self.CALLER, str(traceback.extract_stack(None, 2)[0][2]))
            # remove any last separator
            msg = msg.strip().rstrip(";")
            LOGGER_TEST_SCRIPT_STATS.info(msg)

    def log(self, event, subevent=None, status=None, log_constant=True, log_caller=True):
        """"
        classic logger with mandatory input.
        constant info will be added as well as caller name

        :type event: str
        :param event: mandatory information

        :type subevent: str
        :param subevent: if set to None or "" , it will be ignore

        :type status: str
        :param status: the status of the event, can be PASS or FAIL, if set to None or "" , it will be ignore

        :type log_constant: boolean
        :param log_constant: if set to True, constant value stored in this class will be added to the log message

        :type log_caller: boolean
        :param log_caller: if set to True, the function name that call dynamic_log will be added to the log message
        """
        msg = ""
        msg += "%s=%s; " % (self.EVENT, str(event))
        if subevent not in ["", None]:
            msg += "%s=%s; " % (self.SUBEVENT, str(subevent))
        if status not in ["", None]:
            msg += "%s=%s; " % (self.STATUS, str(status))
        # then add constant info
        if log_constant:
            for key, value in self.__constant_values.items():
                if value not in ["", None]:
                    msg += "%s=%s; " % (key, str(value))

        if msg != "":
            # add caller name
            if log_caller:
                msg += "%s=%s" % (self.CALLER, str(traceback.extract_stack(None, 2)[0][2]))
            # remove any last separator
            msg = msg.strip().rstrip(";")
            LOGGER_TEST_SCRIPT_STATS.info(msg)
