"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the driver for the Minicircuits RCDAT-x000-x0
:since: 2014-08-12
:author: emarchan

"""


from os import getenv, environ
from re import search as re_search
from urllib2 import urlopen
try:
    from acs_test_scripts.Equipment.RFAttenuator.Scenario.LogicDataDebug import LogicDataDebug
except:
    from Scenario.LogicDataDebug import LogicDataDebug


MIN_ATTN_VALUE = 0.0
ATTN_STEP = 0.25


CMD_SET_ATTN = "SETATT"
CMD_GET_ATTN = "ATT?"
CMD_GET_MODEL = "MN?"

CMD_FAIL = "0"
CMD_SUCCESS = "1"


class McRCDAT(object):
    """
    Implementation of Minicircuits RCDAT-x000-x0
    """

    def __init__(self, ip_addr):
        self._host = ip_addr
        self._password = ""
        self._attn_offset = 0.0
        self._max_attn = 0.0
        self._log = LogicDataDebug()

    def configure(self):
        no_proxy_var = getenv("no_proxy", "")
        environ['no_proxy'] = no_proxy_var + ',' + self._host
        self._max_attn = self.get_max_attn()

    def set_attenuation(self, attenuation):
        """
        Sets the attenuation. This method will round the value to the closest step supported by the attenuator.

        :type attenuation: float
        :param attenuation: Attenuation value (between MIN_ATTN_VALUE and max attenuation for the model)
        """
        # Keep the attenuation in the allowed range.
        cur_attn = float(attenuation) + self._attn_offset
        if (cur_attn < MIN_ATTN_VALUE):
            cur_attn = MIN_ATTN_VALUE
        elif (cur_attn > self._max_attn):
            cur_attn = self._max_attn

        # Round the value to the closest step
        value_to_set = round(cur_attn / ATTN_STEP) * ATTN_STEP
        # Set the value
        self._set_value(CMD_SET_ATTN, value_to_set)

    def get_attenuation(self):
        """
        Gets the current attenuation.

        :type attenuation: float
        :param attenuation: Attenuation value
        """
        # gets the value
        return self._get_value(CMD_GET_ATTN)

    def set_offset(self, offset):
        """
        Sets the offset to apply to attenuation.
        This method is used to calibrate several attenuators and set the value to have the same "0" basis.

        :type offset: float
        :param offset: Attenuation value
        """
        self._attn_offset = float(offset)

    def _set_value(self, field, value):
        """
        Generic method to set a value in the attenuator.

        :type field: string
        :param field: Field to set.

        :type value: float
        :param value: Value to set
        """
        attn_response = self._exec_cmd(field, value)
        if attn_response != CMD_SUCCESS:
            self._log_and_raise_failure("Setting %s=%s failed." % (field, value))

    def _get_value(self, field):
        """
        Generic method to get a value from the attenuator.

        :type field: string
        :param field: Field to get.

        :rtype: string
        :return: The attenuator response.
        """
        return self._exec_cmd(field)

    def _exec_cmd(self, field, value=""):
        """
        Generic method to get or set a value in the attenuator.
        Leave "value" empty to execute a get command, fill it for a set.

        :type field: string
        :param field: Field to set or get.
        :type value: float
        :param value: Value to set.

        :rtype: string
        :return: The attenuator response.
        """
        url = 'http://%s/' % (self._host)
        if self._password != "":
            url += "PWD=%s;" % self._password

        if value == "":
            url += field
        else:
            url += "%s=%s" % (field, str(value))
        self._log.debug("Requesting URI: %s" % url)
        attn_response = self._open_uri(url)
        self._log.debug("Attenuator response is %s." % attn_response)
        return attn_response

    def _open_uri(self, uri):
        """
        Opens an URI

        :type uri: string
        :param uri: URI to open.
        """
        return urlopen(uri).read()

    def get_max_attn(self):
        """
        Parses the attenuator model number and computes the maximal attenuation from it.
        :rtype: float
        :return: The maximum attenuator for the current attenuator.
        """
        attn_response = self._get_value(CMD_GET_MODEL)
        # Model should be like MN=RCDAT-6000-60
        max_attn = re_search(".+-(\d+)", attn_response)
        if max_attn is None:
            self._log_and_raise_failure("Can't get the maximal attenuation!")
        max_attn = float(max_attn.group(1))
        self._log.info("Maximum attenuation for this attenuator is %d db" % max_attn)
        return max_attn

    def _log_and_raise_failure(self, msg=""):
        """
        Logs an error and raises an exception in case of failure.

        :type msg: string
        :param msg: Error message for the logging.
        """
        self._log.error(msg)
        raise Exception(msg)

    def set_logger(self, logger):
        """
        Allow an external logger
    
        :type logger: Class
        :param logger: The external logging class
        """
        self._log = logger