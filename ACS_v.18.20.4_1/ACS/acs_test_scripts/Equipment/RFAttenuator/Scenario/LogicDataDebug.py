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

:organization: INTEL CCG
:summary: This file implements the debug class for the SaleaeDataParser
:since: 2015-01-27
:author: emarchan
"""
import os, inspect, csv
import logging, time, datetime

class LogicDataDebug():
    """
    This class is used to switch from ACS logging system to a simple print one.
    """
    DVG_LEVELS = {"ERROR":0, "WARNING":1, "INFO":2, "DEBUG":3}
    def __init__(self, acs_logger=None, create_log_file=""):
        """
        Constructor
        :type acs_logger: Logger (ACS class)
        :param acs_logger: is set to None, the output will be the console, if an ACS logger is provided, the output will
        be directed to it.

        """
        self._acs_logger = acs_logger
        self._dbg_lvl = 3

        logging.basicConfig(filename=create_log_file, format='%(asctime)s -- %(levelname)s -- %(message)s', level=logging.DEBUG)

    def error(self, msg=""):
        """
        Error logger
        """
        self._log ("ERROR", msg)

    def warning(self, msg=""):
        """
        Warning logger
        """
        self._log ("WARNING", msg)

    def info(self, msg=""):
        """
        Info logger
        """
        self._log ("INFO", msg)


    def debug(self, msg=""):
        """
        Debug logger
        """
        self._log ("DEBUG", msg)

    def _log(self, level, msg):
        """
        Generic internal logger called by other loggers.
        """
        if self._acs_logger is None:
            self._internal_logger(level, msg)
        else:
            getattr(self._acs_logger, level)(msg)


    def _internal_logger(self, level, msg):
        """
        Generic internal logger called by other loggers.
        """
        if self.DVG_LEVELS[level] <= self._dbg_lvl:
            print (str(datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d-  %H:%M:%S ')) + level + " - {0}" .format(msg))
        if self.DVG_LEVELS[level] == 0:
            logging.error(msg)
        elif self.DVG_LEVELS[level] == 1:
            logging.warning(msg)
        elif self.DVG_LEVELS[level] == 2:
            logging.info(msg)
        else :
            logging.debug(msg)

    def set_logging_level(self, level):
        """
        Sets the maximum level of traces to print.
        :param level: Maximum level of traces to print
        :type level: key of DVG_LEVELS
        """
        self._dbg_lvl = self.DVG_LEVELS[level]

    def log_tag(self):
        return str(os.path.basename(__file__)) + "@" \
            + str(inspect.currentframe().f_back.f_lineno)\
            + "|"

