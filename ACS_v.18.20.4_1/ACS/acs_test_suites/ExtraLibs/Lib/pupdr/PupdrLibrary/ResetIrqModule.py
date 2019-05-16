#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: Pupdr Library - ResetIrqModule
@since: 11/17/2014
@author: travenex
"""

import re
import itertools
import OutputModule
import HostModule
import LoggerModule
import ConfigurationModule

class ResetIrqModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __host = None
    __output = None
    __configuration = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__output = OutputModule.OutputModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()

    def name(self):
        if self.__configuration.flash.MERRIFIELD or self.__configuration.flash.MOOREFIELD:
            return "RESETSRC1"
        else:
            return "RESETIRQ1"

    def warning(self):
        if self.__configuration.flash.MERRIFIELD or self.__configuration.flash.MOOREFIELD:
            # FORCE_SHTN+COLD_RESET, FORCE_SHTN+COLD_OFF
            return "0x0a", "0x09"
        else:
            # FORCE_SHTN+COLD_RESET, FORCE_SHTN+COLD_OFF, FORCE_SHTN+COLD_BOOT
            return "0x21", "0x28", "0x24"

    def pattern(self, *args):
        """ Returns the string value 'RESETIRQ1=0x03' for example computed thanks to bit names and HARDWARE.

            ie: AND : pattern("COLD_OFF", "FORCE_SHTN"),
                             returns "RESETIRQ2=0x09"
                      PS: No symbol is allowed in AND case
            ie: OR  : pattern("COLD_OFF|FORCE_SHTN"),
                             returns "RESETIRQ2=0x(08|20)"
            ie: NOT : pattern("COLD_OFF|FORCE_SHTN|!COLD_BOOT"),
                             returns "RESETIRQ2=0x<all combinations with the bits AND without COLD_BOOT>"
            ie: OR^ : pattern("^COLD_OFF|^FORCE_SHTN"),
                             returns "RESETIRQ2=0x<all combination with the bits>"
        """
        if self.__configuration.flash.MERRIFIELD or self.__configuration.flash.MOOREFIELD:
            resetirq_dict = {"COLD_OFF"     : 0x01,
                             "COLD_RESET"   : 0x02,
                             "WARM_RESET"   : 0x04,
                             "FORCE_SHTN"   : 0x08,
                             "WD_RESET"     : 0x10,
                             "COLD_BOOT"    : 0x20}
            if self.__configuration.flash.MERRIFIELD:
                # No COLD_BOOT in saltbay so COLD_BOOT=COLD_RESET
                resetirq_dict["COLD_BOOT"] = resetirq_dict["COLD_RESET"]
        else:
            resetirq_dict = {"COLD_RESET"   : 0x01,
                             "WARM_RESET"   : 0x02,
                             "COLD_BOOT"    : 0x04,
                             "COLD_OFF"     : 0x08,
                             "EXTREset"     : 0x10,
                             "FORCE_SHTN"   : 0x20,
                             "WD_RESET"     : 0x40}
        bitsize = len(bin(max(resetirq_dict.values()))[2:])

        log = "pattern(): "
        self.__logger.printLog("INFO", log + "compute keys %s" % str(args))
        result = list()

        if len(args) == 0:
            self.__output.appendOutput("no argument", False)
            return False

        # If one argument with regexp '|'
        elif len(args) == 1 and "|" in str(args):

            arg = "".join(args)
            arg_names = re.findall("[^|]+", arg)
            # If only '|'* then fail
            if len(arg_names) == 0:
                self.__output.appendOutput("bad argument {0}".format(arg), False)
                return False
            else:
                # Here we have arg="COLD_OFF|FORCE_SHTN"
                # replace in arg the key by the value
                arg_rem = "^(?!"
                for el in arg_names:

                    #
                    # If 'not' symbol
                    #
                    if el[0] == "!":
                        arg = arg.replace("|" + el, "")
                        arg = arg.replace(el, "")
                        if el[1:] in resetirq_dict:
                            rem = bin(resetirq_dict[el[1:]])[2:].zfill(bitsize).replace("0", ".")
                            arg_rem += rem + "|"
                        else:
                            self.__output.appendOutput("unknown key '{0}'".format(el), False)
                            return False
                    #
                    # If 'exclusive' symbol ('....1...')
                    #
                    elif el[0] == "^":
                        if el[1:] in resetirq_dict:
                            rep = bin(resetirq_dict[el[1:]])[2:].zfill(bitsize).replace("0", ".")
                            arg = arg.replace(el, rep)
                        else:
                            self.__output.appendOutput("unknown key '{0}'".format(el), False)
                            return False
                    #
                    # If no symbol ('00001000')
                    #
                    elif el in resetirq_dict:
                        rep = bin(resetirq_dict[el])[2:].zfill(bitsize)
                        arg = arg.replace(el, rep)
                    else:
                        self.__output.appendOutput("unknown key '{0}'".format(el), False)
                        return False

                if arg_rem == "^(?!":
                    arg_rem = ""
                else:
                    arg_rem = arg_rem[0:-1] + ").*"
                    # Here we have arg="......1|....1..." and arg_rem="^(?!..1.....|11.....).*"

                self.__logger.printLog("DEBUG", log + "regexp '%s'" % arg)
                if arg_rem:
                    self.__logger.printLog("DEBUG", log + "removing regexp '%s'" % arg_rem)

                #
                # For each possibly N bits combination, keep if arg regexp matches
                #

                for i in ["".join(seq) for seq in itertools.product("01", repeat=bitsize)]:
                    # If nothing to remove (regexp with exclusion)
                    if re.search(arg_rem, i):
                        s = re.search(arg, i)
                        if s: result.append("%02x" % int(s.group(), 2))
                result = "(" + "|".join(result) + ")"

        # Else no regexp allowed
        else:
            result = 0
            for el in args:
                if "|" in el:
                    self.__output.appendOutput("'|' not allowed: {0}".format(el), False)
                    return False
                if el in resetirq_dict:
                    result += resetirq_dict[el]
                else:
                    self.__output.appendOutput("unknown key '{0}'".format(el), False)
                    return False
            # Here we have result=0x02+0x08...
            result = "%02x" % result

        ret = "{0}={1}".format(self.name().lower(), result)
        self.__logger.printLog("INFO", log + ret)
        self.__output.appendOutput("", True)
        return ret