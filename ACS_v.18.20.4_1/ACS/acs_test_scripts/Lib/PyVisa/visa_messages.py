#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0141,W0142,W0401,W0402,W0614,W0621
#
#    visa_messages.py - VISA completion and error messages
#
#    Copyright © 2005, 2006, 2007, 2008
#                Torsten Bronger <bronger@physik.rwth-aachen.de>,
#                Gregor Thalhammer <gth@users.sourceforge.net>.
#
#    This file is part of PyVISA.
#
#    PyVISA is free software; you can redistribute it and/or modify it under
#    the terms of the MIT licence:
#
#    Permission is hereby granted, free of charge, to any person obtaining a
#    copy of this software and associated documentation files (the "Software"),
#    to deal in the Software without restriction, including without limitation
#    the rights to use, copy, modify, merge, publish, distribute, sublicense,
#    and/or sell copies of the Software, and to permit persons to whom the
#    Software is furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be included in
#    all copies or substantial portions of the Software.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#    DEALINGS IN THE SOFTWARE.
#

"""Textual explanations of VISA completion and error codes.

The only export is "completion_and_error_messages", which is a dictionary that
binds every completion and error value to its name and a short textual
description (both in a two-elements tuple).

"""

__version__ = "$Revision: 1.5 $"
# $Source: /cvsroot/pyvisa/pyvisa/src/visa_messages.py,v $


from vpp43_constants import *


# Don't export the VPP-4.3 constants, but only the messages.
__all__ = ("completion_and_error_messages",)


completion_and_error_messages = {

    VI_SUCCESS: ("VI_SUCCESS",
                 "Operation completed successfully."),
    VI_SUCCESS_EVENT_EN: ("VI_SUCCESS_EVENT_EN",
                          "Specified event is already enabled for at "
                          "least one of the specified mechanisms."),
    VI_SUCCESS_EVENT_DIS: ("VI_SUCCESS_EVENT_DIS",
                           "Specified event is already disabled for "
                           "at least one of the specified mechanisms."),
    VI_SUCCESS_QUEUE_EMPTY: ("VI_SUCCESS_QUEUE_EMPTY",
                             "Operation completed successfully, but "
                             "queue was already empty."),
    VI_SUCCESS_TERM_CHAR: ("VI_SUCCESS_TERM_CHAR",
                           "The specified termination character was "
                           "read."),
    VI_SUCCESS_MAX_CNT: ("VI_SUCCESS_MAX_CNT",
                         "The number of bytes transferred is equal "
                         "to the requested input count. More data "
                         "may be available."),
    VI_SUCCESS_DEV_NPRESENT: ("VI_SUCCESS_DEV_NPRESENT",
                              "Session opened successfully, but the "
                              "device at the specified address is not "
                              "responding."),
    VI_SUCCESS_TRIG_MAPPED: ("VI_SUCCESS_TRIG_MAPPED",
                             "The path from trigSrc to trigDest is "
                             "already mapped."),
    VI_SUCCESS_QUEUE_NEMPTY: ("VI_SUCCESS_QUEUE_NEMPTY",
                              "Wait terminated successfully on receipt "
                              "of an event notification. There is at "
                              "least one more event object of the "
                              "requested type(s) available for this "
                              "session."),
    VI_SUCCESS_NCHAIN: ("VI_SUCCESS_NCHAIN",
                        "Event handled successfully. Do not invoke "
                        "any other handlers on this session for "
                        "this event."),
    VI_SUCCESS_NESTED_SHARED: ("VI_SUCCESS_NESTED_SHARED",
                               "Operation completed successfully, and "
                               "this session has nested shared locks."),
    VI_SUCCESS_NESTED_EXCLUSIVE: ("VI_SUCCESS_NESTED_EXCLUSIVE",
                                  "Operation completed successfully, and "
                                  "this session has nested exclusive locks."),
    VI_SUCCESS_SYNC: ("VI_SUCCESS_SYNC",
                      "Operation completed successfully, but the "
                      "operation was actually synchronous rather "
                      "than asynchronous."),
    VI_WARN_QUEUE_OVERFLOW: ("VI_WARN_QUEUE_OVERFLOW",
                             "VISA received more event information of "
                             "the specified type than the configured "
                             "queue size could hold."),
    VI_WARN_CONFIG_NLOADED: ("VI_WARN_CONFIG_NLOADED",
                             "The specified configuration either does "
                             "not exist or could not be loaded. "
                             "VISA-specified defaults will be used."),
    VI_WARN_NULL_OBJECT: ("VI_WARN_NULL_OBJECT",
                          "The specified object reference is "
                          "uninitialized."),
    VI_WARN_NSUP_ATTR_STATE: ("VI_WARN_NSUP_ATTR_STATE",
                              "Although the specified state of the "
                              "attribute is valid, it is not supported "
                              "by this implementation."),
    VI_WARN_UNKNOWN_STATUS: ("VI_WARN_UNKNOWN_STATUS",
                             "The status code passed to the operation "
                             "could not be interpreted."),
    VI_WARN_NSUP_BUF: ("VI_WARN_NSUP_BUF",
                       "The specified I/O buffer type is not "
                       "supported."),
    VI_WARN_EXT_FUNC_NIMPL: ("VI_WARN_EXT_FUNC_NIMPL",
                             "The operation succeeded, but a lower "
                             "level driver did not implement the "
                             "extended functionality."),
    VI_ERROR_SYSTEM_ERROR: ("VI_ERROR_SYSTEM_ERROR",
                            "Unknown system error (miscellaneous "
                            "error)."),
    VI_ERROR_INV_OBJECT: ("VI_ERROR_INV_OBJECT",
                          "The given session or object reference is "
                          "invalid."),
    VI_ERROR_RSRC_LOCKED: ("VI_ERROR_RSRC_LOCKED",
                           "Specified type of lock cannot be "
                           "obtained, or specified operation cannot "
                           "be performed, because the resource is "
                           "locked."),
    VI_ERROR_INV_EXPR: ("VI_ERROR_INV_EXPR",
                        "Invalid expression specified for search."),
    VI_ERROR_RSRC_NFOUND: ("VI_ERROR_RSRC_NFOUND",
                           "Insufficient location information or the "
                           "requested device or resource is not "
                           "present in the system."),
    VI_ERROR_INV_RSRC_NAME: ("VI_ERROR_INV_RSRC_NAME",
                             "Invalid resource reference specified. "
                             "Parsing error."),
    VI_ERROR_INV_ACC_MODE: ("VI_ERROR_INV_ACC_MODE",
                            "Invalid access mode."),
    VI_ERROR_TMO: ("VI_ERROR_TMO",
                   "Timeout expired before operation "
                   "completed."),
    VI_ERROR_CLOSING_FAILED: ("VI_ERROR_CLOSING_FAILED",
                              "The VISA driver failed to properly close "
                              "the session or object reference. This "
                              "might be due to an error freeing internal "
                              "or OS resources, a failed network "
                              "connection, or a lower-level driver or OS "
                              "error."),
    VI_ERROR_INV_DEGREE: ("VI_ERROR_INV_DEGREE",
                          "Specified degree is invalid."),
    VI_ERROR_INV_JOB_ID: ("VI_ERROR_INV_JOB_ID",
                          "Specified job identifier is invalid."),
    VI_ERROR_NSUP_ATTR: ("VI_ERROR_NSUP_ATTR",
                         "The specified attribute is not defined or "
                         "supported by the referenced object."),
    VI_ERROR_NSUP_ATTR_STATE: ("VI_ERROR_NSUP_ATTR_STATE",
                               "The specified state of the attribute is "
                               "not valid, or is not supported as defined "
                               "by the object."),
    VI_ERROR_ATTR_READONLY: ("VI_ERROR_ATTR_READONLY",
                             "The specified attribute is read-only."),
    VI_ERROR_INV_LOCK_TYPE: ("VI_ERROR_INV_LOCK_TYPE",
                             "The specified type of lock is not "
                             "supported by this resource."),
    VI_ERROR_INV_ACCESS_KEY: ("VI_ERROR_INV_ACCESS_KEY",
                              "The access key to the resource associated "
                              "with the specified session is invalid."),
    VI_ERROR_INV_EVENT: ("VI_ERROR_INV_EVENT",
                         "Specified event type is not supported by "
                         "the resource."),
    VI_ERROR_INV_MECH: ("VI_ERROR_INV_MECH",
                        "Invalid mechanism specified."),
    VI_ERROR_HNDLR_NINSTALLED: ("VI_ERROR_HNDLR_NINSTALLED",
                                "A handler was not installed."),
    VI_ERROR_INV_HNDLR_REF: ("VI_ERROR_INV_HNDLR_REF",
                             "The given handler reference is either "
                             "invalid or was not installed."),
    VI_ERROR_INV_CONTEXT: ("VI_ERROR_INV_CONTEXT",
                           "Specified event context is invalid."),
    VI_ERROR_QUEUE_OVERFLOW: ("VI_ERROR_QUEUE_OVERFLOW",
                              "The event queue for the specified type "
                              "has overflowed (usually due to previous "
                              "events not having been closed)."),
    VI_ERROR_NENABLED: ("VI_ERROR_NENABLED",
                        "You must be enabled for events of the "
                        "specified type in order to receive them."),
    VI_ERROR_ABORT: ("VI_ERROR_ABORT",
                     "User abort occurred during transfer."),
    VI_ERROR_RAW_WR_PROT_VIOL: ("VI_ERROR_RAW_WR_PROT_VIOL",
                                "Violation of raw write protocol occurred "
                                "during transfer."),
    VI_ERROR_RAW_RD_PROT_VIOL: ("VI_ERROR_RAW_RD_PROT_VIOL",
                                "Violation of raw read protocol occurred "
                                "during transfer."),
    VI_ERROR_OUTP_PROT_VIOL: ("VI_ERROR_OUTP_PROT_VIOL",
                              "Device reported an output protocol error "
                              "during transfer."),
    VI_ERROR_INP_PROT_VIOL: ("VI_ERROR_INP_PROT_VIOL",
                             "Device reported an input protocol error "
                             "during transfer."),
    VI_ERROR_BERR: ("VI_ERROR_BERR",
                    "Bus error occurred during transfer."),
    VI_ERROR_IN_PROGRESS: ("VI_ERROR_IN_PROGRESS",
                           "Unable to queue the asynchronous "
                           "operation because there is already an "
                           "operation in progress."),
    VI_ERROR_INV_SETUP: ("VI_ERROR_INV_SETUP",
                         "Unable to start operation because setup "
                         "is invalid (usually due to attributes "
                         "being set to an inconsistent state)."),
    VI_ERROR_QUEUE_ERROR: ("VI_ERROR_QUEUE_ERROR",
                           "Unable to queue the asynchronous "
                           "operation (usually due to the I/O "
                           "completion event not being enabled or "
                           "insufficient space in the session's "
                           "queue)."),
    VI_ERROR_ALLOC: ("VI_ERROR_ALLOC",
                     "Insufficient system resources to perform "
                     "necessary memory allocation."),
    VI_ERROR_INV_MASK: ("VI_ERROR_INV_MASK",
                        "Invalid buffer mask specified."),
    VI_ERROR_IO: ("VI_ERROR_IO",
                  "Could not perform operation because of "
                  "I/O error."),
    VI_ERROR_INV_FMT: ("VI_ERROR_INV_FMT",
                       "A format specifier in the format string "
                       "is invalid."),
    VI_ERROR_NSUP_FMT: ("VI_ERROR_NSUP_FMT",
                        "A format specifier in the format string "
                        "is not supported."),
    VI_ERROR_LINE_IN_USE: ("VI_ERROR_LINE_IN_USE",
                           "The specified trigger line is currently "
                           "in use."),
    VI_ERROR_NSUP_MODE: ("VI_ERROR_NSUP_MODE",
                         "The specified mode is not supported by "
                         "this VISA implementation."),
    VI_ERROR_SRQ_NOCCURRED: ("VI_ERROR_SRQ_NOCCURRED",
                             "Service request has not been received for "
                             "the session."),
    VI_ERROR_INV_SPACE: ("VI_ERROR_INV_SPACE",
                         "Invalid address space specified."),
    VI_ERROR_INV_OFFSET: ("VI_ERROR_INV_OFFSET",
                          "Invalid offset specified."),
    VI_ERROR_INV_WIDTH: ("VI_ERROR_INV_WIDTH",
                         "Invalid access width specified."),
    VI_ERROR_NSUP_OFFSET: ("VI_ERROR_NSUP_OFFSET",
                           "Specified offset is not accessible from "
                           "this hardware."),
    VI_ERROR_NSUP_VAR_WIDTH: ("VI_ERROR_NSUP_VAR_WIDTH",
                              "Cannot support source and destination "
                              "widths that are different."),
    VI_ERROR_WINDOW_NMAPPED: ("VI_ERROR_WINDOW_NMAPPED",
                              "The specified session is not currently "
                              "mapped."),
    VI_ERROR_RESP_PENDING: ("VI_ERROR_RESP_PENDING",
                            "A previous response is still pending, "
                            "causing a multiple query error."),
    VI_ERROR_NLISTENERS: ("VI_ERROR_NLISTENERS",
                          "No listeners condition is detected (both "
                          "NRFD and NDAC are deasserted)."),
    VI_ERROR_NCIC: ("VI_ERROR_NCIC",
                    "The interface associated with this "
                    "session is not currently the controller "
                    "in charge."),
    VI_ERROR_NSYS_CNTLR: ("VI_ERROR_NSYS_CNTLR",
                          "The interface associated with this "
                          "session is not the system controller."),
    VI_ERROR_NSUP_OPER: ("VI_ERROR_NSUP_OPER",
                         "The given session or object reference "
                         "does not support this operation."),
    VI_ERROR_INTR_PENDING: ("VI_ERROR_INTR_PENDING",
                            "An interrupt is still pending from a "
                            "previous call."),
    VI_ERROR_ASRL_PARITY: ("VI_ERROR_ASRL_PARITY",
                           "A parity error occurred during transfer."),
    VI_ERROR_ASRL_FRAMING: ("VI_ERROR_ASRL_FRAMING",
                            "A framing error occurred during transfer."),
    VI_ERROR_ASRL_OVERRUN: ("VI_ERROR_ASRL_OVERRUN",
                            "An overrun error occurred during "
                            "transfer. A character was not read from "
                            "the hardware before the next character "
                            "arrived."),
    VI_ERROR_TRIG_NMAPPED: ("VI_ERROR_TRIG_NMAPPED",
                            "The path from trigSrc to trigDest is not "
                            "currently mapped."),
    VI_ERROR_NSUP_ALIGN_OFFSET: ("VI_ERROR_NSUP_ALIGN_OFFSET",
                                 "The specified offset is not properly "
                                 "aligned for the access width of the "
                                 "operation."),
    VI_ERROR_USER_BUF: ("VI_ERROR_USER_BUF",
                        "A specified user buffer is not valid or "
                        "cannot be accessed for the required size."),
    VI_ERROR_RSRC_BUSY: ("VI_ERROR_RSRC_BUSY",
                         "The resource is valid, but VISA cannot "
                         "currently access it."),
    VI_ERROR_NSUP_WIDTH: ("VI_ERROR_NSUP_WIDTH",
                          "Specified width is not supported by this "
                          "hardware."),
    VI_ERROR_INV_PARAMETER: ("VI_ERROR_INV_PARAMETER",
                             "The value of some parameter (which "
                             "parameter is not known) is invalid."),
    VI_ERROR_INV_PROT: ("VI_ERROR_INV_PROT",
                        "The protocol specified is invalid."),
    VI_ERROR_INV_SIZE: ("VI_ERROR_INV_SIZE",
                        "Invalid size of window specified."),
    VI_ERROR_WINDOW_MAPPED: ("VI_ERROR_WINDOW_MAPPED",
                             "The specified session currently contains "
                             "a mapped window."),
    VI_ERROR_NIMPL_OPER: ("VI_ERROR_NIMPL_OPER",
                          "The given operation is not implemented."),
    VI_ERROR_INV_LENGTH: ("VI_ERROR_INV_LENGTH",
                          "Invalid length specified."),
    VI_ERROR_INV_MODE: ("VI_ERROR_INV_MODE",
                        "Invalid mode specified."),
    VI_ERROR_SESN_NLOCKED: ("VI_ERROR_SESN_NLOCKED",
                            "The current session did not have a lock "
                            "on the resource."),
    VI_ERROR_MEM_NSHARED: ("VI_ERROR_MEM_NSHARED",
                           "The device does not export any memory."),
    VI_ERROR_LIBRARY_NFOUND: ("VI_ERROR_LIBRARY_NFOUND",
                              "A code library required by VISA could not "
                              "be located or loaded."),
    VI_ERROR_NSUP_INTR: ("VI_ERROR_NSUP_INTR",
                         "The interface cannot generate an "
                         "interrupt on the requested level or with "
                         "the requested statusID value."),
    VI_ERROR_INV_LINE: ("VI_ERROR_INV_LINE",
                        "The value specified by the line parameter "
                        "is invalid."),
    VI_ERROR_FILE_ACCESS: ("VI_ERROR_FILE_ACCESS",
                           "An error occurred while trying to open "
                           "the specified file. Possible reasons "
                           "include an invalid path or lack of access "
                           "rights."),
    VI_ERROR_FILE_IO: ("VI_ERROR_FILE_IO",
                       "An error occurred while performing I/O on "
                       "the specified file."),
    VI_ERROR_NSUP_LINE: ("VI_ERROR_NSUP_LINE",
                         "One of the specified lines (trigSrc or "
                         "trigDest) is not supported by this VISA "
                         "implementation, or the combination of "
                         "lines is not a valid mapping."),
    VI_ERROR_NSUP_MECH: ("VI_ERROR_NSUP_MECH",
                         "The specified mechanism is not supported "
                         "for the given event type."),
    VI_ERROR_INTF_NUM_NCONFIG: ("VI_ERROR_INTF_NUM_NCONFIG",
                                "The interface type is valid but the "
                                "specified interface number is not "
                                "configured."),
    VI_ERROR_CONN_LOST: ("VI_ERROR_CONN_LOST",
                         "The connection for the given session has "
                         "been lost."),
    VI_ERROR_MACHINE_NAVAIL: ("VI_ERROR_MACHINE_NAVAIL",
                              "The remote machine does not exist or is "
                              "not accepting any connections. If the "
                              "NI-VISA server is installed and running "
                              "on the remote machine, it may have an "
                              "incompatible version or may be listening "
                              "on a different port."),
    VI_ERROR_NPERMISSION: ("VI_ERROR_NPERMISSION",
                           "Access to the resource or remote machine "
                           "is denied. This is due to lack of "
                           "sufficient privileges for the current "
                           "user or machine")
}
"""A dictionary that holds the name and a short description of all completion
and error codes in the VISA specification.  The name and the description are
strings in a tupel.
"""
