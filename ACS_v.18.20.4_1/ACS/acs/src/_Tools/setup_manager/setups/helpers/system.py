#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -------------------------------------------------------------------------------
# @copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.

# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.

# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.

# @organization: INTEL MCG PSI
# @summary: <enter_summary_details_here>
# @since: 4/18/14
# @author: nbrissox
# -------------------------------------------------------------------------------
import os

from setups import logger

DEFAULT_PROXY_SETTINGS = {
    'http': 'http://proxy-ir.intel.com:911'
}


def setup_proxy(environ=None,
                http=DEFAULT_PROXY_SETTINGS['http'],
                https=DEFAULT_PROXY_SETTINGS['http'],
                reset=False):
    """
    Sets the proxy environment variables, if not already set.
    Or reset it (for this current python session)

    :param environ: (optional)
    :type environ: dict

    :param http: (optional)
    :type http: str

    :param https: (optional)
    :type https: str

    :param reset: if True all PROXIES settings are disabled for the session.
    :type reset: bool

    """
    environ = environ or os.environ
    if reset:
        logger.info("Resetting HTTP_PROXY...")
        environ["NO_PROXY"] = ".intel.com"
        if "HTTP_PROXY" in environ:
            del environ["HTTP_PROXY"]
        if "HTTPS_PROXY" in environ:
            del environ["HTTPS_PROXY"]
    else:
        if 'HTTP_PROXY' not in environ:
            logger.info("Setting HTTP_PROXY to " + http)
            environ['HTTP_PROXY'] = http
        else:
            logger.info("HTTP Proxy Already Set: " + environ['HTTP_PROXY'])
        if 'HTTPS_PROXY' not in environ:
            logger.info("Setting HTTPS_PROXY to " + https)
            environ['HTTPS_PROXY'] = https
        else:
            logger.info("HTTPS Proxy Already Set: " + environ['HTTPS_PROXY'])


def setup_exception(class_name, exception_msg):
    """
    Instantiate and Returns on the fly an Exception subclass instance,
    with given Class Name (class_name parameter) and given exception message.

    .. warning:: The Class Name parameter MUST BE CapWords

        .. seealso: Python Coding Conventions `PEP 8 <http://legacy.python.org/dev/peps/pep-0008/#class-names>`_
        .. seealso: Python Docstring Conventions `PEP 257<http://legacy.python.org/dev/peps/pep-0257/>`_

    :param class_name: The Exception Class Name
    :type class_name: str

    :param exception_msg: The Exception Massage
    :type exception_msg: str

    :return: Arbitrary Exception sub Class instance
    :rtype: Exception

    """
    return type(class_name, (Exception,), {})(exception_msg)
