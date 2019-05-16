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
import logging

from os import path

SysErrors = IOError, OSError
try:
    SysErrors += WindowsError,
except (ImportError, NameError):
    pass


absjoin = lambda *paths: path.abspath(path.join(*paths)).replace('\\', '/')

DEBUG = 0

TOP_DIR = absjoin(path.dirname(__file__), '..')

CONFIG_TOP_DIR = absjoin(TOP_DIR, 'configs')
CONFIG_PROFILES_DIR = absjoin(CONFIG_TOP_DIR, 'profiles')
CONFIG_BINARIES_DIR = absjoin(CONFIG_TOP_DIR, 'binaries')
CONFIG_EQUIPMENTS_DIR = absjoin(CONFIG_TOP_DIR, 'equipments')
CONFIG_PIP_REQUIREMENTS_DIR = absjoin(CONFIG_TOP_DIR, 'pip')
CONFIG_PIP_PACKAGES_DIR = absjoin(CONFIG_PIP_REQUIREMENTS_DIR, 'packages')
CONFIG_PIP_CERT_FILE = absjoin(CONFIG_PIP_REQUIREMENTS_DIR, 'certs', 'ca-certificates.crt')

# Initialize the Global logger instance
LOG_DIRNAME = path.join(TOP_DIR, 'logs')
LOG_FILENAME = path.join(LOG_DIRNAME, 'acs_setup_tool.log')

logger = logging.getLogger("ACS_SETUP_MANAGER")
formatter = logging.Formatter('%(name)-12s %(levelname)-8s %(message)s')

from setups.helpers import ensuredirs

ensuredirs(LOG_DIRNAME)

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M',
                    filename=LOG_FILENAME)
console = logging.StreamHandler()
console.setLevel(logging.DEBUG if DEBUG else logging.INFO)
console.setFormatter(formatter)
logger.addHandler(console)
