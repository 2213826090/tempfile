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
import json
import os
import errno

from setups.helpers import types
from setups.helpers.system import setup_exception


def ensuredirs(path):
    """
    Ensures directory creation.

    :param path: the dirname path to be created.

    :raises: TypeError, OSError

    """
    try:
        # Python>3.2
        os.makedirs(path, exist_ok=True)
    except TypeError:
        try:
            os.makedirs(path)
            # Python >2.5
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(path):
                pass
            else:
                raise


class Loader(types.StringBase):
    """
    Loads data from configuration file.

    """

    @classmethod
    def load(cls, source_filename, parser='JSON'):
        """
        Loads Configuration data.

        :param source_filename:
        :type source_filename: str

        :param parser: The parser to be used for the configuration file (JSON, YAML, XML, ...)
        :type parser: str

        :raises: ConfigurationFileNotFound, ParserNotImplemented

        :return: The parsed content
        :rtype: dict

        .. todo:: For now only JSON parser is implemented, could be worth to implement more parsers ?

        """
        if os.path.exists(source_filename):
            with open(source_filename, 'r') as handle:
                content = handle.read()
                if parser == 'JSON':
                    return json.loads(content)
                elif parser == 'XML':
                    raise setup_exception('ParserNotImplemented', 'XMLParser is Undefined!')
        else:
            raise setup_exception('ConfigurationFileNotFound',
                                  'The file {0} does NOT exists!'.format(source_filename))