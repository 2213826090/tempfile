#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "nbrissox"
__since__ = "8/19/14"
__organization__ = "INTEL MCG PSI"

__copyright__ = """(c)Copyright 2014, Intel Corporation All Rights Reserved.
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
and approved by Intel in writing."""

"""
:summary: This script convert a regular `pip` requirement file
          into JSON format for each package.

"""
import sys
import os

from os import path


def absjoin(*fragments):
    """
    Performs an join on given path fragments, then an abspath from :mod:`os.path` module

    :param fragments: Path fragments to be joined and rendered as absolute
    :type fragments: tuple, list

    :return: The joined absolute path from given fragments
    :rtype: str, unicode

    """
    return path.abspath(path.join(*fragments))

TOP_DIR = absjoin(path.dirname(__file__))
PACKAGE_TOP_DIR = absjoin(TOP_DIR, '..', '..')

if PACKAGE_TOP_DIR not in sys.path:
    sys.path.append(PACKAGE_TOP_DIR)


# from here, we can import module(s) from the package top level
try:
    import argparse
except ImportError:
    from setups.vendors import argparse


PACKAGE_TEMPLATE = '''{{
            "name": "{name}",
            "import_name": "{name}",
            "version": "",
            "linux": {{
                "pre_install": [],
                "post_install": []
             }},
            "windows": {{
                "pre_install": [],
                "post_install": []
             }}
        }}'''


def make_parser(description=__doc__):
    """
    Makes a parser instance

    :return: A parser instance
    :rtype: argparse.ArgumentParser

    """

    parser = argparse.ArgumentParser(description=description)

    parser.add_argument('-f', '--file', type=str, dest='filename')
    parser.add_argument('-o', '--output-dir', type=str, dest='output_dir', default="")

    return parser


def main(args=None):
    """
    Script Entry Point.

    :return: The Process status
    :rtype: int

    """
    args = args or sys.argv[1:]
    status = 0

    parser = make_parser()
    options = parser.parse_args(args)

    output = options.output_dir or path.dirname(options.filename)

    if not output.endswith('packages'):
        output = absjoin(output, 'packages')

    if not path.isdir(output):
        try:
            os.makedirs(output)
        except (IOError, OSError) as error:
            sys.stderr.write('{0}\n'.format(error))
            status = -1

    if status != -1:
        with open(options.filename, 'r') as handle:
            lines = handle.readlines()

        for line in lines:
            stripped = line.strip()

            if not stripped:
                continue

            cleaned_stripped = str(stripped.split('=')[0]).strip()
            destination = absjoin(output, '{0}.json'.format(cleaned_stripped.lower()))

            if path.exists(destination):
                continue

            with open(destination, 'w+') as handle:
                handle.write(PACKAGE_TEMPLATE.format(name=cleaned_stripped))

    return status


if __name__ == '__main__':
    sys.exit(main())
