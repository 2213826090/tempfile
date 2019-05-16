#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
@summary: This module implements Sphinx Auto-generator of Documentation
@since: 17/12/14
@author: kturban
"""

import os
from UtilitiesFWK.Parsers.YamlParser import YamlParser

comparison = []
skip = False

# quick and dirty script to ensure yaml integrity files

for file in [x for x in os.listdir(os.getcwd()) if x.endswith(".yaml")]:
    print "test %s" % file
    diff_keys = set()
    previous_keys = set()
    previous_file = ""
    try:
        parser = YamlParser(file)
    except Exception as e:
        print "cannot pars {0} : {1}".format(file, e)
    content = parser.get("ModuleConfiguration")
    my_keys = set([x.tag for x in content])
    for other_file in [x for x in os.listdir(os.getcwd()) if x != file and x.endswith(".yaml")]:
        skip = False
        for x in comparison:
            if set([file, other_file]) == x:
                skip = True
        if skip:
            continue
        comparison.append(set([file, other_file]))
        try:
            parser = YamlParser(other_file)
        except Exception as e:
            print "cannot pars {0} : {1}".format(other_file, e)
        content = parser.get("ModuleConfiguration")
        other_keys =  set([x.tag for x in content])
        difference = my_keys.difference(other_keys)
        if difference:
            print "keys : {0} are present in {1} but there are missing in {2}".format(" , ".join((str(e) for e in difference)), file, other_file)

        difference = other_keys.difference(my_keys)
        if difference:
            print "keys : {0} are present in {1} but there are missing in {2}".format(" , ".join((str(e) for e in difference)), other_file, file)
