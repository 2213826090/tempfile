"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: Enable running Google JUnit in ACS
:since: 03/07/2012
:author: sfusilie
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from tempfile import gettempdir
import os
import re
import urllib2


class AndroidJUnit(UseCaseBase):

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        # Get TC Parameters
        self._apk_path = \
            self._tc_parameters.get_param_value("APK_PATH")
        self._test_cmd_lines = \
            self._tc_parameters.get_param_value("TEST_CMD_LINES")
        self._test_timeout = \
            self._tc_parameters.get_param_value("TEST_TIMEOUT")

    def __grabUrls(self, text):
        """Given a text str, returns all the urls we can find in it."""

        urls = '(?: %s)' % '|'.join("""http telnet gopher file wais
        ftp""".split())
        ltrs = r'\w'
        gunk = r'/#~:.?+=&%@!\-'
        punc = r'.:?\-'
        any_chars = "%(ltrs)s%(gunk)s%(punc)s" % {'ltrs': ltrs,
                                                  'gunk': gunk,
                                                  'punc': punc}

        url = r"""
            \b                            # start at word boundary
                %(urls)s    :             # need resource and a colon
                [%(any)s]  +?             # followed by one or more
                                          #  of any valid character, but
                                          #  be conservative and take only
                                          #  what you need to....
            (?=                           # look-ahead non-consumptive assertion
                    [%(punc)s]*           # either 0 or more punctuation
                    (?:   [^%(any)s]      #  followed by a non-url char
                        |                 #   or end of the string
                          $
                    )
            )
            """ % {'urls': urls,
                   'any': any_chars,
                   'punc': punc}

        url_re = re.compile(url, re.VERBOSE | re.MULTILINE)

        return url_re.findall(text)

    def __retrieve_files(self, urls):
        proxy_support = urllib2.ProxyHandler({})

        dest_files = []
        for url in urls:
            opener = urllib2.build_opener(proxy_support)
            urllib2.install_opener(opener)

            dest_file_name = os.path.join(gettempdir(),
                                          url.rstrip('/').split('/')[-1])

            remote_file = urllib2.urlopen(url)
            output = open(dest_file_name, "wb")
            output.write(remote_file.read())
            remote_file.close()
            output.close()

            dest_files.append(dest_file_name)

        return dest_files

    def __get_test_path(self, script_path):
        new_path = script_path
        if not os.path.exists(script_path):
            new_path = \
                os.path.join(self._execution_config_path, script_path)

        if not os.path.exists(new_path):
            new_path = \
                os.path.join(self._execution_config_path,
                             os.path.dirname(self._name),
                             script_path)

        if not os.path.exists(new_path):
            return Global.FAILURE, new_path
        else:
            return Global.SUCCESS, new_path

    def set_up(self):
        UseCaseBase.set_up(self)

        if self._test_timeout is None:
            return (Global.FAILURE, "You need to specify a "
                                    "TESTS_TIMEOUT value.")

        if self._apk_path is None:
            return (Global.FAILURE, "You need to specify a "
                                    "APK_PATH value.")

        if self._test_cmd_lines is None:
            return (Global.FAILURE, "You need to specify a "
                                    "TEST_PACKAGES_NAMES value.")

        if self._apk_path is not None:
            # Test need to be pushed on the device
            test_bin_url = self.__grabUrls(self._apk_path)
            if len(test_bin_url) > 0:
                # Test is available thru URL => donwload it localy
                self._apk_path = self.__retrieve_files(test_bin_url)
            else:
                # Types not inferred
                paths = self._apk_path.split(";")  # pylint: disable=E1103
                self._apk_path = []
                for path in paths:
                    result, full_path = \
                        self.__get_test_path(path)
                    if result == Global.FAILURE:
                        return result, "Cannot find %s" % path
                    else:
                        self._apk_path.append(full_path)

            # Push the test on the device
            for apk_path in self._apk_path:
                self._device.install_file(apk_path)

        return Global.SUCCESS, ""

    def run_test(self):
        UseCaseBase.run_test(self)

        output_return = []
        for test_cmd_line in self._test_cmd_lines.split(";"):
            test_cmd_line = test_cmd_line.strip()

            result, output = self._device.run_cmd(test_cmd_line,
                                                  self._test_timeout)

            if result != Global.SUCCESS or ("Failures: 0,  Errors: 0" not in output and "OK (1 test)" not in output):
                return Global.FAILURE, output
            else:
                output_return.append(output)

        return Global.SUCCESS, output
