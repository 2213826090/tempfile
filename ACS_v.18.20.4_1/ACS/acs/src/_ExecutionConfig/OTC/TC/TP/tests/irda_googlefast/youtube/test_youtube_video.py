#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: Test play youtube video
@since: 08/21/2014
@author: Grace Yi (gracex.yi@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.youtube.youtube_impl import YoutubeImpl

class YoutubeVideoTest(UIATestBase):
    """
    @summary: Test play youtube video
    """
    def setUp(self):
        super(YoutubeVideoTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.google_fast.conf'
        self.youtube = YoutubeImpl(\
            self.config.read(cfg_file, 'youtube'))
        self.youtube.set_orientation_n()

    def tearDown(self):
        super(YoutubeVideoTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testYoutubeVideo(self):
        """
        Test play youtube video
        """
        print "[RunTest]: %s" % self.__str__()
        self.youtube.youtube_first_movie_play()
