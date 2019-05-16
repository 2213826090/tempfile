"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA
:description: This is a Python UIAutomator script that prints total space for
 internal storage from Settings app. Before calling this script, make sure that
 the Storage activity of Settings is opened.
:since: 2/28/15
:author: dmdumitx
"""
from uiautomator import device as d

if d(packageName="com.android.settings").wait.exists(timeout=10000):
        obj = d(text="Internal storage")
        if d(textContains="Calculating").wait.gone(timeout=30000):
                if obj.down(text="Total space").sibling(resourceId="android:id/summary"):
                        if obj.down(text="Available").sibling(resourceId="android:id/summary"):
                                print(obj.down(text="Total space").sibling(resourceId="android:id/summary").info['text'])
                                print(obj.down(text="Available").sibling(resourceId="android:id/summary").info['text'])
                        else:
                                print("Available space value not found in Settings")
                else:
                        print("Total space value not found in Settings")
        else:
                print("Timeout reached, still calculating values in Settings")
else:
        print("Settings app not opened")
