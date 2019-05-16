'''
Created on May 13, 2016

@author: bob
'''

from testlib.audio.audio_test_base import TelephonyTestBase

class AudioTunesMultiTest(TelephonyTestBase):
    PKG_MSG = 'com.google.android.apps.messaging'
    def setUp(self):
        super(AudioTunesMultiTest, self).setUp()
        self.d.adb_cmd("pm clear %s"%self.PKG_MSG)
        self.r_d.adb_cmd("pm clear %s"%self.PKG_MSG)

    def testReceive_AudioMMS(self):
        '''
        Verify that DUT could receive audio MMS
        '''
        self.r_d.telephony.send_audio_mms_to(self.d.telephony.get_number())
        self.d.telephony.wait_audio_mms()

    def testSend_AudioMMS(self):
        '''
        Verify that DUT could send audio MMS
        '''
        self.d.telephony.send_audio_mms_to(self.r_d.telephony.get_number())
        self.r_d.telephony.wait_audio_mms()