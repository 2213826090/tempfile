from testlib.util.uiatestbase import UIATestBase
from testlib.tts.tts_impl import TTS_Impl

class TTS(UIATestBase):
    """
    @summary: Test TTS
    """
    def setUp(self):
        super(TTS, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.tts = TTS_Impl()

    def tearDown(self):
        super(TTS, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testTTS(self):
        """
        This test used to test TTS.
        The test case spec is following:

        1. Launch "settings" app, select "Accessibility", then select "Text-to-speech output".
        2. Click the setting icon in the right of "Pico TTS", select a language.
        3. Click "Listen to an example".
        4. repeat step 2-3, change the language.
        """
        print "[RunTest]: %s" % self.__str__()

        self.tts.play_tts_example()
