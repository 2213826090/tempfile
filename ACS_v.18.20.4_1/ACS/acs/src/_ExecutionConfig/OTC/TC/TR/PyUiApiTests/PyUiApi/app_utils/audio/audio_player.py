from PyUiApi.adb_helper.instrumentation_utils import *


class AudioPlayer(object):
    pass


class InstrumentationAudioPlayer(AudioPlayer):
    def __init__(self):
        super(InstrumentationAudioPlayer, self).__init__()

    def turn_dut_music_volume_to_max(self):
        ApiTestsInterface\
            .run_instrumentation(class_name="AudioPlayer",
                                 method_name="setMaxMusicStreamVolume",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")

    def play(self, dut_audio_file_path):
        instrumentation_args = ApiTestsGenericExtraArgs()
        test_args = instrumentation_args\
            .get_args_string(audioFilePath=dut_audio_file_path)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="AudioPlayer",
                                 method_name="playLocalAudioFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        return ApiTestsInterface.instrumentation_one_test_pass_output in result
