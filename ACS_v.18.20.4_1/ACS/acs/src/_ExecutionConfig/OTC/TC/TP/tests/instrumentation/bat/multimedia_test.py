from .common import BatTestBase


class TestMultimedia(BatTestBase):
    def testAudioPlayback(self):
        self.bat.instr_run_class('AudioTest#testPlayAudioFromDataURI')
        self.bat.instr_run_class('AudioTest#testPlayAudio')
        self.bat.instr_run_class('AudioTest#testRecordAndPlay')

    def testAudioOnLineoutJack(self):
        self.bat.instr_run_class('AudioTest#testAudioOnLineoutJack')

    def testVideoPlayback(self):
        self.bat.instr_run_class('VideoTest')

    def testVolumeControl(self):
        self.bat.instr_run_class('VolumeControlTest')

    def testCameraTakePictures(self):
        self.bat.instr_run_class('CameraTest#testTakePictureUsingMainCamera')
        self.bat.instr_run_class('CameraTest#testTakePictureUsingFrontCamera')

    def testCameraVideoRecording(self):
        self.bat.instr_run_class('CameraTest#testRecordeVideoUsingAllCameras')
