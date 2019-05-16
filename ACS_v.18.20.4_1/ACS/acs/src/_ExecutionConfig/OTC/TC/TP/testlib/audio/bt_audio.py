import os
import time
import logging
import tempfile
from datetime import datetime
from testlib.util.common import g_common_obj
from testlib.audio.audio_log import AudioLogger
from testlib.bluetooth.bluetooth import BluetoothSetting
from testaid.bluez5.manager import BTManager
from testaid.bluez5.agent import Agent
from testaid.bluez5.sco import ScoServer
from testaid.bluez5.hsp import HeadsetProfile
# from testaid.bluez5.hfp import HandsFreeProfile
from testaid.bluez5.a2dp import SBCAudioSink, SBCAudioSource
from testaid.bluez5.process import Logger


class AudioSink(SBCAudioSink):
    waves_dir = ''
    ''' directory to save sink dump as waves '''
    waves = []
    ''' wav file list dumped, the most recent file put at the end of list '''
    _stream_active = False

    def __init__(self, *args, **kwargs):
        super(AudioSink, self).__init__(*args, **kwargs)
        self.waves_dir = tempfile.mkdtemp()
        self.waves = []

    def transport_event_handler(self, event):
        '''
        For each transport, a wave file will be created
        '''
        if event == self.EVENT_TRANS_ACQUIRE:
            self._stream_active = True
            now = datetime.now()
            fname = "%s.wav" % now.strftime("%H%H%S")
            fpath = os.path.join(self.waves_dir, fname)
            logging.info("dump sink to file: " + fpath)
            self._wavwriter = self.get_wave_writer(fpath)
            self.waves.append(fpath)
        elif event == self.EVENT_TRANS_RELEASE:
            self._stream_active = False
            self._wavwriter.close()
        elif event == self.EVENT_TRANS_IN:
            data = self.read_transport()
            self._wavwriter.writeframesraw(data)

    def is_active(self):
        '''
        check if current a2dp sink is active
        '''
        return self._stream_active

    def get_last_wav(self):
        '''
        get the most recent record wav file
        '''
        if len(self.waves) > 0:
            return self.waves[-1]
        else:
            raise Exception("A2DP Sink: No audio sample received")

    def destroy(self):
        # clean wave file dump, if you want to save wave file, copy it out
        os.system("rm -rf " + self.waves_dir)
        super(AudioSink, self).destroy()


class BTAudioAdapter(object):
    '''
    a BT Audio Adapter to represent BT Audio device
    '''
    def setup(self):
        self.d = g_common_obj.get_device()
        self.dut = BluetoothSetting()  # BluetoothSetting in DUT
        self.adapter = BTManager().default_adapter()
        self.log = AudioLogger.getLogger()
        Logger.set_enable(True)
        try:
            # register a agent, just do nothing to accept, just like headset
            self.agent = Agent.init_from_remote(capability="NoInputNoOutput")
            self.agent.register()

            # profiles
            self.headset = HeadsetProfile.init_from_remote()  # Headset Profile
            self.headset.register()
            # self.handsfree = HandsFreeProfile.init_from_remote()  # Handsfree
            # self.handsfree.register()
            self.a2dp_sink = AudioSink.init_from_remote()  # A2DP Sink
            self.a2dp_sink.register()
            self.a2dp_source = SBCAudioSource.init_from_remote()  # A2DP Source
            self.a2dp_source.register()
            self.sco = ScoServer.init_from_remote()
            self.adapter.Powered = True
            self.adapter.Discoverable = True
        except:
            self.teardown()
            raise

    def teardown(self):
        # the sequence should the same as setup
        self.agent.unregister()
        self.agent.destroy()
        self.headset.unregister()
        self.headset.destroy()
        # self.handsfree.unregister()
        # self.handsfree.destroy()
        self.a2dp_sink.unregister()
        self.a2dp_sink.destroy()
        self.a2dp_source.unregister()
        self.a2dp_source.destroy()
        self.sco.destroy()
        self.adapter.Powered = False

    def reset_adapter(self):
        '''
        reset BT adapter
        '''
        self.log.info("reset adapter")
        self.adapter.Discoverable = False
        self.adapter.Powered = False
        # clean all device found
        for dev in self.adapter.list_devices():
            try:
                self.adapter.remove_device(dev)
            except:
                pass
        os.system('rfkill block bluetooth')
        time.sleep(0.5)
        os.system('rfkill unblock bluetooth')
        time.sleep(2)
        self.adapter.Powered = True
        self.adapter.Discoverable = True
        self.adapter.DiscoverableTimeout = 0

    def connect(self, enable):
        '''
        connect/disconnect BT

        enable:
            True -- connect
            False -- disconnect
        '''
        self.adapter.Discoverable = True
        self.adapter.DiscoverableTimeout = 0
        devname = self.get_alias()
        self.dut.launch()
        self.dut.enable(True)  # make sure BT enabled
        if enable:
            try:
                self.dut.connect(devname)
            except Exception, e:
                self.log.info("first connect fails: %s : retry..." % str(e))
                # reset dut
                self.dut.remove(devname)
                self.dut.enable(False)
                # reset adapter
                self.reset_adapter()
                time.sleep(2)
                # try connect again
                self.dut.enable(True)
                self.dut.connect(devname)
        else:
            self.dut.disconnect(devname)
        self.d.press.back()
        self.d.press.home()

    def get_alias(self):
        return self.adapter.Alias

    def set_alias(self, name):
        self.adapter.Alias = name

    def start_scan(self):
        self.adapter.Powered = True
        try:
            self.adapter.start_discovery()
        except:
            pass  # ignore already in scan exception
        self.adapter.Discoverable = True

    def get_media_control(self):
        '''
        get MediaControl of connected device
        '''
        for d in self.adapter.list_devices():
            if d.Connected:
                return d.get_media_control()
        else:
            raise Exception("Can't find any connected device")


logging.basicConfig(level=logging.DEBUG)
