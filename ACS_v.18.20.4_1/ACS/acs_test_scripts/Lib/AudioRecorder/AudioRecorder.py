# @PydevCodeAnalysisIgnore
# pylint: disable=W0201,W0212,W0602,W0603,W0613
#
#
#   Windows Audio Recorder
#
#   Version : 2.0.0
#   Author  : John Popplewell
#   Email   : john@johnnypops.demon.co.uk
#   Website : http://www.johnnypops.demon.co.uk/
#
#
#   Licence:
#       The authors hereby grant permission to use, copy, modify, distribute,
#       and license this software and its documentation for any purpose,
#       provided that existing copyright notices are retained in all copies
#       and that this notice is included verbatim in any distributions. No
#       written agreement, license, or royalty fee is required for any of the
#       authorized uses.
#
#       IN NO EVENT SHALL THE AUTHORS OR DISTRIBUTORS BE LIABLE TO ANY PARTY
#       FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
#       ARISING OUT OF THE USE OF THIS SOFTWARE, ITS DOCUMENTATION, OR ANY
#       DERIVATIVES THEREOF, EVEN IF THE AUTHOR(S) HAVE BEEN ADVISED OF THE
#       POSSIBILITY OF SUCH DAMAGE.
#
#       THE AUTHOR(S) AND DISTRIBUTORS SPECIFICALLY DISCLAIM ANY WARRANTIES,
#       INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#       MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
#       NON-INFRINGEMENT.  THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, AND
#       THE AUTHOR(S) AND DISTRIBUTORS HAVE NO OBLIGATION TO PROVIDE
#       MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
#


import time
import wave
import types
import threading


from ctypes import windll, addressof, sizeof, byref, cast, create_string_buffer, Structure, \
    c_char, c_char_p, c_int, c_long, c_uint, c_ulong, c_ushort, c_void_p, \
    WINFUNCTYPE
from ErrorHandling.TestEquipmentException import TestEquipmentException

user32 = windll.user32
kernel32 = windll.kernel32
gdi32 = windll.gdi32
winmm = windll.winmm

#

WS_OVERLAPPEDWINDOW = 0xCF0000
CW_USEDEFAULT = 0x80000000L

IDI_APPLICATION = 32512
IDC_ARROW = 32512

WAVE_MAPPER = -1

CALLBACK_WINDOW = 0x00010000  # dwCallback is a HWND

WAVE_FORMAT_PCM = 1

MAXERRORLENGTH = 256

PM_REMOVE = 0x0001
WM_QUIT = 0x0012


class _Msg(Structure):
    _fields_ = [
        ('hwnd', c_ulong),
        ('message', c_ulong),
        ('wParam', c_ulong),
        ('lParam', c_ulong),
        ('time', c_ulong),
        ('x', c_ulong),
        ('y', c_ulong),
    ]

WNDPROC = WINFUNCTYPE(c_int, c_ulong, c_ulong, c_ulong, c_ulong)


class _WndClassEx(Structure):
    _fields_ = [
        ('size', c_uint),
        ('style', c_uint),
        ('wndproc', WNDPROC),
        ('cls_extra', c_int),
        ('wnd_extra', c_int),
        ('instance', c_ulong),
        ('icon', c_ulong),
        ('cursor', c_ulong),
        ('background', c_ulong),
        ('menu_name', c_char_p),
        ('class_name', c_char_p),
        ('icon_sm', c_ulong),
    ]

_window_classes = {}
_window_map = {}

module_handle = kernel32.GetModuleHandleA(0)


class _WindowClass:
    atom = 0

    def __init__(self, class_name, wndproc):
        wc = _WndClassEx()
        self.class_name = class_name
        self.wndproc = WNDPROC(wndproc)
        wc.class_name = c_char_p(self.class_name)
        wc.wndproc = self.wndproc
        wc.instance = module_handle
        wc.icon = user32.LoadIconA(0, IDI_APPLICATION)
        wc.icon_sm = user32.LoadIconA(0, IDI_APPLICATION)
        wc.cursor = user32.LoadCursorA(0, IDC_ARROW)
        wc.background = gdi32.GetStockObject(1)
        wc.size = sizeof(wc)
        self.wc = wc

    def register(self):
        global _window_classes
        if self.class_name not in _window_classes:
            self.atom = user32.RegisterClassExA(byref(self.wc))
            if self.atom:
                _window_classes[self.class_name] = self
            return self
        raise TestEquipmentException(
            TestEquipmentException.PROHIBITIVE_BEHAVIOR,
            "windows class already registered :" + str(self.class_name))

    def unregister(self):
        global _window_classes
        if self.atom:
            if user32.UnregisterClassA(self.wc.class_name, module_handle):
                self.atom = 0
                if _window_classes and self.class_name in _window_classes:
                    del _window_classes[self.class_name]
            return
        raise TestEquipmentException(
            TestEquipmentException.PROHIBITIVE_BEHAVIOR,
            "Attempting to unregister a windows class not registered")

    def __int__(self):
        if self.atom:
            return self.atom
        raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                               "Not registered!")

    def __del__(self):
        if self.atom:
            self.unregister()


def _class_wndproc(hwnd, message, wparam, lparam):
    try:
        if hwnd in _window_map:
            retval = _window_map[hwnd].wndproc(hwnd, message, wparam, lparam)
            if retval is None:
                return 0
            return retval
        return user32.DefWindowProcA(hwnd, message, wparam, lparam)
    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        return user32.DefWindowProcA(hwnd, message, wparam, lparam)

_default_python_window_class = None


def _get_default_python_window_class():
    global _default_python_window_class
    if _default_python_window_class is None:
        c = _WindowClass('default python window class', _class_wndproc)
        c.register()
        _default_python_window_class = c
        return c
    return _default_python_window_class

_messages = {
    2: ['WM_DESTROY'],
    955: ['MM_WOM_OPEN'],
    956: ['MM_WOM_CLOSE'],
    957: ['MM_WOM_DONE'],
    958: ['MM_WIM_OPEN'],
    959: ['MM_WIM_CLOSE'],
    960: ['MM_WIM_DATA'],
}


class _Window:
    hwnd = 0
    style = WS_OVERLAPPEDWINDOW
    parent = 0
    menu = 0
    instance = 0
    param = 0
    ext_style = 0
    x = y = w = h = CW_USEDEFAULT

    def __init__(self, wname, wclass=None):
        self.window_name = wname
        self.window_class = wclass
        self.instance = module_handle

    def post_quit_message(self):
        return user32.PostQuitMessage(0)

    def destroy_window(self):
        return user32.DestroyWindow(self.hwnd)

    def create(self):
        if self.hwnd:
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "Window already created.")

        if self.window_class is None:
            self.window_class = _get_default_python_window_class()

        if isinstance(self.window_class, _WindowClass):
            _class = c_char_p(self.window_class.class_name)
        else:
            _class = c_char_p(self.window_class)

        if isinstance(self.window_name, types.StringType):
            _name = c_char_p(self.window_name)
        else:
            _name = c_char_p(None)

        self.hwnd = user32.CreateWindowExA(
            self.ext_style, _class, _name,
            self.style, self.x, self.y, self.w, self.h,
            self.parent, self.menu, self.instance, self.param
        )
        global _window_map
        if self.hwnd:
            _window_map[self.hwnd] = self
            return self
        raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                               "Window can't be created")

    def wndproc(self, hwnd, message, wparam, lparam):
        handled = 0
        if message in _messages:
            names = _messages[message]
            for name in names:
                if hasattr(self, name):
                    handled = getattr(self, name)(wparam, lparam)
        if not handled:
            return user32.DefWindowProcA(hwnd, message, wparam, lparam)
        return handled

    def WM_DESTROY(self, wparam, lparam):
        global _window_map
        del _window_map[self.hwnd]
        self.hwnd = None
        if not len(_window_map):
            self.post_quit_message()

#


class _WaveInCaps(Structure):
    _fields_ = [
        ('Mid', c_ushort),
        ('Pid', c_ushort),
        ('DriverVersion', c_long),
        ('Pname', c_char * 32),
        ('Formats', c_ulong),
        ('Channels', c_ushort),
        ('Reserved', c_ushort),
    ]


class _WaveFormatEx(Structure):
    _fields_ = [
        ('FormatTag', c_ushort),
        ('Channels', c_ushort),
        ('SamplesPerSec', c_ulong),
        ('AvgBytesPerSec', c_ulong),
        ('BlockAlign', c_ushort),
        ('BitsPerSample', c_ushort),
        ('cbSize', c_ushort),
    ]

    def set_format(self, samplesize, samplerate, channels):
        self.FormatTag = WAVE_FORMAT_PCM
        self.Channels = channels
        self.BitsPerSample = samplesize
        self.BlockAlign = channels * samplesize / 8
        self.SamplesPerSec = samplerate
        self.AvgBytesPerSec = samplerate * self.BlockAlign
        self.cbSize = 0


class _AudioBuffer(Structure):

    """ Wraps up a WAVEHDR and audio data."""
    _fields_ = [
        ('lpData', c_void_p),
        ('BufferLength', c_ulong),
        ('BytesRecorded', c_ulong),
        ('User', c_ulong),
        ('Flags', c_ulong),
        ('Loops', c_ulong),
        ('lpNext', c_ulong),
        ('reserved', c_ulong),
    ]

    def __init__(self, hWave, block_len):
        Structure.__init__(self)
        self.handle = hWave
        self.membuf = create_string_buffer(block_len)
        self.lpData = cast(self.membuf, c_void_p)
        self.BufferLength = block_len
        self.Flags = 0
        self.BytesRecorded = 0
        self.User = 0
        self.Loops = 0
        self.lpNext = 0
        self.reserved = 0

    def inPrepare(self):
        return winmm.waveInPrepareHeader(self.handle, byref(self), sizeof(self))

    def inAdd(self):
        return winmm.waveInAddBuffer(self.handle, byref(self), sizeof(self))

    def inUnprepare(self):
        return winmm.waveInUnprepareHeader(self.handle, byref(self), sizeof(self))

    def __len__(self):
        return self.BytesRecorded

    def read(self):
        return self.membuf[:self.BytesRecorded]


def get_error_text(errcode):
    """ Utility function for error messages """
    msg = create_string_buffer(MAXERRORLENGTH)
    res = winmm.waveInGetErrorTextA(errcode, byref(msg), MAXERRORLENGTH)
    if res != 0:
        raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                               "Unable to retrieve recorder error message")
    return msg.value


def get__WaveInCaps(dev=WAVE_MAPPER):
    """ Utility function for _WaveInCaps """
    wicaps = _WaveInCaps()
    res = winmm.waveInGetDevCapsA(dev, byref(wicaps), sizeof(wicaps))
    if res != 0:
        raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                               "%s while opening device." % (get_error_text(res)))
    return wicaps


class _cbWindow(_Window):

    """ Callback Window for buffer events """

    def __init__(self, window_name, cbTarget):
        _Window.__init__(self, window_name)
        self.cb = cbTarget

    def WM_DESTROY(self, wparam, lparam):
        self.post_quit_message()

    def MM_WIM_OPEN(self, wparam, lparam):
        self.cb._cb_open(wparam)

    def MM_WIM_DATA(self, wparam, lparam):
        self.cb._cb_data(wparam, lparam)

    def MM_WIM_CLOSE(self, wparam, lparam):
        self.cb._cb_close(wparam)


class Recorder:

    """ Recorder class, used to record audio in wav format. """

    NUM_BUFFERS = 4
    BUFFER_SIZE = 32  # in Kb

    INITIALIZING, STOPPED, STARTING, RECORDING, STOPPING = range(5)

    def __init__(self, samplesize="16-bits", samplerate=44100, channel="Stereo"):
        """
        Recorder constructor.
        Defaults setted to 16-bit, 44100hz, Stereo

        :type samplesize: String
        :param samplesize: possible values are:
            - C{8-bits}
            - C{16-bits}

        :type samplerate: Integer
        :param samplerate: Sample rate in Hz. Possible values are:
            - C{11025}
            - C{22050}
            - C{44100}

        :type channel: String
        :param channel: possible values are:
            - C{Mono}
            - C{Stereo}
        """
        try:
            samplerate = int(samplerate)
            if samplerate not in [11025, 22050, 44100, 8000, 16000]:
                raise Exception("bad sample rate")
                # will be catched and reformatted as an TestEquipmentException
            samplesize = str(samplesize).lower()
            if samplesize == "8-bits":
                samplesize = 8
            elif samplesize == "16-bits":
                samplesize = 16
            else:
                raise Exception("bad sample size")
                # will be catched and reformatted as an TestEquipmentException

            channel = str(channel).lower()
            channels = None
            if channel == "mono":
                channels = 1
            elif channel == "stereo":
                channels = 2
            else:
                raise Exception("bad channel")
                # will be catched and reformatted as an TestEquipmentException
        except Exception as e:  # pylint: disable=W0703
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "Recorder initialisation failed because of " + str(e.args[0]))

        self.status = Recorder.INITIALIZING
        self.maxDevices = winmm.waveInGetNumDevs()
        if self.maxDevices == 0:
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "No audio recording device available.")
        self.status = Recorder.STOPPED
        self.wfx = _WaveFormatEx()
        self._set_format(samplesize, samplerate, channels)

    def _set_format(self, samplesize, samplerate, channels):
        """ Sets the recording format """
        if self.status != Recorder.STOPPED:
            return
        self.wfx.set_format(samplesize, samplerate, channels)

    def start(self, filename):
        """ Starts recording - triggers STARTING, RECORDING sequence """
        if self.status != Recorder.STOPPED:
            return
        self.status = Recorder.STARTING
        self.inside = 0

        self.w = _cbWindow('Recording-Callback-Window', self).create()

        self.hWaveIn = c_long()
        res = winmm.waveInOpen(byref(self.hWaveIn), WAVE_MAPPER, byref(self.wfx), self.w.hwnd, 0, CALLBACK_WINDOW)
        if res != 0:
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "%s while opening device." % get_error_text(res))
        self.whdr = {}

        for _i in range(Recorder.NUM_BUFFERS):
            buff = _AudioBuffer(self.hWaveIn, Recorder.BUFFER_SIZE * 1024)
            buff.inPrepare()
            buff.inAdd()
            self.whdr[addressof(buff)] = buff

        self.ofp = wave.open(filename, 'wb')
        self.ofp.setparams((self.wfx.Channels, self.wfx.BitsPerSample / 8, self.wfx.SamplesPerSec, 0, 'NONE', ''))

        res = winmm.waveInStart(self.hWaveIn)
        if res != 0:
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "%s while starting recording." % get_error_text(res))
        self.status = Recorder.RECORDING

    def stop(self):
        """ Stops recording - triggers STOPPING, STOPPED sequence """
        if self.status != Recorder.RECORDING:
            return
        self.status = Recorder.STOPPING
        res = winmm.waveInReset(self.hWaveIn)
        if res != 0:
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                   "%s while reseting device." % get_error_text(res))

    def _poll(self):
        """ Must be repeatedly called while recording. A windows message loop """
        msg = _Msg()
        while user32.PeekMessageA(byref(msg), 0, 0, 0, PM_REMOVE):
            if msg.message == WM_QUIT:
                return 0
            user32.TranslateMessage(byref(msg))
            user32.DispatchMessageA(byref(msg))
        return 1

    def wait(self, delay):
        """ Utility for recording a timed chunk of audio """
        end_time = time.time() + delay
        while True:
            try:
                if not self._poll():
                    break
                curr_time = time.time()
                if curr_time > end_time:
                    self.stop()
                time.sleep(1)
            except KeyboardInterrupt:
                self.stop()

    def _cb_open(self, devID):
        """ Callback from the message window as device is opened """
        pass

    def _cb_data(self, devID, whdr_address):
        """ Callback from the message window for each data block """
        try:
            buff = self.whdr[whdr_address]
            if len(buff) != 0:
                self.ofp.writeframesraw(buff.read())
            if self.status == Recorder.STOPPING:
                buff.inUnprepare()
                del self.whdr[whdr_address]
                if len(self.whdr) == 0:
                    self.ofp.close()
                    winmm.waveInClose(self.hWaveIn)
                    self.status = Recorder.STOPPED
            elif self.status == Recorder.RECORDING:
                buff.inAdd()
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            pass

    def _cb_close(self, devID):
        """ Callback from the message window as device closed.
            Destroys the message window """
        try:
            self.w.destroy_window()
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            pass


class ThreadedRecorder(object):

    """
    A prototype of C{Thread} usage for UE Command implementation.
    This class is intended for I{SMS} reception.
    """

    def __init__(self, samplesize="16-bits", samplerate=44100, channel="Stereo"):
        """
        Recorder constructor.
        Defaults setted to 16-bit, 44100hz, Stereo

        :type samplesize: String
        :param samplesize: possible values are:
            - C{8-bits}
            - C{16-bits}

        :type samplerate: Integer
        :param samplerate: Sample rate in Hz. Possible values are:
            - C{11025}
            - C{22050}
            - C{44100}

        :type channel: String
        :param channel: possible values are:
            - C{Mono}
            - C{Stereo}
        """
        self._my_thread = None
        self._recorder = None
        self.__filename = None
        self.__samplesize = samplesize
        self.__samplerate = samplerate
        self.__channel = channel

    def _do_run(self):
        """
        Actually does the job, this C{Thread} is intended to.
        """
        self._recorder = Recorder(self.__samplesize,
                                  self.__samplerate,
                                  self.__channel)
        self._recorder.start(self.__filename)
        while self._recorder._poll():
            # thread must not take all time in the scheduler!
            time.sleep(0.1)

    def start(self, filename):
        """
        Start recording in sub-process.
        """
        self.__filename = filename
        self._my_thread = threading.Thread(None, self._do_run)
        self._my_thread.start()

    def stop(self):
        """
        Stop recording.
        """
        self._recorder.stop()
