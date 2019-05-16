from testaid.headset import Headset
from testlib.audio.helper import get_hs_devnode
from testlib.audio.bt_audio import BTAudioAdapter


def use_hs(func):
    '''
    case that use headset
    '''
    def wrapper(self, *args, **kwargs):
        devnode = get_hs_devnode()
        logger = self.logger if hasattr(self, 'logger') else None
        self.hs = Headset(devnode, logger)
        try:
            func(self, *args, **kwargs)
        finally:
            self.hs.reset()
    return wrapper


def use_bt(func):
    '''
    case that use BT
    '''
    def wrapper(self, *args, **kwargs):
        self.bt = BTAudioAdapter()
        self.bt.setup()
        try:
            func(self, *args, **kwargs)
        finally:
            self.bt.teardown()
    return wrapper
