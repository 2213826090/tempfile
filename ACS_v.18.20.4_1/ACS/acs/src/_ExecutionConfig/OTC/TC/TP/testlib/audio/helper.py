import os
import random


def dtmf_random_str(length):
    '''
    generate a random DTMF symbol string with *length* size
    '''
    keys = "0123456789*#"
    symbols = ''
    for _ in range(length):
        index = random.randint(0, len(keys) - 1)
        symbols += keys[index]
    return symbols


def get_hs_devnode():
    '''
    get Headset device node
    '''
    # find a proper device node
    for dev in os.listdir('/dev/'):
        if dev.startswith("ttyUSB"):
            devnode = os.path.join("/dev/", dev)
            return devnode
    else:
        raise Exception("Can't find dev node /dev/ttyUSB*")


def get_sound_card():
    '''
    get USB sound if have, else return default sound card
    '''
    cmd = 'aplay -l'
    p = os.popen(cmd)
    buf = p.read()
    for l in buf.split('\n'):
        if 'USB Audio' in l and l.startswith('card'):  # found
            return l.split(':')[0].replace(' ', '')
    return ''


def run_image_case(case):
    '''
    run Multimedia_Image cases

    no need to specified tests.Multimedia_Image, just give the rest
    '''
    env = 'TEST_DATA_ROOT'
    old_val = os.environ.get(env, "")
    if old_val.strip("/").endswith("Multimedia_Audio"):
        image_data_root = os.path.abspath(
            os.path.join(old_val, '..', 'Multimedia_Image'))
    else:
        image_data_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__),
                         "../.."
                         "testplan/Multimedia_Image/"))
    os.environ[env] = image_data_root
    try:
        import unittest
        loader = unittest.TestLoader()
        full_case = "tests.Multimedia_Image." + case
        suite = loader.loadTestsFromName(full_case)
        result = unittest.TestResult()
        suite.run(result)
        if not result.wasSuccessful():
            err_msg = '\n'.join(map(lambda e: e[1], result.errors))
            raise Exception(err_msg)
    finally:
        os.environ[env] = old_val
