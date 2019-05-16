from threading import Thread

import vhal_consts_2_0
import vhal_emulator

vhal = 0

def rxThread(v):
    while (1):
        print v.rxMsg()

def main():
    global vhal
    vhal = vhal_emulator.Vhal(vhal_consts_2_0.vhal_types_2_0)
    rx = Thread(target=rxThread, args=(vhal,))
    rx.start()

if __name__ == '__main__':
    main()