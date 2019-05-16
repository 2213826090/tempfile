#!/usr/bin/env python
import visa
import time
rm = visa.ResourceManager()
rm.list_resources()
toto = rm.get_instrument("TCPIP::192.168.48.3::INST0::INSTR")

cmds = [

'*RST'

      ]

toto.timeout = 20000


for cur_cmd in cmds:
    print toto.write("*CLS")
    print("-> " + cur_cmd)
    result = toto.write(cur_cmd)
    print(result)
    print "Status = " + toto.ask("SYST:ERR?")
toto.clear()
toto.close()
rm.close()
print "Done"
