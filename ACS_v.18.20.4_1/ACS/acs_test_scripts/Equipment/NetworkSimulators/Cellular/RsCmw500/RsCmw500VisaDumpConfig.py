#!/usr/bin/env python
import visa
rm = visa.ResourceManager()
rm.list_resources()
toto = rm.get_instrument("TCPIP::192.168.48.126::INST0::INSTR")

cmds = [
    "*IDN?",

    # LTE_COMMON
    "SOURce:LTE:SIGN:CELL:STATe?",
    "SENSe:LTE:SIGN:RRCState?",
    "CONFigure:LTE:SIGN:ETOE?",
    "CONFigure:LTE:SIGN:BAND?",
    "CONFigure:LTE:SIGN:CELL:BAND:DL?",
    "CONFigure:LTE:SIGN:CONNection:UDCHannels:DL?",
    "CONFigure:LTE:SIGN:CONNection:UDCHannels:UL?",
    "CONFigure:LTE:SIGN:CONNection:STYPE?",

    "CONFigure:LTE:SIGN:CELL:SEC:IALG?",
    "CONFigure:LTE:SIGN:CONN:AMDB?",

    "CONFigure:LTE:SIGN:CONNection:DPCYcle?",
    "CONFigure:LTE:SIGN:DL:RSEPre:LEVel?",
    "CONFigure:LTE:SIGN:UL:PUSCh:OLNPower?",
    "CONFigure:LTE:SIGN:UL:PUSCh:TPC:CLTPower?",
    "CONFigure:LTE:SIGN:CONNection:SCHModel:ENABle?",
    "CONFigure:LTE:SIGN:CONNection:PDCCh:RPDCch?",

    # CAT 4
    "CONFigure:LTE:SIGN:CELL:BAND:DL?",
    "CONFigure:LTE:SIGN:CONNection:UDCHannels:DL?",
    "CONFigure:LTE:SIGN:CONNection:UDCHannels:UL?",


    # US
    'CONFigure:LTE:SIGN:BAND?',
    'CONFigure:LTE:SIGN:RFSettings:CHANnel:UL?',
    'CONFigure:LTE:SIGN:CONNection:OBCHange?',
    'CONFigure:LTE:SIGN:CONNection:FCHange?',
    'CONFigure:LTE:SIGN:DMODe?'

      ]

for cur_cmd in cmds:
    print(cur_cmd)
    try:
        print (toto.ask(cur_cmd))
    except:
        print ("*** ERROR ***")
toto.clear()
toto.close()
rm.close()

print "Done"