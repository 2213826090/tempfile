#!/usr/bin/env python
import os
import sys
import serial
import argparse
import time
import pdb

mode={'shutdown':'r', 'elk':'n4#', 'fastboot':'n2#', 'adb': 'g'}
def get_argument():
	parser = argparse.ArgumentParser(description='get argument about debug card')
	parser.add_argument('--port', required=True, help='please input the third port of your debug card')
	parser.add_argument('--mode', choices=['shutdown','elk','fastboot','adb'], required=True, help='input the mode you want boot to')
	args = parser.parse_args()
	return args

def writeCmd(port,cmd):
	print 'port=%s,cmd=%s' % (port,cmd)
	try:
	#	pdb.set_trace()
		ser = serial.Serial(port=port, baudrate=115200, timeout=2)
		returnV = ser.write(cmd)
		print returnV
	except Exception as e:
		print "write data to debug card error"
def bootTo(port,mod):
	if mod in mode.keys():
		if mod == "shutdown":
			writeCmd(port,mode[mod])
		else:
			writeCmd(port,mode['shutdown'])
			time.sleep(1)
			writeCmd(port,mode[mod])

def main():
	args = get_argument()
	port = args.port
	mod = args.mode
	print port,mod
	bootTo(port,mod)

if __name__ == "__main__":
	print "start main"
	main()