import os
import sys
import time
import subprocess

def adb_push_pull():
	transferCmd = "adb "
	for i in range (1, len(sys.argv)-1):
		transferCmd = transferCmd + " " + sys.argv[i]
	referenceSpeed = sys.argv[4]
	p = subprocess.Popen(transferCmd, shell = True, stdout = subprocess.PIPE, stderr = subprocess.STDOUT)
	retval = p.wait()
	speedList = p.stdout.readlines()
	speedString = speedList[0].split()
	speed = speedString[0]
	print "adb %s speed: %s" %(sys.argv[1], speed)
	if speed >= referenceSpeed:
		print "transfer speed check passed"
	else:
		print "transfer speed check failed"
	
if __name__ == '__main__':
	adb_push_pull()