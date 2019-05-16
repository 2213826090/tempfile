import os
import time
import string
import threading
import unittest
import subprocess

def log(msg):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_176.log"
	curTime = time.strftime("%m-%d %H:%M:%S ")
	os.system("echo " + curTime + "Infor/" + msg + " >> " + logfile)

def check_Connection():
	output = os.popen("adb shell echo hello")
	connectResponce = output.read()
	result = connectResponce.find("hello")	
	if (result != -1):
		log("adb connected")
		return True
	else:
		log("adb disconnected")
		return False

class ST_SYST_TOOLS_LOG_176(unittest.TestCase):
	def setUp(self):
		pass
	
	def tearDown(self):
		pass

	def ST_SYST_TOOLS_LOG_176(self):	
		if (check_Connection() == False):
			log("adb disconnected, terminate and check")
			return False
		else:
			os.system("adb root")
			time.sleep(2)
			os.system("adb shell configure_trace_modem -d -t1")
			time.sleep(5)
			log("board reboot, waiting for few seconds")
			os.system("adb reboot")
			time.sleep(60)
			if (check_Connection() == True):
				os.system("adb root")
				time.sleep(2)
				os.system("adb shell activate_trace_modem -on -sd1 -o")
				time.sleep(5)
				os.system("adb shell activate_trace_modem -off")
				time.sleep(5)
				os.system("adb pull /logs/bplog bplog_176")
				time.sleep(2)
				if os.path.getsize("bplog_176") > 0:
					log("bplog pull successfully")
					return True
				else:
					log("bplog pull unsuccessfully")
					return False
			else:
				log("board can't reboot, terminate and check")				
				return False
	
	def test_ST_SYST_TOOLS_LOG_176(self):
		log("ST_SYST_TOOLS_LOG_176: start!")
		if (self.ST_SYST_TOOLS_LOG_176() == True):
			log("ST_SYST_TOOLS_LOG_176: pass")
			print "ST_SYST_TOOLS_LOG_176: pass"
		else:
			log("ST_SYST_TOOLS_LOG_176: fail")
			print "ST_SYST_TOOLS_LOG_176: fail"
		log("ST_SYST_TOOLS_LOG_176: end!")

if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_176.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_176.log')
	unittest.main()