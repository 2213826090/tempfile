import os
import time
import string
import threading
import unittest
import subprocess

def log(msg):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_179.log"
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

class ST_SYST_TOOLS_LOG_179(unittest.TestCase):
	def setUp(self):
		pass
	
	def tearDown(self):
		pass
			
	def ST_SYST_TOOLS_LOG_179(self):
		if (check_Connection() == False):
			log("adb disconnected, terminate and check")
			return False
		else:
			os.system("adb root")
			time.sleep(2)
			os.system("adb shell configure_trace_modem -h -t2")
			time.sleep(5)
			log("board reboot, waiting for few seconds")
			os.system("adb reboot")
			time.sleep(60)
			if (check_Connection() == True):
				os.system("adb root")
				time.sleep(2)
				os.system("adb shell activate_trace_modem -on -sd2 -p")
				time.sleep(5)
				os.system("adb pull /logs/bplog bplog_179")
				time.sleep(2)
				if os.path.getsize("bplog_179") > 0:
					log("bplog pull successfully")
					log("board reboot again, waiting for few seconds")
					os.system("adb reboot")
					time.sleep(60)
					if (check_Connection() == True):
						os.system("adb root")
						time.sleep(2)
						os.system("adb pull /logs/bplog bplog_179_new")
						time.sleep(2)
						os.system("adb shell activate_trace_modem -off")
						time.sleep(5)
						if (not (os.path.getsize("bplog_179_new") < os.path.getsize("bplog_179"))):
							log("bplog keeps increasing")
							return True
						else:
							log("bplog doesn't keep increasing")
							return False
					else:
						log("board can't reboot, terminate and check")
						return False
				else:
					log("bplog pull unsuccessfully")
					return False
			else:
				log("board can't reboot, terminate and check")
				return False	

	def test_ST_SYST_TOOLS_LOG_179(self):
		log("ST_SYST_TOOLS_LOG_179: start!")
		if (self.ST_SYST_TOOLS_LOG_179() == True):
			log("ST_SYST_TOOLS_LOG_179: pass")
			print "ST_SYST_TOOLS_LOG_179: pass"
		else:
			log("ST_SYST_TOOLS_LOG_179: fail")
			print "ST_SYST_TOOLS_LOG_179: fail"
		log("ST_SYST_TOOLS_LOG_179: end!")
	
if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_179.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_179.log')
	unittest.main()