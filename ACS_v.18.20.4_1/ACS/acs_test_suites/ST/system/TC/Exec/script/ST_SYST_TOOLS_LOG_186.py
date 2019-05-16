import os
import time
import string
import threading
import unittest
import subprocess

def log(msg):
	logfile = os.getcwd() + "/ST_SYST_TOOLS_LOG_186.log"
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

def check_HistoryEvent():
	os.system("adb root")
	time.sleep(1)
	os.system("adb remount")
	time.sleep(1)
	os.system("adb pull /logs/history_event")
	time.sleep(1)
	event_dict = dict()
	eventf = open("history_event")
	lineList = eventf.readlines()
	eventf.close()
	os.remove("history_event")
	for line in lineList:
		if (line.find("IPANIC") != -1) or (line.find("WDT") != -1):
			return True
		else:
			pass
	return False

class ST_SYST_TOOLS_LOG_186(unittest.TestCase):
	def setUp(self):
		pass
	
	def tearDown(self):
		pass
	
	def ST_SYST_TOOLS_LOG_186(self):
		if (check_Connection() == False):
			log("adb disconnected, terminate and check")
			return False
		else:
			os.system("adb root")
			time.sleep(2)
			os.system("adb shell rm -r /logs/history_event")
			time.sleep(1)
			os.system("adb shell mount -t debugfs none /d/")
			time.sleep(5)
			os.system("adb shell cat /d/watchdog/kernel_watchdog/trigger")
			time.sleep(100)
			if (check_Connection() == True):
				if (check_HistoryEvent() == True):
					log("New IPANIC and WDT appears")
					return True
				else:
					log("No new IPANIC and WDT appears")
					return False
			else:
				log("adb disconnected, terminate and check")
				return False

	def test_ST_SYST_TOOLS_LOG_186(self):
		log("ST_SYST_TOOLS_LOG_186: start!")
		if (self.ST_SYST_TOOLS_LOG_186() == True):
			log("ST_SYST_TOOLS_LOG_186: pass")
			print "ST_SYST_TOOLS_LOG_186: pass"
		else:
			log("ST_SYST_TOOLS_LOG_186: fail")
			print "ST_SYST_TOOLS_LOG_186: fail"
		log("ST_SYST_TOOLS_LOG_186: end!")

if __name__ == '__main__':
	if os.path.exists(r'ST_SYST_TOOLS_LOG_186.log'):
		os.remove(r'ST_SYST_TOOLS_LOG_186.log')
	unittest.main()