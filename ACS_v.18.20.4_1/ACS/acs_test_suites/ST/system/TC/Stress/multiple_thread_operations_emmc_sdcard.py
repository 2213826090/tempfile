import os
import sys
import time
import threading

start_time = "2012-01-01/00:00:00"

def log(msg):
	curTime = time.strftime("%Y-%m-%d/%H:%M:%S ")
	os.system("echo " + curTime + msg + " >> " + logfile)
	
def checkCrash():
	global start_time
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

	lineList = lineList[2:]
	i = 0
	for line in lineList:
		event_dict[i] = dict()
		wordList = line.split()
		event_dict[i]["EVENT"] = wordList[0]
		event_dict[i]["ID"] = wordList[1]
		event_dict[i]["DATE"] = wordList[2]
		event_dict[i]["TYPE"] = " ".join(wordList[3:])
		i = i + 1
	
	for j in range(len(event_dict)):
		if event_dict[j]["EVENT"] == "CRASH":
			if start_time == None:
				log("Wrong start time")
				return False
			elif cmp(start_time, event_dict[j]["DATE"]) == -1:
				log("Crash happened")
				return False
			else:
				pass
		else:
			pass
	return True

def operate_emmc_sdcard(a_tid):
	global g_mutex
	print "Thread %s is running..." %a_tid
	log("Thread %s is running..." %a_tid)
	g_mutex.acquire()
	#create file
	log("create file in emmc...")
	os.system("adb shell \"touch /data/thread_%s.txt\"" %a_tid)	
	time.sleep(1)
	#write file
	log("write file in emmc...")
	os.system("adb shell \"ls -l /data/ > /data/thread_%s.txt\"" %a_tid)
	time.sleep(1)
	#copy file
	log("copy file from emmc to sdcard...")
	os.system("adb shell \"cp /data/thread_%s.txt /mnt/sdcard_ext/\"" %a_tid)
	time.sleep(1)
	#remove file
	log("remove file in emmc and sdcard...")
	os.system("adb shell \"rm /data/thread_%s.txt\"" %a_tid)
	os.system("adb shell \"rm /mnt/sdcard_ext/thread_%s.txt\"" %a_tid)	
	time.sleep(1)
	print "Thread %s ended..." %a_tid
	log("Thread %s ended..." %a_tid)
	g_mutex.release()

if __name__ == "__main__":
	if os.path.exists(r'multiple_thread_operations_emmc_sdcard.log'):
		os.remove(r'multiple_thread_operations_emmc_sdcard.log')
	logfile = os.getcwd() + "/multiple_thread_operations_emmc_sdcard.log"

	os.system("adb root")
	time.sleep(2)
	os.system("adb remount")
	time.sleep(2)
	
	global g_mutex
	g_mutex = threading.Lock()
	thread_pool = []
	for i in range(50):
		th = threading.Thread(target = operate_emmc_sdcard, args = (i, ))
		thread_pool.append(th)
		
	start_time = time.strftime("%Y-%m-%d/%H:%M:%S ")
	
	for i in range(50):
		thread_pool[i].start()
	
	for i in range(50):
		threading.Thread.join(thread_pool[i])
  			
	if checkCrash():
		print "multiple thread operations emmc sdcard passed"
	else:
		print "multiple thread operations emmc sdcard failed"
