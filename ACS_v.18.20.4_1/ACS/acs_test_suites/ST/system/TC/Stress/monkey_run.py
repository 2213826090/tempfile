import os, sys, time, subprocess

def MonkeyTest():
	work_dir = os.getcwd() + os.path.sep + "_ExecutionConfig" + os.path.sep + "System" + os.path.sep + "Stress" + os.path.sep
	log_dir = work_dir + "logs" + os.path.sep + time.strftime('%Y-%m-%d_%H-%M-%S',time.localtime(time.time())) + os.path.sep
	os.makedirs(log_dir)
	os.chdir(work_dir)
	
	cmd = "adb shell monkey "
	for i in range (1, len(sys.argv)):
		cmd = cmd + " " + sys.argv[i]
	
	log_output = "output.log"
	log_adb = "logcat.log"
	verdict = True
	
	file_output = open(log_dir + log_output, 'w+')
	#file_adb = open(log_dir + log_adb, 'w')
	
	os.system("adb logcat -c")
	p = subprocess.Popen(cmd, shell = True, stdout = subprocess.PIPE, stderr = subprocess.STDOUT)
	for line in p.stdout.readlines():
		file_output.write(line)
		if line.find("crashed") >= 0:
			verdict = False
	retval = p.wait()
	
	file_output.close()
	
	os.system("adb logcat -d > %s" %(log_dir + log_adb))
	
	if verdict == False or retval != 0:
		print "Monkey test cases failed"
	else:
		print "Monkey test cases passed"
		
if __name__ == '__main__':
	MonkeyTest()
