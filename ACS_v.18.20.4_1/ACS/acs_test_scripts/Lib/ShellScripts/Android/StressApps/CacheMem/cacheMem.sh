#!/system/bin/sh

dir=`dirname $0`
rm -r $dir/*.log 2> /dev/null

echo "Kill all process from previous process" | tee -a $dir/logFileList.log
$dir/killcacheMem.sh 2> /dev/null	# To kill all processes from previous cacheMem.sh

echo -e "\n"

#Set the test duration time
if [ -z "$1" ]; then 
	echo "	Usage: ./cacheMem [Duration in minutes]. $(date)" | tee -a $dir/fail.log
	exit 1
else
	duration=$1
	echo "Test will run for $1 minute(s)" | tee -a $dir/logFileList.log
fi

echo -e "\n"

#setup:
	echo "Please wait while cleaning & extracting the files" | tee -a $dir/logFileList.log
	rm -r [0-9] $dir/results/* $dir/linu* $dir/parallel* $dir/memory_diff* 2>/dev/null
	wait
	sync

#Start the cachebench and memory test.
	echo "Test starts : $(date)" | tee -a $dir/logFileList.log
	$dir/memorytest.sh $duration $2 | tee -a $dir/results/memorytestlog-$(date -u +%F_%T)&

	$dir/run_em.sh $duration
	echo -e "\n\n" | tee -a $dir/logFileList.log

#End of cachebench and memory test.
	echo "done!" | tee -a $dir/logFileList.log
	echo "cacheMem stops : $(date)" | tee -a $dir/logFileList.log

	mv $dir/logFileList.log $dir/pass.log
	exit
