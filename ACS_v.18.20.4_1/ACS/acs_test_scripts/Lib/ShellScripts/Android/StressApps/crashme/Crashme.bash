#!/system/bin/sh

dir=`dirname $0`

rm -r $dir/*.log 2> /dev/null

if [ -z $1 ]; then
	echo "Usage: ./Crashme.bash <duration in minutes>" | tee -a $dir/fail.log
	echo "Eg: ./Crashme 4320"
	exit 1
else 
	echo "Test duration is set to $1 minutes" | tee -a $dir/crashme.log
	test_time=$(($1*60))
fi

tested_time=0
starting=$(date -u +%s)

echo "Start time : $(date)" | tee -a $dir/crashme.log
# Discard the output b/c its just a bunch of "Crashme" repeated and slows down execution
echo "Executing: $dir/Crashme 255 >/dev/null &" | tee -a $dir/crashme.log
($dir/Crashme $test_time  &)

while [ $tested_time -lt $test_time ]
do
	if [ $? -ne 0 ]; then
		echo "Non-zero return code detected!!" | tee -a $dir/crashme.log
		echo "Current Time : $(date)" | tee -a $dir/crashme.log
		echo "CrashMe Test is failed!! Exiting..." | tee -a $dir/crashme.log
		mv $dir/crashme.log $dir/fail.log
		killall Crashme
		exit -1
	fi
	
	ending=$(date -u +%s)
	tested_time=$(($ending-$starting))
    
done

echo "Killing all instances of Crashme." | tee -a $dir/crashme.log
echo "ps | grep Crashme" | tee -a $dir/crashme.log
echo `ps | grep Crashme` | tee -a $dir/crashme.log
Pid=`ps | grep Crashme | awk '{print $2}'`
echo "Process ID $Pid" | tee -a $dir/crashme.log
echo "kill -9 $Pid" | tee -a $dir/crashme.log
kill -9 $Pid
 
for i in $Pid
do
	echo "kill -9 $i" | tee -a $dir/crashme.log
	kill -9 $i
done

echo "CrashMe test is passing @ $(date)" | tee -a $dir/crashme.log
mv $dir/crashme.log $dir/pass.log
