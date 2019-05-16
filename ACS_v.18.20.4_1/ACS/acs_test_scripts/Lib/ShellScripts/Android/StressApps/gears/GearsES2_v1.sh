#!/system/bin/sh

dir=`dirname $0`
package="com.jeffboody.GearsES2eclair"
package_apk="GearsES2_20110906.apk"
activity="GearsES2eclair"
logfile="gearsES2.log"
verify_time=3600
rm -r $dir/*.log 2> /dev/null

if [ -z "$1" ]; then
	echo "	GearsES2: Missing time duration(minute) as first argument!" | tee -a $dir/fail.log
	exit 1
else
	echo "	GearsES2: Test will run for $1 min. $(date)" | tee -a $dir/$logfile
	test_time=$(($1*60))
fi

if [ -z "`pm list packages | grep $package`" ]; then
	echo "	GearsES2: App not found. Installing app now" | tee -a $dir/$logfile
	pm install $dir/$package_apk 2> /dev/null
	sleep 2
fi

# Start to run the test by am script to call the application.

am start -S -a android.intent.action.MAIN -n $package/.$activity
input keyevent 66
input keyevent 66
input keyevent 66

start_time=$(date -u +%s)
end_time=$(($start_time+$test_time))
sleep 1
while [ "$current_time" -lt "$end_time" ]; do
	Pid=`ps | grep $package | awk '{print $2}'`
	current_time=$(date -u +%s)

	if [ -z "$Pid" ]; then
		echo "	ERROR: The $package has quit from the test! $(date)" | tee -a $dir/fail.log
		exit 1
	fi

	time_check=$(($current_time-$start_time))
	if [ "$(($time_check%$verify_time))" -eq 0 ]; then
		echo " 	GearsES2: I am still alive after $time_check seconds" | tee -a $dir/$logfile
	sleep 2
	fi
done

am force-stop $package
echo "	Test $0 ends at $(date)" | tee -a $dir/$logfile
cp $dir/$logfile $dir/pass.log
