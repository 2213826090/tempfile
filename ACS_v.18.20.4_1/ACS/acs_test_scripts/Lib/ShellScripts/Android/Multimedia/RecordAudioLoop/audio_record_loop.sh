#!/system/bin/sh

dir=`dirname $0`
package="com.example.cvaudiorecording"
package_apk="CVAudioRecording.apk"
activity="RecordActivity"
verify_iteration="20"		# to keep 1 copy for verification in each 20 records
media_dest="/data/CVaudio_recording_verification"
logfile="audio_recording_loop.log"
rm -r $dir/*.log $media_dest 2> /dev/null

function start_stop_recording
{
	input keyevent 21
	input keyevent 66
	sleep 60
	input keyevent 66
	sleep 3
}

if [ -z "$1" ]; then
	echo "	Audio Record: Missing time duration(minute) as first argument!" | tee -a $dir/fail.log
	exit 1
else
	echo "	Audio Record: Test will run for $1 min. $(date)" | tee -a $dir/$logfile
	test_time=$(($1*60))
fi

if [ -z "`pm list packages | grep $package`" ]; then
	echo "	Audio Record: App not found. Installing app now" | tee -a $dir/$logfile
	pm install $dir/$package_apk 2> /dev/null
	sleep 2
fi

mkdir $media_dest 2> /dev/null

# Start to run the test by am script to call the application.

am start -S -a android.intent.action.MAIN -n $package/.$activity

start_time=$(date -u +%s)
end_time=$(($start_time+$test_time))
sleep 1
i=0
while [ "$current_time" -lt "$end_time" ]; do
	Pid=`ps | grep $package | awk '{print $2}'`
	current_time=$(date -u +%s)
	
	if [ -z "$Pid" ]; then 
		echo "	ERROR: The $package has quit from the test! $(date)" | tee -a $dir/fail.log
		exit 1
	fi

	start_stop_recording
	
	if [ "$(($i%$verify_iteration))" -eq 0 ]; then
		echo " 	Audio Record: Copy the most recent record #$i to directory $media_dest, for verification purposes if needed" | tee -a $dir/$logfile
		Record=`ls /storage/sdcard0/Music/MyVoiceRecording/ | tail -n 1`
		cp /storage/sdcard0/Music/MyVoiceRecording/$Record $media_dest
		echo "	Audio Record: Remove all previous records"
		rm -r /storage/sdcard0/Music/MyVoiceRecording/*
	fi
	i=$(($i+1))
done

am force-stop $package
echo "	Test $0 ends at $(date)" | tee -a $dir/$logfile
cp $dir/$logfile $dir/pass.log
