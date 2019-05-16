#!/system/bin/sh
#shell script to change the screen orientation in Android
#the script takes three inputs, 1. total duration in minutes 2. rotation mode 3. min delay, and 4. max delay in seconds) between each rotation.
EXE_DIR=`dirname $0`
LOG_FILE=logfile.txt

rm -r $EXE_DIR/*.log $EXE_DIR/*.txt  2> /dev/null

if [ $# -lt 4 ]
then
	echo "ERROR: $0 wrong usage, please provide 4 inputs" >> $EXE_DIR/$LOG_FILE
	echo "ERROR: $0 #duration #rotation_mode #min_delay(S) #max_delay(S)" >> $EXE_DIR/$LOG_FILE
	mv $EXE_DIR/$LOG_FILE $EXE_DIR/fail.log
	exit 1
fi

#enable rotations through user commands
i=0
duration_minutes=$1
duration_seconds=`expr $duration_minutes \* 60`
base_time=`date +%s`
current_time=`date +%s`
end_time=`expr $base_time + $duration_seconds`
prev_orientation=0

echo "Current time in Seconds since beginning of the Time is $base_time" >> $EXE_DIR/$LOG_FILE
echo "Test will complete when world moves past $end_time" >> $EXE_DIR/$LOG_FILE

if [ $2 -eq 1 ]
then
	while [ $current_time -le $end_time ]
	do
		i=$(($RANDOM%4))
		if [ $i -ne $prev_orientation ]
		then
			echo "Screen will be rotated to "$i >> $EXE_DIR/$LOG_FILE
			content insert --uri content://settings/system --bind name:s:user_rotation --bind value:i:$i
			sleep_interval_seconds=`expr $(($RANDOM % ($4 - $3 + 1))) + $3`
			sleep $sleep_interval_seconds
			current_time=`date +%s`
			prev_orientation=$i
		fi
	done
else
	while [ $current_time -le $end_time ]
	do
		i=$(($((i+1))%4))
		echo "Screen will be rotated to "$i >> $EXE_DIR/$LOG_FILE
		content insert --uri content://settings/system --bind name:s:user_rotation --bind value:i:$i
		sleep_interval_seconds=`expr $((RANDOM % ($4 - $3 + 1))) + $3`
		sleep $sleep_interval_seconds
		current_time=`date +%s`
	done
fi
mv $EXE_DIR/$LOG_FILE $EXE_DIR/pass.log
exit 0
