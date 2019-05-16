#!/system/bin/sh

dir=`dirname $0`

if [ $# -lt 2 ]
then
	user_time=2880
	cpu=0
	echo "Test time is defaulted to 48 hours."
	echo "Binding the test to CPU$cpu"
else
	user_time=$1
	cpu=$2
fi

echo "Test time will run for $user_time minute(s)".
echo "Binding the test to CPU$cpu"

#total_sec=`expr $user_time \* 3600`
total_sec=`expr $user_time \* 60`
current_time=`date +%s`
end_time=`expr $current_time + $total_sec`
time_left=`expr $end_time - $current_time`
loop=0

while [ $current_time -lt $end_time ]
do
	current_time=`date +%s`
	time_left=`expr $end_time - $current_time`
	time_left=`expr $time_left / 60`
	echo "Minutes Left $time_left" 

	$dir/pi_ca $cpu
	diff pi$cpu.dat $dir/pi_golden.dat > $dir/checkfile$cpu
	if [ -s "$dir/checkfile$cpu" ]
	then
		echo "pi: Test Failed" | tee -a $dir/fail.log
		touch PI_FFT_FAILED
		exit 1
	else
		echo "Test Passed."
	fi

	loop=`expr $loop + 1`
	echo "Completed $loop loop."
	echo
done
