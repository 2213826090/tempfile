#!/system/bin/sh

tested_time=0
start=$(date)
starting=$(date -u +%s)
dir=`dirname $0`
rm -r $dir/*.log

if [ -z "$1" ]; then
	echo "Usage: ./run_all_cpu.sh <duration in minutes>" | tee -a $dir/fail.log
	exit 1
else
	test_time=$1
	echo "Test duration is manually set to $1 minute(s)" | tee -a $dir/pi.log
	time=$1
fi

num_cpus=`grep "^processor" /proc/cpuinfo | wc -l`

i=0
while [ $i -lt $num_cpus ];
do
	sh $dir/run_pi.sh $time $i &	
	i=$(($i+1))
done

echo "Start time: $start" | tee -a $dir/pi.log 
while [ $tested_time -lt $(($test_time*60)) ]
do
	if [ $? -ne 0 -o -f "PI_FFT_FAILED" ]; then
		end=$(date)
		echo "start time: $start" > $dir/fail.log
		echo "End time: $end" >> $dir/fail.log
		echo "PI_FFT is failed" >> $dir/fail.log
		if [ -f "PI_FFT_FAIL" ]; then
			echo "PI_FFT_FAILED existed!!" >> $dir/fail.log
			cat ./PI_FFT_FAILED >> $dir/fail.log
		fi
		exit 1
	fi
		ending=$(date -u +%s)
		tested_time=$(($ending-$starting))
		#echo "tested_time=$tested_time"
done

echo "End time: $(date)" | tee -a $dir/pi.log
echo "PI_FFT pass!!" | tee -a $dir/pi.log $dir/pass.log
