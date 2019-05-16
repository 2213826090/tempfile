#!/system/bin/sh

dir=$stdir

# dd-ing: Start ~
echo -e "\n\n\n"
if [ -z "$1" ]; then echo "Usage: $dir/dd-ing.sh <number of loops>"; exit 1; fi
echo "Start dd-ing.sh test on $(date)" | tee -a $dir/dd-ing.log
looping=$1
counter=1
result=0
echo "	Run dd-ing.sh with $looping loop(s)" | tee -a $dir/dd-ing.log

echo "		Clearing all contents from previous test in the device..."

sdloop=1
while [ $sdloop -le $stloop ]; do
	rm -r $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
	mkdir -p $dir/cv$sdloop/tempfile$sdloop/src$sdloop 2> /dev/null
	sdloop=$(($sdloop+1))
done
sync

while [ $counter -le $looping ]; do 
	sdloop=1
	counter1=1
	echo "Loop #$counter : dd-ing" | tee -a $dir/dd-ing.log
	cvrandom=$(($$$(date +%s)%20+1))
	
	kilomega=$(($counter%2))
	echo $kilomega

	if [ $(($counter%2)) -eq 0 ]; then
		multiplier=1000
		echo "		Each test file size is $cvrandom kByte" | tee -a $dir/dd-ing.log
	else 
		multiplier=1000000
		echo "		Each test file size is $cvrandom MByte" | tee -a $dir/dd-ing.log
	fi
	
	while [ $sdloop -le $stloop ]; do
		echo -n "		'dd-ing' the test files to $dir/cv$sdloop/tempfile$sdloop/src$sdloop/" | tee -a $dir/dd-ing.log
		start=$(date +%s)
		counter1=1
		while [ "$counter1" -le 100 ]; do
			dd if=/dev/urandom of=$dir/cv$sdloop/tempfile$sdloop/src$sdloop/cv_dd_$counter1 bs=$multiplier count=$cvrandom  
			counter1=$(($counter1+1))
		done
		sync
		wait
		stop=$(date +%s)
		duration=$(($stop-$start))
		echo "	$duration seconds" | tee -a $dir/dd-ing.log
		rm -r $dir/cv$sdloop/tempfile$sdloop/src$sdloop/*
		sync
		sdloop=$(($sdloop+1))
	done
	counter=$(($counter+1))
done
echo "	**No compare machanism is needed. Please check the duration time in the log file for each storage device." | tee -a $dir/dd-ing.log
echo "	**Fail the test if the duration is unreasonable" | tee -a $dir/dd-ing.log
echo "Completed dd-ing.sh, $(date)" | tee -a $dir/dd-ing.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# dd-ing: End ~
