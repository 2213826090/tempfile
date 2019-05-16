#!/system/bin/sh
# Token Ring: Start ~

dir=$stdir

echo -e "\n\n\n"
if [ -z "$1" ]; then echo "Usage: $dir/token_ring.sh <number of loops>"; exit 1; fi
echo "Start token_ring.sh test on $(date)" | tee -a $dir/token_ring.log
looping=$1
counter=1
result=0

##### Functions
copy(){
	rm -r $2 2> /dev/null
	sync
	echo -n "		Copy from $1 to $2" | tee -a $dir/token_ring.log
	start=$(date +%s)
	cp -rf $1 $2
	sync
	stop=$(date +%s)
	duration=$((($stop-$start)/60+1))
	echo "	$duration minutes" | tee -a $dir/token_ring.log
}

compare(){
	echo "		Comparing the source file $1 and $2" | tee -a $dir/token_ring.log
	diff -r $1 $2 >> $dir/check_if_fail/error_token_ring.log
	if [ $? -ne 0 ]; then
		echo "****** Error during the diff process, $(date) ******" | tee -a $dir/token_ring.log
		echo "Failed in token_ring.sh test, $(date)" >> $dir/error.log
		result=1
	else
		echo "		No error found." | tee -a $dir/token_ring.log
		echo "		Done diff process, $(date)." | tee -a $dir/token_ring.log
	fi
}
#####

echo "		Clearing all contents from previous test in the device..."

size=0
sdloop=1
while [ $sdloop -le $stloop ]; do
	rm -r $dir/cv$sdloop/tempfile$sdloop 2> /dev/null
	mkdir $dir/cv$sdloop/tempfile$sdloop 2> /dev/null

	spaces=$(df -m | grep cv$sdloop | awk '{ print($4) }')

	if [ $size -eq 0 ]; then
		size=$spaces
	elif [ $size -ge $spaces ]; then
		size=$spaces
	fi

	sdloop=$(($sdloop+1))
done
sync

if [ $size -ge 2000 ]; then
	sourcemedia=$mediapath
elif [ $size -ge 1000 ]; then
	sourcemedia=$mediapath/1GBFiles
elif [ $size -ge 512 ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles
elif [ $size -ge 256 ]; then
	sourcemedia=$mediapath/1GBFiles/512MBFiles/256MBFiles
else
	echo "token_ring: One of device not enough space, $(date)" | tee -a $dir/error.log
	exit 1
fi

while [ $counter -le $looping ]; do 
	sdloop=1
	echo "Loop #$counter : token_ring" | tee -a $dir/token_ring.log
	copy $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop

	while [ "$sdloop" -lt "$stloop" ]; do
		copy $dir/cv$sdloop/tempfile$sdloop/src$sdloop $dir/cv$(($sdloop+1))/tempfile$(($sdloop+1))/src$(($sdloop+1))
		sdloop=$(($sdloop+1))
	done
	
	compare $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop

# Token Ring: Reverse way ~
	echo "	Reverse way" | tee -a $dir/token_ring.log
	copy $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop

	while [ "$sdloop" -gt 1 ]; do
		copy $dir/cv$sdloop/tempfile$sdloop/src$sdloop $dir/cv$(($sdloop-1))/tempfile$(($sdloop-1))/src$(($sdloop-1))
		sdloop=$(($sdloop-1))
	done

	compare $sourcemedia $dir/cv$sdloop/tempfile$sdloop/src$sdloop
	counter=$(($counter+1))
done

if [ ! -s $dir/check_if_fail/error_token_ring.log ]; then
	rm -r $dir/check_if_fail/error_token_ring.log 2> /dev/null
fi

echo "Completed Token_Ring.sh, $(date)" | tee -a $dir/token_ring.log

if [ $result -eq 0 ]; then
	echo -n "	[PASS] - " | tee -a $dir/logfile.log
else		
	echo -n "	[FAIL] - " | tee -a $dir/logfile.log
fi
# Token Ring: End ~
