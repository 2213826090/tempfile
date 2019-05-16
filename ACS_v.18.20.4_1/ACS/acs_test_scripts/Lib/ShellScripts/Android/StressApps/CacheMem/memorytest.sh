#!/system/bin/sh
#
#memorytest.sh
#
# Shell script to help isolate memory failures under linux
#
# Author: Doug Ledford  + contributors
#
# (C) Copyright 2000-2002 Doug Ledford; Red Hat, Inc.
# This shell script is released under the terms of the GNU General

# Public License Version 2, June 1991.  If you do not have a copy
# of the GNU General Public License Version 2, then one may be
# retrieved from http://people.redhat.com/dledford/GPL.html
#
# Note, this needs bash2 for the wait command support.

# This is where we will run the tests at
if [ -z "$TEST_DIR" ]; then
  PS1=`dirname $0`;
  TEST_DIR=$PS1;
fi
# The location of the linux kernel source file we will be using
if [ -z "$SOURCE_FILE" ]; then
  tar -xzf $TEST_DIR/memorytest.tar.gz -C $TEST_DIR
  SOURCE_FILE=linux.tar.gz;
fi

if [ ! -f "$TEST_DIR/$SOURCE_FILE" ]; then
  echo "Missing source file $SOURCE_FILE"
  exit 1
fi

# How many passes to run of this test, higher numbers are better
#if [ -z "$NR_PASSES" ]; then
#  NR_PASSES=99999
#fi

# Guess how many megs the unpacked archive is
if [ -z "$MEG_PER_COPY" ]; then
   MEG_PER_COPY=228
 # MEG_PER_COPY=$(ls -l $SOURCE_FILE | awk '{print int($5/1024/1024) * 4}')
fi


# How many trees do we have to unpack in order to make our trees be larger
# than physical RAM?  If we don't unpack more data than memory can hold
# before we start to run the diff program on the trees then we won't
# actually flush the data to disk and force the system to reread the data
# from disk.  Instead, the system will do everything in RAM.  That doesn't
# work (as far as the memory test is concerned).  It's the simultaneous
# unpacking of data in memory and the read/writes to hard disk via DMA that
# breaks the memory subsystem in most cases.  Doing everything in RAM without
# causing disk I/O will pass bad memory far more often than when you add
# in the disk I/O.

NR_SIMULTANEOUS=$2  # Recommend 3 instances for concurrency tests, since the test will consume most of the Memory resources.

if [ -z "$NR_SIMULTANEOUS" ]; then
	NR_SIMULTANEOUS=$(free | awk -v meg_per_copy=$MEG_PER_COPY 'NR == 2 {print int($2*1.5/1024/meg_per_copy + (($2/1024)%meg_per_copy >= (meg_per_copy/2)) + (($2/1024/32) < 1))}')
	if [ $? -ne 0 ]; then
		NR_SIMULTANEOUS=3
	fi
fi

echo "Commence memory test";
#echo "Number of cycles = $NR_PASSES";
echo "Number of Simultaneous iterations per cycle = $NR_SIMULTANEOUS";

# Should we unpack/diff the $NR_SIMULTANEOUS trees in series or in parallel?

PARALLEL=y
if [ ! -z "$PARALLEL" ]; then
  PARALLEL="yes"
	echo "CacheMem: PARALLEL enabled"
else
  PARALLEL="no"
	echo "CacheMem: PARALLEL disabled"
fi

if [ ! -z "$JUST_INFO" ]; then
  echo "TEST_DIR:		$TEST_DIR"
  echo "SOURCE_FILE:		$SOURCE_FILE"
#  echo "NR_PASSES:		$NR_PASSES"
  echo "MEG_PER_COPY:		$MEG_PER_COPY"
  echo "NR_SIMULTANEOUS:	$NR_SIMULTANEOUS"
  echo "PARALLEL:		$PARALLEL"
  echo
  exit
fi

  echo "TEST_DIR:               $TEST_DIR"
  echo "SOURCE_FILE:            $SOURCE_FILE"
#  echo "NR_PASSES:              $NR_PASSES"
  echo "MEG_PER_COPY:           $MEG_PER_COPY"
  echo "NR_SIMULTANEOUS:        $NR_SIMULTANEOUS"
  echo "PARALLEL:               $PARALLEL"
  echo
cd $TEST_DIR

# Remove any possible left over directories from a cancelled previous run
rm -r linux linux.orig linux.pass.* 2> /dev/null
sync

# Unpack the one copy of the source tree that we will be comparing against
tar -xzvf $SOURCE_FILE
wait
mv linux linux.orig

test_time=$(($1*60))
tested_time=0
start_time=$(date -u +%s)
i=1

function timecheck
{
        ending=$1
        tested_time=$(($ending-$start_time))
        if [ $tested_time -gt $test_time ]; then
                echo "  Memory Test Completed. $(date)"
                exit 1
        fi
}

function see_diff() 
{
	if [ ! -z `cat $1` ]; then
		echo "  CacheMem: Memory test in CacheMem found diff error in $1. $(date)" | tee -a $dir/fail.log
		exit 1
	else
		echo "  CacheMem: diff found no error in $1"
	fi
}


echo "Mem: Test start = $(date), with total test time $1 minutes";
while [ $tested_time -lt $test_time ]; do
	echo "Iteration number $i, $(date)";
  	j=0
  	while [ "$j" -lt "$NR_SIMULTANEOUS" ]; do
    		if [ $PARALLEL = "yes" ]; then
      			(mkdir "parallel.$j" 2> /dev/null; tar -xzf $SOURCE_FILE -C parallel.$j; mv parallel.$j/linux linux.pass.$j; rm -r "parallel.$j" 2> /dev/null) &
    		else
      			tar -xzf $SOURCE_FILE
      			mv linux linux.pass.$j
    		fi
    		echo "  Extracted linux.pass.$j";
    		j=`expr $j + 1`
  	done 
  	wait
	
	timecheck `date -u +%s`	

  	j=0 
  	while [ "$j" -lt "$NR_SIMULTANEOUS" ]; do
    		echo "  Compare linux.pass.$j with linux.orig";
    		if [ $PARALLEL = "yes" ]; then
      			(diff -U 3 -rN linux.orig linux.pass.$j > memory_diff.$j; see_diff memory_diff.$j; rm -r linux.pass.$j memory_diff.$j 2> /dev/null) &
    		else
      			diff -U 3 -rN linux.orig linux.pass.$j > memory_diff.$j
			see_diff memory_diff.$j
      			rm -r linux.pass.$j
    		fi
    		j=`expr $j + 1`
  	done
  	wait
  	i=`expr $i + 1`
  	echo -e "\n"

	end_time=$(date -u +%s)
	tested_time=$(($end_time-$start_time))
done

echo "Memory test stop = $(date)"

# Clean up
rm -r linux linux.orig linux.pass.* 2> /dev/null
