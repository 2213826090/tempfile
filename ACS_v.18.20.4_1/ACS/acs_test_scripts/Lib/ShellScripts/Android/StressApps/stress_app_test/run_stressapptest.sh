#!/system/bin/sh

dir=`dirname $0`
rm -r $dir/*.log 2> /dev/null

echo "	Stressapptest configured to run in $1 minute(s)"

$dir/stressapptest -s $(($1*60)) -M $2 -m 2 -C 2 -i 2 -W -f $dir/tmp/file1 -f $dir/tmp/file2 --cc_test --cc_inc_count --cc_line_count --pause_delay 5 --pause_duration 2 --local_numa --remote_numa --stop_on_errors -l $dir/result_stressapptest1.log

grep -r "Status: PASS" $dir/result_stressapptest1.log
if [ $? -ne 0 ]; then
	echo "Stressapptest hit error(s). Please check the result_stressapptest1.log" | tee -a $dir/fail.log
else
	echo "Stressapptest completed after $1 minute(s)" | tee -a $dir/pass.log
fi
