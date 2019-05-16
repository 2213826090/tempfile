#!/system/bin/sh

##################################################################################
# run_socwatch.sh
#
# Description: an Android shell script that runs socwatch to monitor PM and IO.
#       The Cherrytrail concurrency team used this script because it worked better in IMIN based Android branch.
#       A $output.csv is created in the current working directory for the result.
#       A $output.sw1 is created in the current working directory for VTune amplifer
#
# Usage:
#       run_socwatch.sh <socwatch_feature_to_run> <duration_in_seconds> <socwatch_result_path> <socperf_driver_name> <socwatch_driver_name>
#
# Author: Jongyoon Choi
# Organization: PEG-SVE-DSV
# Date: 20 Apr 2015
##################################################################################

dir=`dirname $0`
feature=$1
duration=$2
output=$3
socperf_driver=$4
socwatch_driver=$5

source $dir/setup_socwatch_env.sh
insmod /lib/modules/$socperf_driver
insmod /lib/modules/$socwatch_driver

rm -r $dir/*.csv
rm -r $dir/*.sw1
rm -r $dir/*.log

chmod 766 $dir/socwatch

$dir/socwatch --max-detail -f $feature -t $duration -o $dir/$output

sleep $($duration - 5)