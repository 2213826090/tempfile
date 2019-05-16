# From the Killer: 
## I only kill the name listed in process*. 
## I am not gentle. It will be an instant kill without waiting the process to end or scream.
## And I won't clear the mess I left behind. The app that I killed may not be reused, since sometimes I leave it mutate... 
## I won't tell you after I kill. It will be silent.
# Call me anytime you want to kill by "./killer.sh"


#!/system/bin/sh

if [ ! -e "process" ]; then
	echo "Killer: I need \"process\" name list to kill!"
	exit 1
fi

echo "	Start to kill all the processes"

while read line; do
        let Pid=`ps | grep $line | awk '{print $2}'` 2> /dev/null
        let PPid=`ps | grep $line | awk '{print $3}'` 2> /dev/null
#	echo "line=$line"
#	echo "Pid=$Pid"
#	echo "PPid = $PPid"
	kill -9 $Pid 2> /dev/null
	if [ ! -z $PPid ]; then
		if [ $PPid -gt 200 ]; then
			kill -9 $PPid 2> /dev/null
			echo "Parent Process ID of $line killed."
		fi
	fi
done < process
