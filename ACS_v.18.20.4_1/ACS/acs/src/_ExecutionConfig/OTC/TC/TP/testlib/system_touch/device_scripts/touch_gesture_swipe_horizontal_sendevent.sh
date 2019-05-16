#! /bash/sh
N=10
event_num=$1
x1=$2
y1=$3
x2=$4
y2=$5

dy=$((($x1-$x2)/$N))

sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 53

sendevent /dev/input/event${event_num} 3 57 0
sendevent /dev/input/event${event_num} 3 53 $x1
sendevent /dev/input/event${event_num} 3 54 $y1
#touch
sendevent /dev/input/event${event_num} 1 330 1
sendevent /dev/input/event${event_num} 0 0 0

i=0
while [ $x2 -lt $x1 ]
do
    sendevent /dev/input/event${event_num} 3 53 $x1
    sendevent /dev/input/event${event_num} 0 0 0
    x1=$((($x1-$dy)))
    echo $x1
    #i=$(($i+1))
done
sendevent /dev/input/event${event_num} 3 53 $x2
sendevent /dev/input/event${event_num} 0 0 0

#untouch
sendevent /dev/input/event${event_num} 3 57 4294967295
sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 0
sendevent /dev/input/event${event_num} 1 330 0
sendevent /dev/input/event${event_num} 0 0 0

