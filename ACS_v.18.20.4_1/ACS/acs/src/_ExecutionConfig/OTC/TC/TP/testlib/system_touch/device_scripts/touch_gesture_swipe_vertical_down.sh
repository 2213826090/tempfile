#! /bash/sh
N=30
event_num=$1
x1=$2
y1=$3
x2=$4
y2=$5
dy=$((($y2-$y1)/$N))

# need swipe TS from Down to TOP by finger(Up swipe)

sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 53

sendevent /dev/input/event${event_num} 3 57 0
sendevent /dev/input/event${event_num} 3 53 $x2
sendevent /dev/input/event${event_num} 3 54 $y2
#touch
sendevent /dev/input/event${event_num} 1 330 1
sendevent /dev/input/event${event_num} 0 0 0

i=0
while [ $y1 -lt $y2 ]
do
    sendevent /dev/input/event${event_num} 3 54 $y2
    sendevent /dev/input/event${event_num} 0 0 0
    y2=$((($y2-$dy)))
    echo $y2
    #i=$(($i+1))
done
sendevent /dev/input/event${event_num} 3 54 $y1
sendevent /dev/input/event${event_num} 0 0 0

#untouch
sendevent /dev/input/event${event_num} 3 57 4294967295
sendevent /dev/input/event${event_num} 3 47 0
sendevent /dev/input/event${event_num} 3 57 0
sendevent /dev/input/event${event_num} 1 330 0
sendevent /dev/input/event${event_num} 0 0 0

