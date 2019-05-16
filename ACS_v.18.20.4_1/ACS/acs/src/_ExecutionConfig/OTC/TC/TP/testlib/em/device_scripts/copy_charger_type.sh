i=0
echo $$ > pid.txt
while [ $i -lt 10 ]
do
    cp $1 $2
    sleep $3
done
