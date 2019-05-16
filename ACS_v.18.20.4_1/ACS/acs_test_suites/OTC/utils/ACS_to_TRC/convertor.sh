<<COMMENT
    Copyright 2014 Android Open Source Project

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
COMMENT

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

ACS_XML=$MY_PATH"/ACS_XML"
ET=$MY_PATH"/ET"
TIMESTAMP=`date '+%d-%m_%H-%M-%S'`
BUILD_ID=$1
TRC=$MY_PATH"/TRC/"$BUILD_ID"-TRC-"$TIMESTAMP

#execution folders
AFT=$ET"/AFT/"
BAT=$ET"/BAT/"

#default execution set
EXEC_SET="AFT"
if [ ! -d $ACS_XML ]
then
    echo "You need a folder named ACS_XML where ACS xml results are stored, at the same level with this script"
    exit
fi

if [ -z "$1" ]
then
   echo "You need to specify a build id. ex: bash convertor.sh COHOL00835"
   exit
fi

if [ ! -z "$2" ]
then
    EXEC_SET=$2
fi

echo "Using execution set: "$EXEC_SET
cd $ET"/"$EXEC_SET
for f in *;
do
    cp $f ../"added"$f
done
cd $MY_PATH


#use like: bash convertor.sh 00835
cd $ET
for f in *.csv;
do
    #echo $f;
    con=0
    while read line;
    do
        if [ $con -eq 1 ]
        then
            echo $line >> ET_DB.csv;
        else
            #skip csv header
            con=1
        fi
    done <$f;
done
sed -i "s/Subject\/SSG\/System Functional Tests\//Script Library - System Functional Test|/" ET_DB.csv
sed -i "s/Subject\/SSG\/BAT\//Script Library - BAT|/" ET_DB.csv
sed -i "/.*,/ s/\//|/" ET_DB.csv
cd ..
sleep 1

#convertor
python ACS_to_TRC.py -if $ACS_XML -of tmp -ef $ET -r $TRC -exec_set $EXEC_SET -id "executed in $1"

#cleanup
cd ET
for f in *.csv;
do
    rm $f
done
cd ..

if [ -d "tmp" ]
then
    rm -r tmp
fi

cd "TRC/"$BUILD_ID"-TRC-"$TIMESTAMP
if [ -f "None.csv" ]
then
    rm "None.csv"
fi

echo "TRC upload ready results folder is: $TRC"
