#!/bin/bash
if [ $# -ne 1 ]&&[ $# -ne 2 ]
then
  echo "Usage: `basename $0` [-w] path_to_campaigns_folder"
  echo ""
  echo " This script has been made in order to help you to validate that the TCs you refering to in your campaign XML files really exist as XML files in the file system"
  echo ""
  echo " Option:"
  echo ""
  echo "   -w: activate Windows folder separation char check"
  exit -1
fi

if [ $# == 2 ]
then
  camp_folder=$2
else
  camp_folder=$1
fi

echo ""

# Move the the campaign folder
current_folder=`pwd`
cd $camp_folder

# Check that the campaign folder passed as parameter is valid
if [ $? -ne 0 ]
then
  echo "path_to_campaign_folder is not valid"
  exit -1
fi

# for each TC in every campaign file
for tc in `grep -i "<TestCase Id" *.xml|sed "s#.*[Ii][Dd] *= *[\"\']\(.*\)[\"\'].*#\1#g"|sort|uniq`; do

 # Check windows file separator char
 if [ "$1" == "-w" ]&&[[ "$tc" =~ \\ ]]
 then
  echo " !! WARNING !! Your TC ID contains Windows specific file separator '\\'"
  echo " $tc"
  echo ""
 fi

  # Check windows file separator char
 if [[ "$tc" =~ \\ ]]
 then

  if [ "$1" == "-w" ]
  then
   echo " !! WARNING !! Your TC ID contains Windows specific file separator '\\'"
   echo " $tc"
   echo ""
  fi

  # Convert to linux folder separation char
  tc2=`echo "$tc" | sed 's#\\\\#\\/#g'`
  # Convert to egrep searchable path
  tc3=`echo "$tc" | sed 's#\\\\#\[\\\]#g'`

 else
  tc2=$tc
  tc3=$tc
 fi

 # Check TC file existance
 if [ ! -f "$tc2.xml" ]
 then
  echo "  >> FAILURE for $tc found in:"
  # Find in which campaign file it has been found
  egrep "$tc3" *.xml|sed "s/:.*//g"|uniq
  echo ""
 fi
done

cd $current_folder
