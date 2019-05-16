#!/bin/bash

cd ..
cp -r TCInterface/TCInterface ../../../acs/src/_ExecutionConfig/
cp TCInterface/run.sh ../../../acs/src/_ExecutionConfig/
cp TCInterface/CampaignTemplate.xml ../../../acs/src/_ExecutionConfig/
cp TCInterface/TCList.txt ../../../acs/src/_ExecutionConfig/

cd ../../../acs/src/_ExecutionConfig

find . -iname "*.xml" | grep "/TC/"| grep "/TC/" | cut -c 3- > TCList.txt

cd TCInterface/
javac ACSTCInterface.java
