#!/bin/bash
cd ./scripts/
echo "Download Houding_sanity_test"
wget --no-proxy --no-check-certificate https://mcg-depot.intel.com/artifactory/acs_test_artifacts/HOUDINI_TEST/Houdini_sanity_test.zip
echo "Unzip Houdini_sanity_test.zip"
unzip Houdini_sanity_test.zip
echo "Run Houdini_sanity_test.zip"
cd ./Houdini_sanity_test/
bash ./check_houdini.sh L `adb devices | awk 'NR == 2 { print $1 }'` | grep -e "FAIL" -e "MISSING"
if [ $? -eq 1 ]; then
    echo "PASS"
else
    echo "FAIL"
fi
cd ../
rm -rf Houdini_sanity_test Houdini_sanity_test.zip
