Python tools Check_Checksum check if the folder or the zip given in parameter is not modified by accident.

How do you runs the tools:
--------------------------

1- python _Tools/Check_Checksum.py $ARG

    $ARG is a report ACS folder or a report in format .tar.gz

Example: python _Tools/Check_Checksum.py _Reports/2015-10-09_08h59.36.7_Dummy_Campaign_Config.tar.gz
Example: python _Tools/Check_Checksum.py _Reports/2015-10-09_08h59.36.7_Dummy_Campaign_Config

Example of stdout:

************************************************************************************
List of file modified:
************************************************************************************
ALR_live_reporting.log
Original Checksum: 0c2bd35e9e2c307cb70a1c9199f4ea7298dcf0e06e533223762335102a370c53
Modified Checksum: 76b9989780f6150ec5e4c30ff86e87d3285a8e168077ddfb607c3fb94e80ca0d

************************************************************************************
report.xsl
Original Checksum: New File
Modified Checksum: ab377baefb401b9a85a1eaddad4b62f27dcced26f600a83c4898f357934a0410

************************************************************************************

