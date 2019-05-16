/*************************************************************************************/
// This document described how to use export_csv.py export csv from ET.
// And how to use config file
// csv files use for convert acs results to TRC.
// ***********************************************************************************

1. Location
Scripts: ~/acs_test_suites/OTC/utils/Export_csv_tool/export_csv.py
Config file: ~/acs_test_suites/OTC/utils/Export_csv_tool/csv_query.ini 
DEST: ~/acs_test_suites/OTC/utils/ACS_to_TRC/ET/AFT

2. Parameters
[-d] Directory which you want to save csv files
[-u] Username of access ET
[-p] Password of access ET

3. Config files
We can change config file to export components which you want.
Eg:
DEVICE_PRC = Status IN ["Approved"] AND EntityType IN [TestScript, AutomatedTest] AND AllPackages IN ["Script Library - System Functional Test"] AND Project IN ["OTC Android Testing"] AND AllPackages IN ["Chromecast", "Graphics_RenderApp", "Graphics_Display", "Graphics_System", "IRDA_Auto_Detect", "IRDA_OEM_Customization", "Multimedia_Audio", "Multimedia_Camera", "Multimedia_DRM", "Multimedia_Image", "Multimedia_Media", "Multimedia_Video", "Sensor", "EnergyManagement", "GOTA"]

You could change AllPackages IN [] to define which components you want.
Eg: 
If you want to add System_ADB, you can add "System_ADB" to AllPackages IN []:
DEVICE_PRC = Status IN ["Approved"] AND EntityType IN [TestScript, AutomatedTest] ... AllPackages IN ["Chromecast"..."System_ADB"...]


DEVICE_PRC export components for PRC
DEVICE_RO export components for RO
DEVICE_ALL export all components

4. Command

$cd ~/acs_test_suites/OTC/utils/Export_csv_tool/
$python export_csv.py -u [username] -p [password] csv_query.ini -d [directory which you want to save]





