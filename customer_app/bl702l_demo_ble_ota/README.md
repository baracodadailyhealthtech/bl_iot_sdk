In this demo, you can use bl702L ble master to do ota upgrade for bl702L ble slave.
This demo is just to demonstrate the function of OTA transfer, and doesn't check file version.

Test steps:

1.Build slave bin.
  (1)Configure the MACRO ROLE_SELECT to 0 in bl702l_demo_ble_ota\ble_ota_config.h.
  (2)Use genblem2s1p to generate slave bin.
  
2.Build master bin.
  (1)Configure the MACRO ROLE_SELECT to 1 in bl702l_demo_ble_ota\ble_ota_config.h.
  (2)Use genblem2s1p to generate master bin.  

3.Use flash tool to download slave bin to BL702L device. And generate ota bin.
  The original ota bin will be generated in flash tool. In order to demonstrate whether firmwware is successfully upgraded, you can 
  use a different slave bin to generate the ota file, such as different print log.
  If flash tool BouffaloLabDevCube-v1.8.5 is used, the original ota bin is BouffaloLabDevCube-v1.8.5\chips\bl702l\ota\FW_OTA.bin.xz.hash.
  FW_OTA.bin.xz.hash doesn't include ota file's length, use gen_ble_ota_bin.py in customer_app\bl702l_demo_ble_ota to generate a file that
  incldues the length of FW_OTA.bin.xz.hash and the content of FW_OTA.bin.xz.hash.
  
          python gen_ble_ota_bin.py FW_OTA.bin.xz.hash ota.bin
  
  This output file ota.bin will be downloaded to master devcie's mfg partition.
  Master device will get the length of FW_OTA.bin.xz.hash and transport FW_OTA.bin.xz.hash to slave.
  
4.Use flash tool to download master bin and ota bin to BL702L device.
  The genrated ota.bin in step3 is downloaded to mfg partition.
  
5.Master will automatically do ble scan.
  Slave will not automatically do ble advertising, please input 'ota_start' cli comamnd to make it do advertising with device name 'BL702L_OTA'.
  Master will check the devcie name in received advertising packet, if the device name is 'BL702L_OTA', master will automatilly connect slave device and do OTA.