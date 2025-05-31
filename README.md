# Intro
Arduino-based balancing robot, from utter scratch  

  
I hate my life

# balancing_robot_v3.ino

it includes the following headers.  

| Headers  | Reference | Loaction |
| ------------- | ------------- | ------------- |
| <Bluepad32.h>  | [link](https://github.com/ricardoquesada/bluepad32)| :C:\Users\user\AppData\Local\Arduino15\packages\esp32-bluepad32\hardware\esp32\4.1.0\tools\sdk\esp32\include\bluepad32.h  |
| <Wire.h>  |[link](https://docs.arduino.cc/language-reference/en/functions/communication/wire/)| C:\Users\user\AppData\Local\Arduino15\packages\esp32-bluepad32\hardware\esp32\4.1.0\libraries\Wire\src\Wire.h  |
| <Adafruit_MPU6050.h> |[link](https://github.com/adafruit/Adafruit_MPU6050) |C:\Users\user\Documents\Arduino\libraries\Adafruit_MPU6050\Adafruit_MPU6050.h |
| <Adafruit_Sensor.h> |[link](https://github.com/adafruit/Adafruit_Sensor) |C:\Users\user\Documents\Arduino\libraries\Adafruit_Unified_Sensor\Adafruit_Sensor.h |
|driver/twai.h |[link](https://github.com/espressif/esp-idf/tree/master)| C:\Users\user\AppData\Local\Arduino15\packages\esp32-bluepad32\hardware\esp32\4.1.0\tools\sdk\esp32\include\driver\include\driver\twai.h |
|xiaomi_cybergear_driver.h |[link](https://github.com/DanielKalicki/Xiaomi_CyberGear_Arduino)| C:\Users\user\Documents\Arduino\libraries\xiaomi_cybergear\xiaomi_cybergear_driver.h |
|I2Cdev.h |[link](https://github.com/UncleRus/esp-idf-lib/blob/master/components/i2cdev/i2cdev.h)|C:\Users\user\Documents\Arduino\libraries\I2Cdev\I2Cdev.h |

# BOM

MCU : [ESP32 WROOM-32E module](https://www.google.com/search?q=esp32+wroom+datasheet&sca_esv=b5e030dce8bbb8a8&ei=2qg6aMeVNt-n2roP-aG38QU&oq=esp32+wroom+data&gs_lp=Egxnd3Mtd2l6LXNlcnAiEGVzcDMyIHdyb29tIGRhdGEqAggAMgUQABiABDIFEAAYgAQyBBAAGB4yBhAAGAgYHjIGEAAYCBgeMgYQABgIGB4yBhAAGAgYHjIGEAAYCBgeMgYQABgIGB4yCBAAGAgYChgeSLkGUDZYrwNwAXgBkAEAmAHQAaABmgaqAQUwLjQuMbgBAcgBAPgBAZgCBqACwQbCAgoQABiwAxjWBBhHwgIKEAAYgAQYQxiKBZgDAIgGAZAGCpIHBTEuNC4xoAf5H7IHBTAuNC4xuAe8BsIHBTItNS4xyAcj&sclient=gws-wiz-serp) 

IDE : Arduino IDE (supports ESP32 module bia board manager)  

IMU : [MPU6050](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) 

motor : [xiaomi cybergear](https://github.com/belovictor/cybergear-docs) 

CAN to UART : [CANBus Unit(CA-IS3050G)](https://shop.m5stack.com/products/canbus-unitca-is3050g?srsltid=AfmBOoo45a3Z4aiPm0DFpa_wk6L1i1zsp_HxvBCnvUw1hWO9vbXH9BUA)  

Teleop : [dualshock 4](https://www.playstation.com/ko-kr/accessories/dualshock-4-wireless-controller/)  

Chassis : 3D-print (PLA, [Bambu Lab Carbon X1](https://kr.store.bambulab.com/products/x1-carbon))  

wheels : [Ø100 wheel](https://kr.misumi-ec.com/vona2/detail/221000193783/?ProductCode=S-100UR)

Battery : [coms 24V 3A](https://www.coupang.com/vp/products/6761147603?itemId=15845511242&vendorItemId=83792381924&src=1042503&spec=10304025&addtag=400&ctag=6761147603&lptag=6761147603-15845511242&itime=20250531162122&pageType=PRODUCT&pageValue=6761147603&wPcid=17369392389484159327326&wRef=&wTime=20250531162122&redirect=landing&gclid=Cj0KCQjw0erBBhDTARIsAKO8iqRPxm5TNP5QtiZhhECka1vpn2KdWHgOK1eWB05oxIEn7RVgEn0xRuoaAqtkEALw_wcB&mcid=64c3aae965194f1b8afdfc962537f89e&campaignid=21519412236&adgroupid=)

# CAD
the files are on the models folder : .stp, .CATProduct, .SLDASM  
<img src="https://github.com/user-attachments/assets/fb096a44-df72-425c-9a0b-45032a784a4b" width="800">  

# Communication Diagram
**Diagram for wiring & communication**  
<img src="https://github.com/user-attachments/assets/ca0b048f-2eb5-48bb-8f68-40d9d4971ba6" width="1000">  
red : Power train  
blue : Communication

**Diagram for logic**  
<img src="https://github.com/user-attachments/assets/b378d3ac-ae88-4715-8365-010987db7762" width="1000">  
oragne : Inner PID loop    
red : Outer PID loop  
blue : Human expererience

# Demo video
[![Video Label](https://github.com/user-attachments/assets/e69b6bc8-4ccc-4300-9a95-97ef5d8b9e3e)](https://www.youtube.com/shorts/0yKNOIgKEw8?si=WzQcaBdiawOKCOyA)

# Some Photos
<img src="https://github.com/user-attachments/assets/d677fda0-eca6-4935-aa0a-5d9cac4aef55" width="400">
<img src="https://github.com/user-attachments/assets/4383606e-8685-4943-bc71-b843edf0f009" width="400">

# TO-DO
1. PCB integration
<img src="https://github.com/user-attachments/assets/5dd9b5f5-c43d-4ed6-b806-f62461ed27d9" width="400">
<img src="https://github.com/user-attachments/assets/3f1b456c-2aea-454e-8a12-a100a7784bf8" width="400">


