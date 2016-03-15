# Hardware 
Here is described the hardware.

## Garmin GPS 18 LVC

Seller site : [GPS 18x OEM | Garmin](https://buy.garmin.com/en-US/US/oem/sensors-and-boards/gps-18x-oem/prod27594.html)

[Driver and Software](https://www8.garmin.com/support/collection.jsp?product=010-00321-31)

Please refer to APPENDIX E of [Garmin 18.pdf](Docs/GPS/Garmin 18.pdf), and download [SNSRXCFG](https://www8.garmin.com/support/download_details.jsp?id=4053) to configure the GPS.

### Configuration

Sensor configuration:
 * Earth Datum : WGS 84
 * Fix mode : Automatic
 * Differential mode : Automatic
 * Baud Rate : 38400
 * DGPS Mode : WAAS Only
 * Pulse Per Second : checked
 * Phase Output Data : NOT checked
 * PPS Auto Off Mode : NOT checked
 * Position Averaging : NOT checked
 * NMEA 2.30 Mode : checked

NMEA Sentence Selections :
 * GPRMC
 * GPGGA
 * GPGSA
 * PGRME
 * GPVTG
 * PGRMV
 
### Connection

This sensor is connected to the PC with a Serial Basic Breakout Board. 
The one used in was : [DFRobot FTDI USB to Serial Basic Breakout Board](http://www.robotshop.com/en/dfrobot-ftdi-usb-serial-basic-breakout-board.html)

The signal from the GPS has to be inverted. The [MC74HCO4AN](Docs/GPS/MC74HC04ADT.pdf) hex inverter was used.

Pinout of the GPS is in [Garmin 18.pdf](Docs/GPS/Garmin 18.pdf), section 2.1 GPS 18 LVC & GPS 18-5Hz PINOUT (page 8)

#### Wiring Diagrams

**Note** : The Serial Basic Breakout Board refers RX and TX as the ones of the peripheral board, not its own.

```
+----------------------+                 +-----------------------+-----+
|                   GND|-----------------|GND (black, 2 wires)   | GPS |
|   Serial Basic    VCC|-----------------|VCC (red)              +-----+
|  Breakout Board   RXI|-------|>o-------|RCV (green)                  |
|                   TXD|-------o<|-------|TXD (white)                  |
|                   DCD|-----------------|MPO (yellow, optional)       |
+----------------------+                 +-----------------------------+
```
For a more detailed diagram, with MC74HCO4AN connection, see [wiring.pdf](Docs/GPS/wiring.pdf)


## Drotek IMU 10DOF - MPU9250 + MS5611

Seller site : [Drotek IMU 10DOF - MPU9250 + MS5611](http://www.drotek.com/shop/fr/home/466-imu-10dof-mpu9250-ms5611.html)
This sensor is connected to the Arduino using a Category 5 cable - UDP.

### Connection
####IMU side

```
                       +----------+
                       |()  __  ()|
                       |   [__]   |
       (green)  CS_MS -|o        o|- SDI (blue)
 (green-white)    SDO -|o   ##   o|- SCL (brown-white)
(orange-white)    INT -|o   ##   o|- GND (blue-white)
      (orange) CS_MPU -|o        o|- VDD (brown)
                       | MPU 9250 |
                       |()      ()|
                       +----------+
```

#### Arduino Side

```
                                                 PC
                                                / \
                                                | |
                                              +-----+
                 +----[PWR]-------------------| USB |--+
                 |                            +-----+  |
                 |         GND/RST2  [ ][ ]            |
                 |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |
                 |          5V/MISO2 [ ][ ]  A4/SDA[ ] | 
                 |                             AREF[ ] |
                 |                              GND[ ] |
                 | [ ]N/C                    SCK/13[ ] |--- brown-white
                 | [ ]v.ref                 MISO/12[ ] |--- green-white
                 | [ ]RST                   MOSI/11[ ]~|--- blue
                 | [ ]3V3    +---+               10[ ]~|--- orange
        brown ---| [ ]5v     | A |                9[ ]~|--- green
   blue-white ---| [ ]GND   -| R |-               8[ ] |--- orange-white
                 | [ ]GND   -| D |-                    |
                 | [ ]Vin   -| U |-               7[ ] |
                 |          -| I |-               6[ ]~|
                 | [ ]A0    -| N |-               5[ ]~|
                 | [ ]A1    -| O |-               4[ ] |
                 | [ ]A2     +---+           INT1/3[ ]~|
                 | [ ]A3                     INT0/2[ ] |
                 | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |
                 | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |
                 |            [ ] [ ] [ ]              |
                 |  UNO_R3    GND MOSI 5V  ____________/
                  \_______________________/
```
Arduino ASCII art from [BusyDucks](http://busyducks.com/ascii-art-arduinos)

#### Summary

| Arduino Pin | IMU    | Cable's wire color |
|:-----------:|:------:|:------------------:|
| 8           | INT    | orange-white       |
| 9           | CS_MS  | green              |
| 10          | CS_MPU | orange             |
| 11 (MOSI)   | SDA    | blue               |
| 12 (MISO)   | SDO    | green-white        |
| 13 (SCK)    | SCL    | brown-white        |
| GND         | GND    | blue-white         |
| 5V          | VDD    | brown              |

### Axes
#### Drotek IMU default axes

Accelerometer and gyroscope :
```
           +----------+   Z
          /()  __  ()/    ↺
         /   /__/   /     |
        /o        o/      |
       /o   ##   o/       +-------⤹ Y 
      /o   ##   o/       /
     /o        o/       /
    / MPU 9250 /       ↺
   /()      ()/       X
  +----------+
```
Compass :
```
           +----------+   
          /()  __  ()/    
         /   /__/   /     
        /o        o/      
       /o   ##   o/       +------- X
      /o   ##   o/       /|
     /o        o/       / |
    / MPU 9250 /       Y  Z
   /()      ()/
  +----------+

```

#### ROS
Accelerometer, gyroscope and compass :

```
           +----------+   Z (up)
          /()  __  ()/    ↺
         /   /__/   /     |
        /o        o/      |
       /o   ##   o/       +-------⤹ Y (left)
      /o   ##   o/       /
     /o        o/       /
    / MPU 9250 /       ↺
   /()      ()/       X (front)
  +----------+
```


