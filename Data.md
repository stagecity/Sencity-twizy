# Sencity-twizy data specification

Here is the data specification of saved files.
The structures of each following section are stored sequentially, so that a file is like an array of structure.

## LIDAR
File name : VLP-16_{unix time, format : seconds.nanoseconds}.bin

```C
typedef struct {
    uint64_t stamp;     /**< ROS timestamp in nano seconds */
    float x;            /**< Forward (relative to car), in meter from LIDAR center */
    float y;            /**< Left (relative to car), in meter from LIDAR center */
    float z;            /**< Up (relative to car), in meter from LIDAR center */
    uint8_t ring;       /**< Velodyne laser number */
    uint8_t intensity;  /**< Laser intensity measured */
} __attribute__((packed)) Point;
```

## GPS
File name : GPS_{unix time, format : seconds.nanoseconds}.bin

```C
typedef struct {
    uint64_t stamp;         /**< ROS timestamp in nano seconds */
    double latitude;        /**< GPS latitude in degrees */
    double longitude;       /**< GPS longitude in degrees */
    float seaHeight;        /**< Height above sea in meters */
    float geoidHeight;      /**< Geoidal height in meters */
    float positionDilutionOfPrecision;      /**< Position dilution of precision */
    float horizontalDilutionOfPrecision;    /**< Horizontal dilution of precision */
    float verticalDilutionOfPrecision;      /**< Vertical dilution of precision */
    float magneticVariation;                /**< Magnetic variation of direction in degrees*/
    float trueCourseOverGround;             /**< True course over ground in degrees */
    float magneticCourseOverGround;         /**< Magnetic course over ground in degrees */
    float speedOverGround;                  /**< Speed over ground in kilometers/hour */
    float estimatedHorizontalPosError;      /**< Estimated horizontal position error in meters */
    float estimatedVerticalPosError;        /**< Estimated vertical position error in meters */
    float estimatedPosError;        /**< Estimated position error in meters */
    float trueEstVelocity;          /**< True est velocity in meters/second */
    float trueNorthVelocity;        /**< True north velocity in meters/second */
    float upVelocity;       /**< Up velocity in meters/second */
    uint8_t gpsQuality;     /**< GPS quality :GPS quality indication, 0 = fix not available, 1 = Non-differential GPS fix available, 2 = Differential GPS (WAAS) fix available, 6 = Estimated */
    uint8_t nbSat;          /**< Number of satellite in view */
    uint8_t gsaModeAuto;    /**< Automatic mode */
    uint8_t fixType;        /**< Fix type, 1 = not available, 2 = 2D, 3 = 3D */
    uint8_t utcHour;        /**< UTC hour */
    uint8_t utcMin;         /**< UTC minute */
    uint8_t utcSec;         /**< UTC second */
    uint8_t validPosition;  /**< is position valid : boolean  */
    uint8_t utcDay;         /**< UTC day */
    uint8_t utcMounth;      /**< UTC mounth */
    uint8_t utcYear;        /**< UTC Year */
    uint8_t mode;           /**< Mode indicator : A = Autonomous, D = Differential, E = Estimated, N = Data not valid */
    uint8_t prnUsed[12];    /**< PRN number, 01 to 32, of satellite used in solution, up to 12 transmitted */
} GpsBinaryData;
```

## IMU
File name : IMU_{unix time, format : seconds.nanoseconds}.bin

```C
typedef struct {
    uint64_t stamp;     /**< ROS timestamp in nanoseconds */
    double accel[3];    /**< 3D accelerometer vector. Composant in meter/second^2 (SI)  */
    double gyro[3];     /**< 3D gyroscope vector. Composant in rad/second  */
    double mag[3];      /**< 3D magnetometer vector. Composant in Tesla */
} ImuBinaryData;
```

This 3D vectors are defined as :
 * vector[0] = x (Forward (relative to car))
 * vector[1] = y (Left (relative to car))
 * vector[2] = z (Up (relative to car))
    
## WiFi
File name : WIFI_{unix time, format : seconds.nanoseconds}.bin
```C
// Char size of MAC address
#define ADDRESS_SIZE    18
// max char size of WiFi SSID
#define SSID_SIZE       30
// Max char size of WiFi security information
#define SECURITY_SIZE   50
// Max char size of security group
#define GROUP_SIZE      20
// Max char size of security pair information
#define PAIR_SIZE       20
// Max char size of protocol information
#define PROTOCOL_SIZE   20

typedef struct {
    uint64_t stamp;                 /**< ROS timestamp in nano seconds */
    uint16_t frequency;             /**< WiFi frequency (channel) in MHz */
    char address[ADDRESS_SIZE];     /**< WiFi access point MAC address */
    char ssid[SSID_SIZE];           /**< WiFi SSID */
    char security[SECURITY_SIZE];   /**< WiFi security protocol */
    char groupCipher[GROUP_SIZE];   /**< WiFi security group cipher */
    char pairwiseCiphers[PAIR_SIZE];/**< WiFi security pair ciphers */
    char protocol[PROTOCOL_SIZE];   /**< WiFi protocol */
    uint8_t signal;                 /**< signal strengh form 0 to 100(highest) */
    uint8_t encryption;             /**< is WiFi encrypted (true is non null) */
} WifiBinaryData;
```


