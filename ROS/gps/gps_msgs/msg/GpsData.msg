Header           header         # standard ROS message header

float64 latitude     # degres
float64 longitude    # degres
float32 seaHeight    # meters
float32 geoidHeight  # meters

uint8 GPS_QUALITY_NOT_AVAILABLE = 0
uint8 GPS_QUALITY_NON_DIFFERENTIAL = 1
uint8 GPS_QUALITY_DIFFERENTIAL = 2
uint8 GPS_QUALITY_ESTIMATED = 6
uint8 gpsQuality

uint8 nbSat     # number of satellite in used

bool gsaModeAuto    # Automitic position (true) or manual

uint8 FIX_TYPE_NOT_AVAILABLE = 1
uint8 FIX_TYPE_2D = 2
uint8 FIX_TYPE_3D = 3
uint8 fixType       # position calculation

uint8[12] prnUsed   # sattelites used number

float32 positionDilutionOfPrecision     # no units
float32 horizontalDilutionOfPrecision   # no units
float32 verticalDilutionOfPrecision     # no units

uint8 utcHour
uint8 utcMin
uint8 utcSec
bool validPosition      # validity of data

uint8 utcDay
uint8 utcMounth
uint8 utcYear

# Variation of magnetic direction
float32 magneticVariation # degrees; W is positive, E is negative

uint8 MODE_AUTONOMOUS = 65      # A
uint8 MODE_DIFFERENTIAL = 68    # D
uint8 MODE_ESTIMATED = 69       # E
uint8 MODE_NOT_VALID = 78       # N
uint8 mode

float32 trueCourseOverGround        # degrees
float32 magneticCourseOverGround    # degrees

float32 speedOverGround             # kilometers per hour

float32 estimatedHorizontalPosError # meters
float32 estimatedVerticalPosError   # meters
float32 estimatedPosError           # meters

float32 trueEstVelocity         # meters/second
float32 trueNorthVelocity       # meters/second
float32 upVelocity              # meters/second
