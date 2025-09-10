#include <SPI.h>
#include <LoRa.h>
#include <math.h>
#include <string.h>

// Motor Pins (use GPIOs that are safe for output)
#define PUL_PIN1 D1  // GPIO5
#define DIR_PIN1 D2  // GPIO4
#define PUL_PIN2 D5  // GPIO14
#define DIR_PIN2 D6  // GPIO12

// LoRa Pins for ESP8266 (based on typical wiring)
#define LORA_SS    D8  // GPIO15
#define LORA_RST   D0  // GPIO16 (can also tie to RESET with a resistor)
#define LORA_DIO0  D3  // GPIO0

const float stepAngle = 1.8;
const int microstepping = 32;
const int stepsPerRev = (360 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

long currentPosition1 = 0;
long currentPosition2 = 0;

double lat_base = 32.990254;
double lon_base = -106.974998;
double EARTH_RADIUS = 6371.0;
double prev_bearing = 0;

// --- Math Functions ---
double toRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}

double toDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double lat1Rad = toRadians(lat1);
  double lon1Rad = toRadians(lon1);
  double lat2Rad = toRadians(lat2);
  double lon2Rad = toRadians(lon2);
  double deltaLon = lon2Rad - lon1Rad;

  double y = sin(deltaLon) * cos(lat2Rad);
  double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(deltaLon);

  double bearingRad = atan2(y, x);
  double bearing = toDegrees(bearingRad);
  return fmod(bearing + 360.0, 360.0);
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);

  double a = pow(sin(dLat / 2), 2) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) * pow(sin(dLon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS * c;
}

double calculateElevation(double alt, double dist) {
  return toDegrees(atan2(alt, dist * 1000));  // Convert dist to meters
}

// --- Motor Control ---
void moveBothMotors(float angle1, float angle2) {
  long targetSteps1 = angle1 * stepsPerDegree;
  long targetSteps2 = angle2 * stepsPerDegree;

  long stepsToMove1 = targetSteps1 - currentPosition1;
  long stepsToMove2 = targetSteps2 - currentPosition2;

  digitalWrite(DIR_PIN1, stepsToMove1 >= 0 ? HIGH : LOW);
  digitalWrite(DIR_PIN2, stepsToMove2 >= 0 ? HIGH : LOW);

  stepsToMove1 = abs(stepsToMove1);
  stepsToMove2 = abs(stepsToMove2);
  long maxSteps = max(stepsToMove1, stepsToMove2);

  for (long i = 0; i < maxSteps; i++) {
    if (i < stepsToMove1) digitalWrite(PUL_PIN1, HIGH);
    if (i < stepsToMove2) digitalWrite(PUL_PIN2, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN1, LOW);
    digitalWrite(PUL_PIN2, LOW);
    delayMicroseconds(500);
  }

  currentPosition1 = targetSteps1;
  currentPosition2 = targetSteps2;

  Serial.print("Motor 1 moved to: "); Serial.println(angle1);
  Serial.print("Motor 2 moved to: "); Serial.println(angle2);
}

void setup() {
  Serial.begin(115200);

  pinMode(PUL_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(PUL_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (1);
  }
  Serial.println("LoRa initialized");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    int firstComma = receivedData.indexOf(',');
    int secondComma = receivedData.indexOf(',', firstComma + 1);
    int thirdComma = receivedData.indexOf(',', secondComma + 1);

    if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
      Serial.println("Invalid data");
      return;
    }

    String value1Str = receivedData.substring(0, firstComma);
    String value2Str = receivedData.substring(firstComma + 1, secondComma);
    String value3Str = receivedData.substring(secondComma + 1, thirdComma);
    String value4Str = receivedData.substring(thirdComma + 1);

    double time = value1Str.toDouble();  // Unused
    double alt = value2Str.toDouble();
    double lat = value3Str.toDouble();
    double lon = value4Str.toDouble();

    double bearing = calculateBearing(lat_base, lon_base, lat, lon);
    double distance = haversineDistance(lat_base, lon_base, lat, lon);
    double elevation = calculateElevation(alt, distance);
    double act_bearing = bearing - prev_bearing;
    prev_bearing = bearing;

    moveBothMotors(act_bearing, elevation);

    Serial.print("Bearing: "); Serial.print(act_bearing); Serial.print("°  ");
    Serial.print("Distance: "); Serial.print(distance); Serial.print(" km  ");
    Serial.print("Elevation: "); Serial.println(elevation);
  }
}

FSW Format:
TeamID
Time
PacketCount
Altitude
Pressure
Temp
Voltage
Velocity
Gps_Speed
Gps_lat
Gps_long
Gps_alt
Gps_sats
Gps_time
Mag_x
Mag_y
Mag_z
Gyro_x
Gyro_y
Gyro_z
AngleRoll
AnglePitch
AngleYaw
Accel_r
Accel_p
Accel_y
Trigger
CurrentState
