#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// Motor Pins (ESP8266 GPIO mapping)
#define PUL_PIN1 5   // D1
#define DIR_PIN1 4   // D2
#define PUL_PIN2 14  // D5
#define DIR_PIN2 12  // D6

// LoRa Pins (ESP8266 specific)
#define LORA_SS    15  // D8
#define LORA_RST   16  // D0
#define LORA_DIO0  0   // D3

const float stepAngle = 1.8;
const int microstepping = 32;
const int stepsPerRev = (360.0 / stepAngle) * microstepping;
const float stepsPerDegree = stepsPerRev / 360.0;

volatile long currentPosition1 = 0;
volatile long currentPosition2 = 0;

const double lat_base = 32.990254;
const double lon_base = -106.974998;
const double EARTH_RADIUS = 6371.0;
double prev_bearing = 0;

// --- Math Functions ---
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  const double lat1Rad = lat1 * M_PI / 180.0;
  const double lon1Rad = lon1 * M_PI / 180.0;
  const double lat2Rad = lat2 * M_PI / 180.0;
  const double deltaLon = (lon2 - lon1) * M_PI / 180.0;

  const double y = sin(deltaLon) * cos(lat2Rad);
  const double x = cos(lat1Rad)*sin(lat2Rad) - sin(lat1Rad)*cos(lat2Rad)*cos(deltaLon);
  
  return fmod(atan2(y, x) * 180.0 / M_PI + 360.0, 360.0);
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const double dLat = (lat2 - lat1) * M_PI / 180.0;
  const double dLon = (lon2 - lon1) * M_PI / 180.0;
  
  const double a = pow(sin(dLat/2), 2) + 
                   cos(lat1*M_PI/180.0) * cos(lat2*M_PI/180.0) * 
                   pow(sin(dLon/2), 2);
                   
  return EARTH_RADIUS * 2 * atan2(sqrt(a), sqrt(1-a));
}

//  Motor Control
void moveBothMotors(float azimuth, float elevation) {
  const long target1 = azimuth * stepsPerDegree;
  const long target2 = elevation * stepsPerDegree;
  
  const long steps1 = target1 - currentPosition1;
  const long steps2 = target2 - currentPosition2;

  digitalWrite(DIR_PIN1, steps1 >= 0 ? HIGH : LOW);
  digitalWrite(DIR_PIN2, steps2 >= 0 ? HIGH : LOW);

  const unsigned long steps = max(abs(steps1), abs(steps2));
  const unsigned long stepDelay = 1000;  // µs per step
  
  for(unsigned long i = 0; i < steps; i++) {
    if(i < abs(steps1)) digitalWrite(PUL_PIN1, HIGH);
    if(i < abs(steps2)) digitalWrite(PUL_PIN2, HIGH);
    delayMicroseconds(stepDelay/2);
    digitalWrite(PUL_PIN1, LOW);
    digitalWrite(PUL_PIN2, LOW);
    delayMicroseconds(stepDelay/2);
  }

  currentPosition1 = target1;
  currentPosition2 = target2;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  // Motor control setup
  pinMode(PUL_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(PUL_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(PUL_PIN1, LOW);
  digitalWrite(PUL_PIN2, LOW);

  // LoRa initialization
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed!");
    while(1);
  }
  LoRa.setSyncWord(0x12);
  Serial.println("LoRa initialized");
}

void loop() {
  static String packetBuffer;
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    packetBuffer = "";
    while (LoRa.available()) {
      packetBuffer += (char)LoRa.read();
    }

    char buffer[256];
    char* fields[28];
    int fieldCount = 0;
    
    packetBuffer.toCharArray(buffer, sizeof(buffer));
    char* token = strtok(buffer, ",");
    
    while(token != NULL && fieldCount < 28) {
      fields[fieldCount++] = token;
      token = strtok(NULL, ",");
    }

    if(fieldCount < 13) {
      Serial.println("Invalid packet");
      return;
    }

    // GPS data extraction
    double gps_lat = atof(fields[9]);   // 10th field
    double gps_lon = atof(fields[10]);  // 11th field
    double gps_alt = atof(fields[11]);  // 12th field

    // Position calculation
    const double bearing = calculateBearing(lat_base, lon_base, gps_lat, gps_lon);
    const double distance = haversineDistance(lat_base, lon_base, gps_lat, gps_lon);
    const double elevation = atan2(gps_alt, distance*1000) * 180.0 / M_PI;
    const double azDelta = bearing - prev_bearing;
    
    prev_bearing = bearing;

    // Motor control
    moveBothMotors(azDelta, elevation);

    // Status output
    Serial.printf("Lat: %.6f Lon: %.6f Alt: %.1fm | Az: %.1f° El: %.1f°\n",
                  gps_lat, gps_lon, gps_alt, azDelta, elevation);
  }
}
