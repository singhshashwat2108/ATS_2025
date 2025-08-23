#include <math.h>

// Radius of Earth in kilometers
#define EARTH_RADIUS 6371.0

// Function to convert degrees to radians
double toRadians(double degrees) {
  return degrees * (PI / 180.0);
}

// Function to calculate distance using Haversine formula
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitude and longitude from degrees to radians
  double lat1Rad = toRadians(lat1);
  double lon1Rad = toRadians(lon1);
  double lat2Rad = toRadians(lat2);
  double lon2Rad = toRadians(lon2);

  // Calculate differences
  double dLat = lat2Rad - lat1Rad;
  double dLon = lon2Rad - lon1Rad;

  // Haversine formula
  double a = pow(sin(dLat / 2), 2) +
             cos(lat1Rad) * cos(lat2Rad) * pow(sin(dLon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  // Distance in kilometers
  return EARTH_RADIUS * c;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  double lat1 = 26.4499; // Latitude of vellore
  double lon1 = 80.3319; // Longitude of Reichstag
  double lat2 = 25.5941; // Latitude of  chennai
  double lon2 = 85.1376; // Longitude of Brandenburg Gate

  // Calculate distance
  double distance = haversineDistance(lat1, lon1, lat2, lon2);

  // Print the result
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" km");
  delay(3000);
}
