#include <math.h>
double EARTH_RADIUS=6371.0;
// Correct conversion functions using M_PI for more accuracy
double toRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}

double toDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

// Fixed bearing calculation
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
  
  // Normalize to 0-360
  bearing = fmod(bearing + 360.0, 360.0);
  return bearing;
}

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
  Serial.println("Setup complete");
}

void loop() {
  // Correct coordinates for Vellore and Chennai
  double lat1 = 26.4499; // Latitude of Vellore
  double lon1 = 80.3319; // Longitude of Vellore
  double lat2 = 25.5941; // Latitude of Chennai
  double lon2 = 85.1376; // Longitude of Chennai

  double bearing = calculateBearing(lat1, lon1, lat2, lon2);
  double distance = haversineDistance(lat1, lon1, lat2, lon2);
  
  Serial.print("Bearing: ");
  Serial.print(bearing, 4);  
  Serial.print("°");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" km");
  delay(2000);
}