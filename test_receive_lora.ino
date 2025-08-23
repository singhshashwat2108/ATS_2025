#include <SPI.h>
#include <LoRa.h>
#include<string.h>

#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5       // Chip Select
#define RST 32     // Reset
#define DIO0 33    // Interrupt

double equation(double x){
  return x+10;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Receiver");

  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(915E6)) { // Frequency (915 MHz to match the transmitter)
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa initialized successfully");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.println("Received packet");

    // Read full packet
    String packet = "";
    while (LoRa.available()) {
      packet += (char)LoRa.read();
    }

    // After the packet is fully read
    double number = packet.toDouble();
    Serial.print("Received number: ");
    Serial.println(number);

    // Process the number using your equation
    double result = equation(number);
    Serial.print("Result after equation: ");
    Serial.println(result);

    // Print RSSI if you want
    // Serial.print("' with RSSI ");
    // Serial.println(LoRa.packetRssi());
  }
}

