#include <SPI.h>
#include <LoRa.h>

#define SCK 18    // SPI Clock
#define MISO 19   // SPI MISO
#define MOSI 23   // SPI MOSI
#define SS 5      // LoRa Chip Select
#define RST 32    // LoRa Reset
#define DIO0 33   // LoRa IRQ (interrupt)

bool shouldSend = false;
int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to be available

  Serial.println("Starting setup...");

  // Initialize SPI with custom pins
  SPI.begin(SCK, MISO, MOSI, SS);

  // Configure LoRa module pins
  LoRa.setPins(SS, RST, DIO0);

  Serial.println("Initializing LoRa...");

  // Initialize LoRa at 915 MHz (adjust frequency as needed)
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (true);  // Halt here if LoRa fails
  }

  Serial.println("LoRa initialized successfully.");

  // Optionally send a startup message
  LoRa.beginPacket();
  LoRa.print("7");  // Initial reset value or status
  LoRa.endPacket();
}

void loop() {
  // Check for serial commands to start/stop transmission
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any newline/whitespace

    if (input.equalsIgnoreCase("stop")) {
      shouldSend = false;
      Serial.println("Transmission stopped.");
    } else if (input.equalsIgnoreCase("start")) {
      shouldSend = true;
      Serial.println("Transmission resumed.");
    }
  }

  // Send LoRa packet if allowed
  if (shouldSend) {
    counter++;
    
    LoRa.beginPacket();
    LoRa.print(counter);
    LoRa.endPacket();

    Serial.print("Sent packet: ");
    Serial.println(counter);

    delay(1000);  // Wait 1 second before next transmission
  }
}
