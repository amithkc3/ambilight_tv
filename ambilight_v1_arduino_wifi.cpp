#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

// UDP settings
WiFiUDP udp;
unsigned int localUdpPort = 4210;  // Local port to listen on
char incomingPacket[1024];  // Buffer for incoming packets

// NeoPixel settings
#define LED_PIN D5
#define LED_COUNT 200  // Number of LEDs in the strip
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Start the Serial communication
  Serial.begin(115200);
  Serial.println();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start UDP
  udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // Initialize NeoPixel strip
  strip.begin();
  strip.show();  // Turn off all pixels
  strip.setBrightness(100);  // Set brightness to about 1/5 (max = 255)
}

void refresh_strip(uint8_t* pixels, size_t length) {
  for (size_t i = 0; i < LED_COUNT; i++) {
    if (i * 3 + 2 < length) {
      uint8_t b = pixels[i * 3];
      uint8_t g = pixels[i * 3 + 1];
      uint8_t r = pixels[i * 3 + 2];
      strip.setPixelColor(i, strip.Color(r, g, b));
    }
  }
  strip.show();
  delay(1);  // Delay for 50 ms
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet into the buffer
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0;  // Null-terminate the string
    }
    Serial.printf("Received packet: %s\n", incomingPacket);

    // Convert the incoming packet to an array of integers
    uint8_t data[LED_COUNT * 3];  // 200 pixels * 3 (r, g, b)
    int index = 0;
    char* token = strtok(incomingPacket, ",");
    while (token != NULL && index < sizeof(data)) {
      data[index++] = atoi(token);
      token = strtok(NULL, ",");
    }

    // Call the refresh_strip function
    refresh_strip(data, index);
  }
}