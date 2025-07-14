/*
 * ESP32 Local NTP Server using LAN8720 Ethernet
 *
 * This sketch turns an ESP32 into a dedicated, high-availability NTP server
 * for a local network.
 *
 * How it works:
 * 1.  Connects to the local network via an LAN8720 Ethernet board with a static IP address.
 * 2.  Synchronizes its own internal clock with a public, high-stratum NTP server (e.g., time.apple.com).
 * 3.  Listens for incoming NTP requests from other devices on the local network.
 * 4.  Responds to those requests, allowing other devices to sync their clocks to it.
 * 5.  Periodically re-syncs its own clock with the public server to maintain accuracy.
 *
 * Author: Pratham Virani
 * Date: July 14, 2025
 */

// =============================================================================
// LIBRARIES
// =============================================================================
#include <ETH.h>
#include <WiFi.h> // Required for WiFiUdp and hostByName
#include <WiFiUdp.h>
#include <time.h>

// =============================================================================
// ETHERNET & NETWORK CONFIGURATION - (USER MUST CONFIGURE THIS SECTION)
// =============================================================================

// -- Ethernet PHY Configuration --
// Set the type of Ethernet Physical Layer (PHY) chip you are using.
#define ETH_PHY_TYPE ETH_PHY_LAN8720
// Set the I2C address of the PHY. For LAN8720, this is usually 0 or 1.
// Check your board's documentation.
#define ETH_PHY_ADDR 1
// Set the pin connected to the MDC (Management Data Clock) line of the PHY.
#define ETH_PHY_MDC 23
// Set the pin connected to the MDIO (Management Data Input/Output) line of the PHY.
#define ETH_PHY_MDIO 18
// Set the pin that provides power to the PHY. -1 if not used.
#define ETH_PHY_POWER -1
// Set the clock mode for the RMII interface.
// ETH_CLOCK_GPIO17_OUT is a robust option where the ESP32 generates the 50MHz clock
// itself and outputs it on GPIO17. This avoids issues with the GPIO0 strapping pin.
// WIRING: You must connect ESP32's GPIO17 to the LAN8720's clock input pin
// (often labeled XTAL1 or CLKIN). Ensure the LAN8720's onboard oscillator is
// disabled or disconnected if your board allows it.
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

// -- Static IP Address Configuration --
// Set a static IP address for your ESP32 NTP server.
// This is crucial so your other devices always know where to find it.
// MAKE SURE this IP is available on your network and is outside your router's DHCP range.
IPAddress local_IP(192, 168, 1, 100);
// Set your network's gateway IP address (usually your router's IP).
IPAddress gateway(192, 168, 1, 1);
// Set your network's subnet mask.
IPAddress subnet(255, 255, 255, 0);
// Set your primary DNS server (can also be your router's IP).
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(1, 1, 1, 1);


// =============================================================================
// NTP CONFIGURATION
// =============================================================================

// -- Public NTP Server for Self-Syncing --
// This is the server your ESP32 will use to get its own time.
const char* ntpServer = "time.apple.com";

// -- Timezone Configuration for Serial Monitor Display --
// This is ONLY for displaying local time on the Serial Monitor for debugging.
// The NTP server itself operates and serves time in UTC.
// Set your timezone string here. Find yours at:
// https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
const char* tzInfo = "MYT-8"; // Malaysia Time

// -- NTP Server Port --
const int NTP_PORT = 123;
// -- NTP Packet Buffer --
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

// -- Self-Sync Interval --
// How often the ESP32 re-syncs with the public NTP server (in milliseconds).
// Changed to 10 minutes for testing purposes.
const long selfSyncInterval = 4 * 60 * 60 * 1000; // 4 Hours
unsigned long previousSelfSyncTime = 0;

// UDP object for listening for and sending NTP packets
WiFiUDP udp;

// Flag to track Ethernet connection status
static bool eth_connected = false;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void WiFiEvent(WiFiEvent_t event);
void handleNtpRequest();
void syncWithPublicNtp();
void printLocalTime();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nESP32 Local NTP Server - Initializing...");

  // Register the event handler for Ethernet events
  WiFi.onEvent(WiFiEvent);

  // Initialize Ethernet first to create the network interface
  Serial.println("Starting Ethernet hardware...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);

  // Now, configure the static IP on the created interface
  Serial.println("Configuring static IP for Ethernet...");
  if (!ETH.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Failed to configure Ethernet with static IP. Halting.");
    while (true);
  }

  // Wait for the Ethernet connection to be established
  unsigned long startTime = millis();
  while (!eth_connected && millis() - startTime < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (!eth_connected) {
    Serial.println("Ethernet connection failed! Please check your hardware and wiring.");
    while (true); // Halt on failure
  }

  // **NEW**: Add a DNS test to verify internet connectivity
  delay(60000);
  Serial.print("Testing DNS resolution for ");
  Serial.print(ntpServer);
  Serial.println("...");
  IPAddress resolvedIP;
  if (!WiFi.hostByName(ntpServer, resolvedIP)) {
    Serial.println("DNS lookup FAILED. ESP32 cannot contact the internet.");
    Serial.println("Please check your static IP, gateway, and DNS settings.");
    while(true); // Halt on failure
  }
  Serial.print("DNS lookup successful. ");
  Serial.print(ntpServer);
  Serial.print(" resolved to ");
  Serial.println(resolvedIP);


  // Start the UDP server on the NTP port
  Serial.println("Starting UDP listener on port " + String(NTP_PORT));
  udp.begin(NTP_PORT);

  // Initial time synchronization with the public server on every boot
  syncWithPublicNtp();
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  // Task 1: Handle incoming NTP requests from local devices (always running)
  handleNtpRequest();

  // Task 2: Periodically re-sync this server's own clock (background task)
  unsigned long currentMillis = millis();
  if (currentMillis - previousSelfSyncTime >= selfSyncInterval) {
    previousSelfSyncTime = currentMillis;
    Serial.println("\nPeriodic self-sync triggered.");
    syncWithPublicNtp();
  }
}

// =============================================================================
// CORE FUNCTIONS
// =============================================================================

/**
 * @brief Handles incoming NTP requests from clients on the local network.
 */
void handleNtpRequest() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    if (packetSize == NTP_PACKET_SIZE) {
      Serial.print("Received NTP request from ");
      Serial.println(udp.remoteIP());

      // Read the incoming packet into the buffer
      udp.read(packetBuffer, NTP_PACKET_SIZE);

      // Get the current time from the ESP32's internal clock
      struct timeval tv;
      gettimeofday(&tv, NULL);

      // --- Construct the NTP response packet ---

      // Set the Leap Indicator, Version Number, and Mode
      // LI = 0 (no warning), VN = 4, Mode = 4 (server)
      packetBuffer[0] = 0b00100100;

      // Set Stratum to 2, as we are a secondary server
      packetBuffer[1] = 2;

      // Set Poll Interval
      packetBuffer[2] = 4; // 2^4 = 16 seconds (a common value)

      // Set Precision
      packetBuffer[3] = 0xEC; // -20 (represents ~1 microsecond precision)

      // Timestamps are in NTP format (seconds since Jan 1, 1900).
      // The ESP32's time is in Unix format (seconds since Jan 1, 1970).
      // The difference is 2208988800 seconds.
      const uint32_t seventyYears = 2208988800UL;

      // Copy the client's transmit timestamp to the origin timestamp
      packetBuffer[24] = packetBuffer[40];
      packetBuffer[25] = packetBuffer[41];
      packetBuffer[26] = packetBuffer[42];
      packetBuffer[27] = packetBuffer[43];
      packetBuffer[28] = packetBuffer[44];
      packetBuffer[29] = packetBuffer[45];
      packetBuffer[30] = packetBuffer[46];
      packetBuffer[31] = packetBuffer[47];

      // Set the Receive Timestamp (when we received the request)
      uint32_t receiveTimestamp_s = tv.tv_sec + seventyYears;
      uint32_t receiveTimestamp_f = (uint32_t)((tv.tv_usec / 1000000.0) * 4294967296.0); // Fraction part
      packetBuffer[32] = (byte)(receiveTimestamp_s >> 24);
      packetBuffer[33] = (byte)(receiveTimestamp_s >> 16);
      packetBuffer[34] = (byte)(receiveTimestamp_s >> 8);
      packetBuffer[35] = (byte)(receiveTimestamp_s);
      packetBuffer[36] = (byte)(receiveTimestamp_f >> 24);
      packetBuffer[37] = (byte)(receiveTimestamp_f >> 16);
      packetBuffer[38] = (byte)(receiveTimestamp_f >> 8);
      packetBuffer[39] = (byte)(receiveTimestamp_f);
      
      // Set the Transmit Timestamp (the same as receive for simplicity and speed)
      packetBuffer[40] = packetBuffer[32];
      packetBuffer[41] = packetBuffer[33];
      packetBuffer[42] = packetBuffer[34];
      packetBuffer[43] = packetBuffer[35];
      packetBuffer[44] = packetBuffer[36];
      packetBuffer[45] = packetBuffer[37];
      packetBuffer[46] = packetBuffer[38];
      packetBuffer[47] = packetBuffer[39];

      // Send the response packet back to the client
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write(packetBuffer, NTP_PACKET_SIZE);
      udp.endPacket();
    } else {
        Serial.print("Received invalid packet of size ");
        Serial.println(packetSize);
    }
  }
}

/**
 * @brief Synchronizes the ESP32's internal clock with a public NTP server.
 */
void syncWithPublicNtp() {
  Serial.println("Contacting public NTP server to sync time...");
  // Configure time with timezone and public NTP server
  configTime(0, 0, ntpServer);
  setenv("TZ", tzInfo, 1);
  tzset();

  // Wait for time to be set
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10000)) { // 10-second timeout
    Serial.println("Failed to obtain time from public server.");
  } else {
    Serial.println("Time successfully synchronized with public server.");
    printLocalTime();
  }
}

/**
 * @brief Prints the current local time to the Serial monitor for debugging.
 */
void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time for printing.");
    return;
  }
  Serial.print("Current local time: ");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}


/**
 * @brief Event handler for WiFi/Ethernet events.
 * @param event The event type.
 */
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-ntp-server");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("");
      Serial.println("--- ETH Got IP Address ---");
      Serial.print("  IPv4 Address:  ");
      Serial.println(ETH.localIP());
      Serial.print("  Subnet Mask:   ");
      Serial.println(ETH.subnetMask());
      Serial.print("  Gateway IP:    ");
      Serial.println(ETH.gatewayIP());
      Serial.print("  Primary DNS:   ");
      Serial.println(ETH.dnsIP(0));
      Serial.print("  MAC Address:   ");
      Serial.println(ETH.macAddress());
      Serial.print("  Link Speed:    ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      Serial.print("  Duplex Mode:   ");
      Serial.println(ETH.fullDuplex() ? "Full" : "Half");
      Serial.println("--------------------------");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}
