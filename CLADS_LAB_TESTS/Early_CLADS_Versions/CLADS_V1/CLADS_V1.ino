
#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
#include <RHRouter.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config.h"   // Must provide: nodeId, and ROOT_NODE_ID

// ---------- ESP32 Hardware Encryption Includes ----------
#include "mbedtls/gcm.h"

// ---------- Pins (Adjust for your specific board / Heltec v4 mapping) ----------
static const uint8_t RADIO_RST = 26;
static const uint8_t RADIO_IRQ = 34;  
static const uint8_t SPI_CS    = 25;
static const uint8_t SPI_CLK   = 5;
static const uint8_t SPI_MISO  = 19;
static const uint8_t SPI_MOSI  = 18;

static const int SOIL_PIN      = 32;  
static const uint8_t I2C_SDA   = 23;
static const uint8_t I2C_SCL   = 22;

// ---------- LoRa ----------
#define LORA_FREQ_MHZ 915.0
#define LORA_TX_POWER 20

RH_RF95 rf95(SPI_CS, RADIO_IRQ);
RHMesh* manager = nullptr;

// ---------- Sensors ----------
TinyGPSPlus gps;
Adafruit_BME280 bme;

// ---------- Timing & Bootstrap States ----------
static const uint32_t TRANSMIT_PERIOD_MS = 5000;
static const uint32_t BOOTSTRAP_TIMEOUT_MS = 6000;
uint32_t lastSend = 0;
uint32_t bootStartTime = 0;
bool bootstrapFinished = false;

// Custom Bootstrap Message Markers (Packet header types)
enum BootstrapMsgType {
    MSG_TYPE_ROOT_BEACON = 0xA1,
    MSG_TYPE_RELAY_ANNOUNCE = 0xA2,
    MSG_TYPE_APP_DATA = 0xA3
};

// ---------- Binary-safe payload ----------
typedef struct __attribute__((packed)) {
    uint32_t sequence_num;    
    float lat; 
    float lon; 
    float temperature; 
    float humidity; 
    float pressure; 
    uint16_t soil_moisture; 
    int16_t next_hop_rssi; 
    uint8_t node_id; 
    uint8_t next_hop; 
} Payload; 

static Payload theData;
uint32_t globalTxSequence = 1; 

// ---------- Crypto Settings ----------
const unsigned char aes_key[16] = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

#define PAYLOAD_SIZE sizeof(Payload)            
#define IV_SIZE 12                            
#define TAG_SIZE 16                           
#define TOTAL_PACKET_SIZE (IV_SIZE + PAYLOAD_SIZE + TAG_SIZE) 

#define MAX_MESH_NODES 10
uint32_t nodeSequenceTracker[MAX_MESH_NODES] = { 0 }; 

// ---------- Prototypes ----------
static bool init_lora();
static bool init_sensors();
static void read_gps();
static void read_environment();
static void fill_routing_fields();
static void handle_incoming_packets();
static void run_bootstrap_phase();

// ================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial2.begin(9600, SERIAL_8N1, 27, 33);
  Wire.begin(I2C_SDA, I2C_SCL);
  SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

  if (!init_lora()) {
    Serial.println(F("LoRa bring-up failed. Check wiring/pins/frequency."));
  }

  if (!init_sensors()) {
    Serial.println(F("Sensor init warning (continuing)."));
  }

  bootStartTime = millis();

  // If this node is the Root, broadcast the initial beacon immediately
  if (nodeId == ROOT_NODE_ID) {
      Serial.println(F("[BOOT] Root node active. Broadcasting root beacon..."));
      uint8_t beaconPayload[2] = { MSG_TYPE_ROOT_BEACON, (uint8_t)nodeId };
      manager->sendto(beaconPayload, sizeof(beaconPayload), RH_BROADCAST_ADDRESS);
  }
}

// ================== Loop =====================
void loop() {
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }

    // 1. Run Bootstrap Phase (Listen for Beacons/Relays or Fallback)
    if (!bootstrapFinished) {
        run_bootstrap_phase();
    } 
    // 2. Normal Operational Phase (Periodic telemetry + background forwarding)
    else {
        uint32_t now = millis();
        if (nodeId != ROOT_NODE_ID && (now - lastSend >= TRANSMIT_PERIOD_MS)) {
            lastSend = now;
            
            theData.sequence_num = globalTxSequence++;
            read_gps();
            read_environment();
            fill_routing_fields();

            unsigned char radio_packet[TOTAL_PACKET_SIZE];
            unsigned char iv[IV_SIZE];
            unsigned char tag[TAG_SIZE];

            for(int i = 0; i < IV_SIZE; i++) {
                iv[i] = (unsigned char)esp_random();
            }
            memcpy(radio_packet, iv, IV_SIZE); 

            mbedtls_gcm_context gcm;
            mbedtls_gcm_init(&gcm);
            mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, aes_key, 128);

            mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT, PAYLOAD_SIZE, 
                                      iv, IV_SIZE, NULL, 0, 
                                      (const unsigned char*)&theData, radio_packet + IV_SIZE, 
                                      TAG_SIZE, tag);

            memcpy(radio_packet + IV_SIZE + PAYLOAD_SIZE, tag, TAG_SIZE);
            mbedtls_gcm_free(&gcm);

            // Send payload via RHMesh. If custom bootstrap failed to find a route, 
            // RHMesh will seamlessly engage its built-in fallback route discovery here.
            uint8_t err = manager->sendtoWait(radio_packet, TOTAL_PACKET_SIZE, ROOT_NODE_ID);
            if (err == RH_ROUTER_ERROR_NONE) {
                Serial.println(F("[TX] Authenticated Payload Sent OK"));
            } else {
                Serial.print(F("[TX] Error/Fallback Discovery Triggered: ")); Serial.println(err);
            }
        }

        // Keep listening for application packets / forwarding data
        handle_incoming_packets();
    }
}

// =============== Custom Bootstrap Routine ===============
static void run_bootstrap_phase() {
    uint8_t buf[50];
    uint8_t len = sizeof(buf);
    uint8_t from = 0;

    // Check if bootstrap timeout window has elapsed
    if (millis() - bootStartTime > BOOTSTRAP_TIMEOUT_MS) {
        Serial.println(F("[BOOT] Bootstrap window expired. Transitioning to normal operation."));
        bootstrapFinished = true;
        return;
    }

    // Listen for incoming bootstrap broadcast packets (non-blocking short check)
    if (manager->recvfromAckTimeout(buf, &len, 200, &from)) {
        if (len >= 2) {
            uint8_t msgType = buf[0];
            uint8_t originId = buf[1];

            // Case A: Node hears the Root Beacon directly (1-Hop Neighbor)
            if (msgType == MSG_TYPE_ROOT_BEACON && nodeId != ROOT_NODE_ID) {
                Serial.print(F("[BOOT] Heard Root Beacon directly from Node ")); Serial.println(originId);
                
                // Manually add route: Root is reachable directly via Root
                manager->addRouteTo(ROOT_NODE_ID, ROOT_NODE_ID);
                
                // Rebroadcast our status as a newly minted relay node to 2-hop neighbours
                uint8_t relayAnnounce[2] = { MSG_TYPE_RELAY_ANNOUNCE, (uint8_t)nodeId };
                manager->sendto(relayAnnounce, sizeof(relayAnnounce), RH_BROADCAST_ADDRESS);
                
                bootstrapFinished = true;
            }
            // Case B: Node hears a Relay Announcement (2-Hop Neighbor)
            else if (msgType == MSG_TYPE_RELAY_ANNOUNCE && nodeId != ROOT_NODE_ID) {
                Serial.print(F("[BOOT] Heard Relay Announcement from Node ")); Serial.println(originId);
                
                // Manually add route: Root is reachable through this intermediate relay
                manager->addRouteTo(ROOT_NODE_ID, originId);
                
                bootstrapFinished = true;
            }
        }
    }
}

// =============== Packet Handler & Forwarder ===============
static void handle_incoming_packets() {
    uint8_t buf[TOTAL_PACKET_SIZE];
    uint8_t len = sizeof(buf);
    uint8_t from = 0;

    if (manager->recvfromAckTimeout(buf, &len, 100, &from)) {
        // Filter out bootstrap messages if any bleed into operational phase
        if (buf[0] == MSG_TYPE_ROOT_BEACON || buf[0] == MSG_TYPE_RELAY_ANNOUNCE) {
            return; 
        }

        if (len == TOTAL_PACKET_SIZE) {
            unsigned char iv_decrypt[IV_SIZE];
            unsigned char tag_decrypt[TAG_SIZE];
            Payload p; 

            memcpy(iv_decrypt, buf, IV_SIZE);
            memcpy(tag_decrypt, buf + IV_SIZE + PAYLOAD_SIZE, TAG_SIZE);

            mbedtls_gcm_context gcm;
            mbedtls_gcm_init(&gcm);
            mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, aes_key, 128);

            int auth_status = mbedtls_gcm_auth_decrypt(&gcm, PAYLOAD_SIZE, 
                                                       iv_decrypt, IV_SIZE, NULL, 0, 
                                                       tag_decrypt, TAG_SIZE, 
                                                       buf + IV_SIZE, (unsigned char*)&p);
            mbedtls_gcm_free(&gcm);

            if (auth_status != 0) {
                Serial.println(F("[RX] SECURITY ALERT: Cryptographic Authentication Failed!"));
                return; 
            }

            uint8_t sender = p.node_id;
            if (sender < MAX_MESH_NODES) {
                if (p.sequence_num <= nodeSequenceTracker[sender]) {
                    Serial.print(F("[RX] Replay detected from Node ")); Serial.println(sender);
                    return; 
                }
                nodeSequenceTracker[sender] = p.sequence_num;
            }

            Serial.print(F("[RX] Verified Packet via Hop ")); Serial.print(from);
            Serial.print(F(" (Node ")); Serial.print(p.node_id);
            Serial.print(F("), Seq ")); Serial.print(p.sequence_num);
            Serial.print(F(", Temp: ")); Serial.print(p.temperature, 2);
            Serial.println(F("C"));
        }
    }
}

// =============== Init helpers ===============
static bool init_lora() {
    pinMode(RADIO_RST, OUTPUT);
    digitalWrite(RADIO_RST, HIGH); delay(5);
    digitalWrite(RADIO_RST, LOW);  delay(10);
    digitalWrite(RADIO_RST, HIGH); delay(10);

    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

    if (!rf95.init()) return false;
    rf95.setFrequency(LORA_FREQ_MHZ);
    rf95.setTxPower(LORA_TX_POWER, false); 

    manager = new RHMesh(rf95, nodeId);
    if (!manager->init()) return false;
    manager->setMaxHops(3);

    return true;
}

static bool init_sensors() {
    bool ok = true;
    if (nodeId == ROOT_NODE_ID) return ok;

    if (!bme.begin(0x76)) ok = false;
    pinMode(SOIL_PIN, INPUT);
    analogReadResolution(12); 
    return ok;
}

static void read_gps() {
    if (gps.location.isValid()) {
        theData.lat = gps.location.lat();
        theData.lon = gps.location.lng();
    } else {
        theData.lat = 0.0f;
        theData.lon = 0.0f;
    }
}

static void read_environment() {

  if (!init_lora()) {
    Serial.println(F("LoRa bring-up failed. Check wiring/pins/frequency."));
  }

  if (!init_sensors()) {
    Serial.println(F("Sensor init warning (continuing)."));
  }

  bootStartTime = millis();


Not connected. Select a board and a port to connect automatically.
New Line

    if (nodeId == ROOT_NODE_ID) return;
    theData.temperature = bme.readTemperature();
    theData.humidity    = bme.readHumidity();
    theData.pressure    = bme.readPressure() / 100.0f; 
    int soil_raw = analogRead(SOIL_PIN);        
    theData.soil_moisture = (uint16_t)map(soil_raw, 4095, 0, 0, 100);
}

static void fill_routing_fields() {
    theData.node_id       = (uint8_t)nodeId;
    theData.next_hop_rssi = (int16_t)rf95.lastRssi();

    RHRouter::RoutingTableEntry* route = manager->getRouteTo(ROOT_NODE_ID);
    theData.next_hop = route ? route->next_hop : 0;
}