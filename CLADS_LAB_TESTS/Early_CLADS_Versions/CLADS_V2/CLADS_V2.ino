#define RH_HAVE_SERIAL
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

// ---------- Timing & State Machine Constants ----------
static const uint32_t BOOTSTRAP_WINDOW_MS = 5000;   // 0s - 5s: Listen phase
static const uint32_t ACTIVE_WINDOW_MS    = 36000;  // Total active window limit (36 seconds)
static const uint64_t HOUR_IN_US          = 3600ULL * 1000000ULL; // 1 Hour cycle

// State tracking variables
uint32_t wakeTime = 0;
bool bootstrapFinished = false;
bool dataSent = false;

// Node Role Classification
enum NodeRole {
    ROLE_UNKNOWN = 0,
    ROLE_DIRECT_RELAY,  // Heard root directly, announces itself as relay
    ROLE_REGULAR_NODE,  // Heard a relay, acts as leaf node
    ROLE_DYNAMIC_NODE   // Timed out bootstrap, uses dynamic routing fallback
};
NodeRole myRole = ROLE_UNKNOWN;

// Custom Bootstrap Message Markers
enum BootstrapMsgType {
    MSG_TYPE_ROOT_BEACON    = 0xA1,
    MSG_TYPE_RELAY_ANNOUNCE = 0xA2,
    MSG_TYPE_APP_DATA       = 0xA3
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

// Persistent sequence number across deep sleep cycles using RTC memory
RTC_DATA_ATTR uint32_t globalTxSequence = 1; 

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
static void send_telemetry();
static void enter_deep_sleep();

// ================== Setup ====================
void setup() {
    Serial.begin(115200);
    delay(200);

    wakeTime = millis();

    Serial2.begin(9600, SERIAL_8N1, 27, 33);
    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

    if (!init_lora()) {
        Serial.println(F("LoRa bring-up failed. Check wiring/pins/frequency."));
    }

    if (!init_sensors()) {
        Serial.println(F("Sensor init warning (continuing)."));
    }

    // If this node is the Root, broadcast the initial beacon immediately at T=0
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

    uint32_t elapsed = millis() - wakeTime;

    // --- PHASE 1: Listen & Bootstrap Window (0s to 5s) ---
    if (!bootstrapFinished && elapsed < BOOTSTRAP_WINDOW_MS) {
        run_bootstrap_phase();
        return; // Keep focus on bootstrap during this window
    }

    // If bootstrap window expired and node never heard anything, fallback to dynamic routing
    if (!bootstrapFinished && elapsed >= BOOTSTRAP_WINDOW_MS) {
        if (nodeId != ROOT_NODE_ID) {
            myRole = ROLE_DYNAMIC_NODE;
            Serial.println(F("[BOOT] Bootstrap window expired. Falling back to dynamic routing."));
        }
        bootstrapFinished = true;
    }

    // --- PHASE 2: Transmission & Relaying Window (5s to 36s) ---
    if (nodeId != ROOT_NODE_ID && !dataSent && elapsed >= BOOTSTRAP_WINDOW_MS) {
        send_telemetry();
        dataSent = true;

        // RULE: If node is a regular leaf node (not a relay/dynamic helper), sleep immediately after sending
        if (myRole == ROLE_REGULAR_NODE) {
            Serial.println(F("[POWER] Regular leaf node sent update. Entering immediate sleep."));
            enter_deep_sleep();
        }
    }

    // Continuously handle incoming packets and relay/forward duties during the active window
    handle_incoming_packets();

    // --- PHASE 3: Window Expiry & Sleep Management for Relays/Dynamic Nodes ---
    if (nodeId != ROOT_NODE_ID && elapsed >= ACTIVE_WINDOW_MS) {
        Serial.println(F("[POWER] 36-second active window closed. Relays/Forwarders entering sleep."));
        enter_deep_sleep();
    }

    // Root node stays awake indefinitely (acting as gateway/sink)
    if (nodeId == ROOT_NODE_ID) {
        // Root can perform background tasks here if needed
        delay(10); 
    }
}

// =============== Custom Bootstrap Routine (0s - 5s) ===============
static void run_bootstrap_phase() {
    uint8_t buf[50];
    uint8_t len = sizeof(buf);
    uint8_t from = 0;

    if (manager->recvfromAckTimeout(buf, &len, 50, &from)) {
        if (len >= 2) {
            uint8_t msgType = buf[0];
            uint8_t originId = buf[1];

            // Case A: Node hears the Root Beacon directly (Becomes a Direct Relay)
            if (msgType == MSG_TYPE_ROOT_BEACON && nodeId != ROOT_NODE_ID) {
                Serial.print(F("[BOOT] Heard Root Beacon directly from Node ")); Serial.println(originId);
                
                manager->addRouteTo(ROOT_NODE_ID, ROOT_NODE_ID);
                myRole = ROLE_DIRECT_RELAY;
                
                // Rebroadcast status as a relay node to 2-hop neighbors
                uint8_t relayAnnounce[2] = { MSG_TYPE_RELAY_ANNOUNCE, (uint8_t)nodeId };
                manager->sendto(relayAnnounce, sizeof(relayAnnounce), RH_BROADCAST_ADDRESS);
                
                bootstrapFinished = true;
            }
            // Case B: Node hears a Relay Announcement (Becomes a Regular Leaf Node)
            else if (msgType == MSG_TYPE_RELAY_ANNOUNCE && nodeId != ROOT_NODE_ID) {
                Serial.print(F("[BOOT] Heard Relay Announcement from Node ")); Serial.println(originId);
                
                manager->addRouteTo(ROOT_NODE_ID, originId);
                myRole = ROLE_REGULAR_NODE;
                
                bootstrapFinished = true;
            }
        }
    }
}

// =============== Send Telemetry Data ===============
static void send_telemetry() {
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

    uint8_t err = manager->sendtoWait(radio_packet, TOTAL_PACKET_SIZE, ROOT_NODE_ID);
    if (err == RH_ROUTER_ERROR_NONE) {
        Serial.println(F("[TX] Authenticated Payload Sent OK"));
    } else {
        Serial.print(F("[TX] Error/Fallback Discovery Triggered: ")); Serial.println(err);
    }
}

// =============== Packet Handler & Forwarder ===============
static void handle_incoming_packets() {
    uint8_t buf[TOTAL_PACKET_SIZE];
    uint8_t len = sizeof(buf);
    uint8_t from = 0;

    if (manager->recvfromAckTimeout(buf, &len, 50, &from)) {
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

// =============== Deep Sleep Handler ===============
static void enter_deep_sleep() {
    uint32_t activeDurationMs = millis() - wakeTime;
    uint64_t sleepDurationUs = 0;

    if (HOUR_IN_US > (uint64_t)activeDurationMs * 1000ULL) {
        sleepDurationUs = HOUR_IN_US - ((uint64_t)activeDurationMs * 1000ULL);
    } else {
        sleepDurationUs = 1000000ULL; // Fallback 1 second if active time exceeded
    }

    Serial.print(F("[SLEEP] Entering deep sleep for ~")); 
    Serial.print((uint32_t)(sleepDurationUs / 1000000ULL)); 
    Serial.println(F(" seconds."));
    Serial.flush();

    esp_sleep_enable_timer_wakeup(sleepDurationUs);
    esp_deep_sleep_start();
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