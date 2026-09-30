#define RH_HAVE_SERIAL
#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
#include <RHRouter.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "config.h" 

#include "mbedtls/gcm.h"

// ---------- Pins ----------
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

TinyGPSPlus gps;
Adafruit_BME280 bme;

// ---------- Timing & Cycle Constants ----------
static const uint32_t LISTEN_WINDOW_MS = 5000;   // 0s to 5s
static const uint32_t ACTIVE_WINDOW_MS = 36000;  // Total active window (36s)
static const uint32_t CYCLE_PERIOD_SEC = 3600;   // 1 Hour cycle

// RTC Memory to preserve sequence across deep sleep wakes
RTC_DATA_ATTR uint32_t globalTxSequence = 1; 

bool bootstrapFinished = false;
bool isRelayNode = false;        // True if node acts as a deterministic or dynamic relay

enum BootstrapMsgType {
    MSG_TYPE_ROOT_BEACON = 0xA1,
    MSG_TYPE_RELAY_ANNOUNCE = 0xA2,
    MSG_TYPE_APP_DATA = 0xA3
};

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

// Prototypes
static bool init_lora();
static bool init_sensors();
static void read_gps();
static void read_environment();
static void fill_routing_fields();
static void handle_incoming_packets();
static void enter_deep_sleep(uint32_t activeDurationMs);

// ================== Setup ====================
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial2.begin(9600, SERIAL_8N1, 27, 33);
    Wire.begin(I2C_SDA, I2C_SCL);
    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, SPI_CS);

    if (!init_lora()) {
        Serial.println(F("LoRa bring-up failed."));
    }
    if (!init_sensors()) {
        Serial.println(F("Sensor init warning."));
    }

    uint32_t cycleStartTime = millis();

    // Root Node Initialization
    if (nodeId == ROOT_NODE_ID) {
        Serial.println(F("[BOOT] Root node active. Broadcasting root beacon..."));
        uint8_t beaconPayload[2] = { MSG_TYPE_ROOT_BEACON, (uint8_t)nodeId };
        manager->sendto(beaconPayload, sizeof(beaconPayload), RH_BROADCAST_ADDRESS);
        isRelayNode = true; // Root acts as sink/relay throughout the window
    } else {
        // ================= Phase 1: Listen Window (0 - 5 seconds) =================
        Serial.println(F("[BOOT] Entering 5s Listen Phase..."));
        while (millis() - cycleStartTime < LISTEN_WINDOW_MS) {
            while (Serial2.available() > 0) {
                gps.encode(Serial2.read());
            }

            uint8_t buf[50];
            uint8_t len = sizeof(buf);
            uint8_t from = 0;

            if (manager->recvfromAckTimeout(buf, &len, 100, &from)) {
                if (len >= 2) {
                    uint8_t msgType = buf[0];
                    uint8_t originId = buf[1];

                    // Heard Root Directly -> Deterministic Route + Become Relay
                    if (msgType == MSG_TYPE_ROOT_BEACON) {
                        Serial.print(F("[BOOT] Heard Root directly from ")); Serial.println(originId);
                        manager->addRouteTo(ROOT_NODE_ID, ROOT_NODE_ID);
                        
                        uint8_t relayAnnounce[2] = { MSG_TYPE_RELAY_ANNOUNCE, (uint8_t)nodeId };
                        manager->sendto(relayAnnounce, sizeof(relayAnnounce), RH_BROADCAST_ADDRESS);
                        
                        isRelayNode = true;
                        bootstrapFinished = true;
                        break;
                    }
                    // Heard a Relay -> Deterministic Route via Relay
                    else if (msgType == MSG_TYPE_RELAY_ANNOUNCE) {
                        Serial.print(F("[BOOT] Heard Relay from ")); Serial.println(originId);
                        manager->addRouteTo(ROOT_NODE_ID, originId);
                        
                        isRelayNode = false; // Leaf node utilizing relay
                        bootstrapFinished = true;
                        break;
                    }
                }
            }
        }

        // Fallback: If 5s window expired without hearing root/relay, use Dynamic Routing
        if (!bootstrapFinished) {
            Serial.println(F("[BOOT] Listen timeout. Falling back to dynamic routing."));
            isRelayNode = false; // Standard leaf node using dynamic routing
            bootstrapFinished = true;
        }
    }

    // ================= Phase 2: Transmission & Relay Window (5s - 36s) =================
    Serial.println(F("[TX] Entering Transmission & Forwarding Window..."));
    
    // Non-root nodes send their telemetry update in this window
    if (nodeId != ROOT_NODE_ID) {
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
            Serial.println(F("[TX] Payload Sent OK"));
        } else {
            Serial.print(F("[TX] Dynamic Routing Fallback Error: ")); Serial.println(err);
        }

        // Rule Check: If node is NOT a relay, sleep immediately after sending update!
        if (!isRelayNode) {
            Serial.println(F("[SLEEP] Leaf node finished TX. Going to deep sleep immediately."));
            enter_deep_sleep(millis() - cycleStartTime);
            return;
        }
    }

    // ================= Phase 3: Relay/Sink Active Wait =================
    // Relays and Root stay awake until the 36-second window elapses to handle forwarding
    while (millis() - cycleStartTime < ACTIVE_WINDOW_MS) {
        handle_incoming_packets();
    }

    Serial.println(F("[SLEEP] Active 36s window closed. Relays entering deep sleep."));
    enter_deep_sleep(millis() - cycleStartTime);
}

void loop() {
    // Empty: Execution handles the 1-hour cycle inside setup() via deep sleep wakeups
}

// =============== Helper Functions ===============

static void enter_deep_sleep(uint32_t activeDurationMs) {
    uint32_t sleepDurationSec = CYCLE_PERIOD_SEC;
    if (CYCLE_PERIOD_SEC * 1000UL > activeDurationMs) {
        sleepDurationSec = (CYCLE_PERIOD_SEC * 1000UL - activeDurationMs) / 1000;
    }
    
    esp_sleep_enable_timer_wakeup((uint64_t)sleepDurationSec * 1000000ULL);
    esp_deep_sleep_start();
}

static void handle_incoming_packets() {
    uint8_t buf[TOTAL_PACKET_SIZE];
    uint8_t len = sizeof(buf);
    uint8_t from = 0;

    if (manager->recvfromAckTimeout(buf, &len, 100, &from)) {
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

            if (auth_status != 0) return;

            uint8_t sender = p.node_id;
            if (sender < MAX_MESH_NODES) {
                if (p.sequence_num <= nodeSequenceTracker[sender]) return;
                nodeSequenceTracker[sender] = p.sequence_num;
            }

            Serial.print(F("[RX/RELAY] Verified packet from Node ")); Serial.println(p.node_id);
        }
    }
}

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