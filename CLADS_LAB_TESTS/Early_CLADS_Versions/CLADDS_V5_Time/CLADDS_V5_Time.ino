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

// RTC Memory to preserve sequence and time offset tracking across deep sleep
RTC_DATA_ATTR uint32_t globalTxSequence = 1; 
RTC_DATA_ATTR int32_t timeDriftOffsetMs = 0;   // Accumulated clock drift correction

bool bootstrapFinished = false;
bool isRelayNode = false;        
bool gotSyncFromRoot = false;

// Custom Message Types
enum BootstrapMsgType {
    MSG_TYPE_ROOT_BEACON = 0xA1,
    MSG_TYPE_RELAY_ANNOUNCE = 0xA2,
    MSG_TYPE_APP_DATA = 0xA3,
    MSG_TYPE_SYNC_REPLY = 0xA4       // App-level sync feedback for dynamic routes
};

// Bootstrap / Sync Packet Structure
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t origin_id;
    uint32_t root_relative_time_ms;
} SyncPacket;

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

    if (!init_lora()) Serial.println(F("LoRa bring-up failed."));
    if (!init_sensors()) Serial.println(F("Sensor init warning."));

    uint32_t cycleStartTime = millis();

   // ================= Root Node Logic =================
    if (nodeId == ROOT_NODE_ID) {
        Serial.println(F("[BOOT] Root node active. Entering continuous awake mode..."));
        uint32_t lastBeaconTime = 0;
        
        while (true) {
            // Broadcast hourly sync beacon at the start of each hour
            if (millis() - lastBeaconTime >= (uint64_t)CYCLE_PERIOD_SEC * 1000UL || lastBeaconTime == 0) {
                Serial.println(F("[ROOT] Broadcasting hourly sync beacon..."));
                for (int i = 0; i <= 4; i++) {
                    SyncPacket beacon;
                    beacon.msg_type = MSG_TYPE_ROOT_BEACON;
                    beacon.origin_id = (uint8_t)nodeId;
                    beacon.root_relative_time_ms = millis();

                    // Fixed: using sendtoWait for proper mesh header formatting
                    manager->sendtoWait((uint8_t*)&beacon, sizeof(beacon), RH_BROADCAST_ADDRESS);
                    isRelayNode = true;
                    delay(400);
                }
                lastBeaconTime = millis();
            }
            
            // Root always stays awake and continuously handles incoming packets
            handle_incoming_packets();
            yield();
        }
    } 
    // ================= Non-Root Node Logic =================
    else {
        // Phase 1: Full 5s Listen Window (0 - 5 seconds)
        Serial.println(F("[BOOT] Entering full 5s Listen Phase..."));
        while (millis() - cycleStartTime < LISTEN_WINDOW_MS) {
            while (Serial2.available() > 0) {
                gps.encode(Serial2.read());
            }

            SyncPacket rxPkt;
            uint8_t len = sizeof(rxPkt);
            uint8_t from = 0;

            if (manager->recvfromAckTimeout((uint8_t*)&rxPkt, &len, 100, &from)) {
                if (len == sizeof(SyncPacket)) {
                    // Calculate clock drift offset based on root's relative clock
                    uint32_t localTime = millis();
                    timeDriftOffsetMs = (int32_t)rxPkt.root_relative_time_ms - (int32_t)localTime;
                    gotSyncFromRoot = true;

                    // Case A: Heard Root Directly -> Deterministic Route + Become Relay
                    if (rxPkt.msg_type == MSG_TYPE_ROOT_BEACON && !bootstrapFinished) {
                        Serial.print(F("[BOOT] Heard Root directly. Drift offset: ")); Serial.println(timeDriftOffsetMs);
                        manager->addRouteTo(ROOT_NODE_ID, ROOT_NODE_ID);
                        
                        SyncPacket relayAnnounce;
                        relayAnnounce.msg_type = MSG_TYPE_RELAY_ANNOUNCE;
                        relayAnnounce.origin_id = (uint8_t)nodeId;
                        relayAnnounce.root_relative_time_ms = rxPkt.root_relative_time_ms; // Propagate time
                        
                        manager->sendtoWait((uint8_t*)&relayAnnounce, sizeof(relayAnnounce), RH_BROADCAST_ADDRESS);
                        
                        isRelayNode = true;
                        bootstrapFinished = true;
                        // Note: Removed early break so it continues listening for the remainder of the 5s window
                    }
                    // Case B: Heard a Relay -> Deterministic Route via Relay
                    else if (rxPkt.msg_type == MSG_TYPE_RELAY_ANNOUNCE && !bootstrapFinished) {
                        Serial.print(F("[BOOT] Heard Relay via Node ")); Serial.println(rxPkt.origin_id);
                        manager->addRouteTo(ROOT_NODE_ID, rxPkt.origin_id);
                        
                        isRelayNode = false; 
                        bootstrapFinished = true;
                        // Note: Removed early break so it continues listening for the remainder of the 5s window
                    }
                }
            }
        }

        // Fallback: If 5s window expired without sync, use Dynamic Routing
        if (!bootstrapFinished) {
            Serial.println(F("[BOOT] Listen timeout. Falling back to dynamic routing."));
            isRelayNode = false; 
            bootstrapFinished = true;
        }

        // Phase 2: Transmission Window (5s - 36s)
        theData.sequence_num = globalTxSequence++;
        read_gps();
        read_environment();
        fill_routing_fields();

        unsigned char radio_packet[TOTAL_PACKET_SIZE];
        unsigned char iv[IV_SIZE];
        unsigned char tag[TAG_SIZE];

        for(int i = 0; i < IV_SIZE; i++) iv[i] = (unsigned char)esp_random();
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
            Serial.print(F("[TX] Dynamic Routing Error: ")); Serial.println(err);
        }

        // If we used dynamic routing and missed initial sync, listen for app-level sync reply (1-2s window)
        if (!gotSyncFromRoot) {
            uint32_t replyWaitStart = millis();
            while (millis() - replyWaitStart < 2000) {
                uint8_t replyBuf[30];
                uint8_t replyLen = sizeof(replyBuf);
                uint8_t replyFrom = 0;
                if (manager->recvfromAckTimeout(replyBuf, &replyLen, 100, &replyFrom)) {
                    if (replyBuf[0] == MSG_TYPE_SYNC_REPLY) {
                        uint32_t rootTime;
                        memcpy(&rootTime, replyBuf + 1, sizeof(rootTime));
                        timeDriftOffsetMs = (int32_t)rootTime - (int32_t)millis();
                        Serial.println(F("[SYNC] App-level sync reply received from Root!"));
                        break;
                    }
                }
            }
        }

        // Rule Check: If node is NOT a relay, sleep immediately!
        if (!isRelayNode) {
            Serial.println(F("[SLEEP] Leaf node finished. Going to deep sleep."));
            enter_deep_sleep(millis() - cycleStartTime);
            return;
        }

        // Phase 3: Relay Active Wait until 36s
        while (millis() - cycleStartTime < ACTIVE_WINDOW_MS) {
            handle_incoming_packets();
        }

        Serial.println(F("[SLEEP] Active 36s window closed. Relay entering deep sleep."));
        enter_deep_sleep(millis() - cycleStartTime);
    }
}

void loop() {}

// =============== Helper Functions ===============

static void enter_deep_sleep(uint32_t activeDurationMs) {
    int32_t targetSleepSec = CYCLE_PERIOD_SEC - (activeDurationMs / 1000);
    targetSleepSec += (timeDriftOffsetMs / 1000);
    if (targetSleepSec < 10) targetSleepSec = 10; 

    Serial.print(F("[SLEEP] Entering deep sleep for adjusted seconds: ")); 
    Serial.println(targetSleepSec);
    
    esp_sleep_enable_timer_wakeup((uint64_t)targetSleepSec * 1000000ULL);
    esp_deep_sleep_start();
}

static void handle_incoming_packets() {
    uint8_t buf[TOTAL_PACKET_SIZE];
    uint8_t len = sizeof(buf);
    uint8_t from = 0;

    if (manager->recvfromAckTimeout(buf, &len, 100, &from)) {
        if (buf[0] == MSG_TYPE_ROOT_BEACON || buf[0] == MSG_TYPE_RELAY_ANNOUNCE) return; 

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

            Serial.print(F("[RX/RELAY] Verified packet from Node ")); Serial.print(p.node_id);
            Serial.print(F(" :SeqID: ")); Serial.print(p.sequence_num);
            Serial.print(F(" :Hop: ")); Serial.println(from);

            if (nodeId == ROOT_NODE_ID && p.node_id != ROOT_NODE_ID) {
                uint8_t syncReply[5];
                syncReply[0] = MSG_TYPE_SYNC_REPLY;
                uint32_t currentRootTime = millis();
                memcpy(syncReply + 1, &currentRootTime, sizeof(currentRootTime));
                
                manager->sendto(syncReply, sizeof(syncReply), p.node_id);
            }
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