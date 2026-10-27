/*
  ESP32 Probe-Request Sniffer (use Arduino IDE)
  - Averages signal strength for up to MAX_DEVICES devices
  - Buffers CSV lines in RAM and flushes them to SPIFFS from loop()
  - Blinks blue LED each time a probe request is logged
  - Send "GET_CSV\n" over USB Serial to retrieve /probes.csv
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include <SPIFFS.h>

#define LED_PIN 2

#define MAX_DEVICES 5
#define CHANNEL_MIN 1
#define CHANNEL_MAX 13
#define CHANNEL_HOP_MS 200

#define PENDING_CAP 64
#define PENDING_LINE_MAX 128
#define SSID_MAX_LEN 32

#define CSV_PATH "/probes.csv"
#define FLUSH_INTERVAL_MS 2000
#define LED_BLINK_MS 50

#define RSSI_AT_1M (-30)   // expected RSSI 1 m from a phone
#define PATH_LOSS_N 3.0f   // avg for inside

struct Device {
  uint8_t mac[6];
  int avgRSSI;
  bool used;
};

Device devices[MAX_DEVICES];

int currentChannel = CHANNEL_MIN;
unsigned long lastHop = 0;
unsigned long lastFlush = 0;
bool fsReady = false;

// buffer shared between the wifi callback and loop()
char pending[PENDING_CAP][PENDING_LINE_MAX];
int pendingHead = 0;
int pendingCount = 0;
portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool packetDetected = false;
bool ledOn = false;
unsigned long ledOnSince = 0;

void macToHexStr(const uint8_t *mac, char *out, int outLen) {
  snprintf(out, outLen, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Log-distance path loss model - spend some time calibrating this if it's important to you
int estimateDistanceCM(int rssi) {
  float exponent = (float)(RSSI_AT_1M - rssi) / (10.0f * PATH_LOSS_N);
  float meters = powf(10.0f, exponent);
  return (int)(meters * 100.0f + 0.5f);
}

// Finds the next SSID tag at or after *pos, replacing characters that would break the CSV.
// On success, *pos is moved past that tag so the next call finds the following SSID.
bool extractSSID(const uint8_t *buf, int len, int *pos, char *ssidOut, int ssidOutLen) {
  int i = *pos;
  while (i + 2 <= len) {
    uint8_t tag = buf[i];
    uint8_t slen = buf[i + 1];
    if (i + 2 + slen > len) return false;
    if (tag == 0x00) {
      int copyLen = slen;
      if (copyLen > ssidOutLen - 1) copyLen = ssidOutLen - 1;
      for (int k = 0; k < copyLen; ++k) {
        uint8_t c = buf[i + 2 + k];
        if (c < ' ' || c > '~' || c == ',' || c == '"') c = '_';
        ssidOut[k] = c;
      }
      ssidOut[copyLen] = '\0';
      *pos = i + 2 + slen;
      return true;
    }
    i += 2 + slen;
  }
  return false;
}

// Checks if a MAC is being tracks and updates its avg RSSI, adds new MACs or returns NULL if array is full
Device* getDeviceOrAdd(const uint8_t *mac, int rssi) {
  for (int i = 0; i < MAX_DEVICES; ++i) {
    if (devices[i].used && memcmp(devices[i].mac, mac, 6) == 0) {
      devices[i].avgRSSI = (devices[i].avgRSSI + rssi) / 2;
      return &devices[i];
    }
  }
  for (int i = 0; i < MAX_DEVICES; ++i) {
    if (!devices[i].used) {
      devices[i].used = true;
      memcpy(devices[i].mac, mac, 6);
      devices[i].avgRSSI = rssi;
      return &devices[i];
    }
  }
  return NULL;
}

// Adds a CSV line to the end of the buffer
void enqueueLine(const char *line) {
  portENTER_CRITICAL(&pendingMux);
  if (pendingCount == PENDING_CAP) {
    pendingHead = (pendingHead + 1) % PENDING_CAP;
    pendingCount--;
  }
  int tail = (pendingHead + pendingCount) % PENDING_CAP;
  strlcpy(pending[tail], line, PENDING_LINE_MAX);
  pendingCount++;
  portEXIT_CRITICAL(&pendingMux);
}

// Copies the oldest CSV line from buffer into input var
bool dequeueLine(char *out) {
  portENTER_CRITICAL(&pendingMux);
  if (pendingCount == 0) {
    portEXIT_CRITICAL(&pendingMux);
    return false;
  }
  strlcpy(out, pending[pendingHead], PENDING_LINE_MAX);
  pendingHead = (pendingHead + 1) % PENDING_CAP;
  pendingCount--;
  portEXIT_CRITICAL(&pendingMux);
  return true;
}

// For each captured packet it logs one CSV line per SSID in every probe request
void snifferCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const uint8_t *payload = ppkt->payload;
  int len = (int)ppkt->rx_ctrl.sig_len - 4;  // Verifies length of packet
  if (len <= 24) return;

  uint8_t fc0 = payload[0];
  uint8_t frameType = (fc0 >> 2) & 0x03;
  uint8_t subtype = (fc0 >> 4) & 0x0F;
  if (frameType != 0 || subtype != 4) return;  // probe requests only

  const uint8_t *srcMac = payload + 10;  
  const uint8_t *tags = payload + 24;    
  int tagsLen = len - 24;
  int pos = 0;
  char ssid[SSID_MAX_LEN + 1];
  if (!extractSSID(tags, tagsLen, &pos, ssid, sizeof(ssid))) return;

  int rssi = ppkt->rx_ctrl.rssi;
  int avgRSSI = rssi;
  Device *d = getDeviceOrAdd(srcMac, rssi);
  if (d) avgRSSI = d->avgRSSI;

  char macStr[18];
  macToHexStr(srcMac, macStr, sizeof(macStr));
  unsigned long timestamp = millis();
  int distanceCM = estimateDistanceCM(avgRSSI);

  // One CSV line per SSID: timestamp_ms,channel,rssi,avg_rssi,distance_cm,mac,ssid
  do {
    char line[PENDING_LINE_MAX];
    int written = snprintf(line, sizeof(line), "%lu,%u,%d,%d,%d,%s,%s",
                           timestamp, (unsigned)ppkt->rx_ctrl.channel, rssi,
                           avgRSSI, distanceCM, macStr, ssid);
    if (written > 0) {
      enqueueLine(line);
      packetDetected = true;
    }
  } while (extractSSID(tags, tagsLen, &pos, ssid, sizeof(ssid)));
}

// Sends csv to SPIFFS incase of loss of power
void flushPendingToSPIFFS() {
  if (!fsReady || pendingCount == 0) return;
  File f = SPIFFS.open(CSV_PATH, FILE_APPEND);
  if (!f) return;
  char line[PENDING_LINE_MAX];
  while (dequeueLine(line)) f.println(line);
  f.close();
}

// Serves CSV to user
void sendCSV() {
  if (!fsReady) {
    Serial.println("SPIFFS not mounted");
    return;
  }
  flushPendingToSPIFFS();
  File f = SPIFFS.open(CSV_PATH, FILE_READ);
  if (!f) {
    Serial.println("No CSV file found");
    return;
  }
  uint8_t buf[256];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    Serial.write(buf, n);
  }
  f.close();
  Serial.println("---END---");
}

void setup() {
  Serial.begin(115200);
  delay(20);
  esp_log_level_set("wifi", ESP_LOG_NONE);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  fsReady = SPIFFS.begin(true);
  if (!fsReady) Serial.println("SPIFFS mount failed; logging disabled");

  WiFi.mode(WIFI_MODE_NULL);
  esp_wifi_stop();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();

  wifi_promiscuous_filter_t filt;
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&snifferCallback);
  esp_wifi_set_promiscuous(true);

  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  lastHop = millis();

  if (fsReady && !SPIFFS.exists(CSV_PATH)) {
    File f = SPIFFS.open(CSV_PATH, FILE_WRITE);
    if (f) {
      f.println("timestamp_ms,channel,rssi,avg_rssi,distance_cm,mac,ssid");
      f.close();
    }
  }

  Serial.printf("Sniffer started (max %d devices). Type GET_CSV and press Enter to download.\n", MAX_DEVICES);
}

void loop() {
  unsigned long now = millis();

  if (now - lastHop >= CHANNEL_HOP_MS) {
    lastHop = now;
    currentChannel++;
    if (currentChannel > CHANNEL_MAX) currentChannel = CHANNEL_MIN;
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  }

  if (now - lastFlush >= FLUSH_INTERVAL_MS) {
    lastFlush = now;
    flushPendingToSPIFFS();
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("GET_CSV")) sendCSV();
  }

  if (packetDetected) {
    packetDetected = false;
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
    ledOnSince = now;
  }
  if (ledOn && now - ledOnSince >= LED_BLINK_MS) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  }

  delay(1);
}