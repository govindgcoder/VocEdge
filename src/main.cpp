#include "WString.h"
#include "config.h"
#include "esp32-hal-gpio.h"
#include "mic.h"
#include "roboeyes.h"
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// wake-word inference (Edge Impulse "vocedge" model)
// Undef conflict: vocedge_inferencing.h undefs min/max/round etc.
#ifndef EIDSP_QUANTIZE_FILTERBANK
#define EIDSP_QUANTIZE_FILTERBANK 0 // save 10K RAM
#endif
#include <vocedge_inferencing.h>
#include <algorithm>
using std::max;
using std::min;

#include <vocedge_inferencing.h> // EI project

// for message
enum MsgType : uint8_t { MSG_AUDIO = 0x01, MSG_END = 0x02, MSG_TEXT = 0x03 };

constexpr int MAX_LINES = 64;      // max lines for wrapped text
constexpr int CHARS_PER_LINE = 10; // max chars before wrapping

constexpr uint16_t LOADING_SPEED = 300;      // dot animation speed
constexpr uint16_t TEXT_SCROLL_SPEED = 1200; // ms between scroll steps
constexpr uint32_t END_HOLD_DELAY = 2000;    // hold at end of text
constexpr uint32_t CALIBRATE_INTERVAL_MS = 120000; // recalibrate ambient noise in idle

uint8_t loadingDots = 0;        // current dot count (0-3)
uint32_t lastLoadingUpdate = 0; // last time dots were updated

String lines[MAX_LINES];     // wrapped text lines
uint8_t lineCount = 0;       // number of valid wrapped lines
int textScrollLine = 0;      // first visible line index
uint32_t lastTextScroll = 0; // last time text scrolled
String reply = "";           // response text to display
bool lightsOn = false;       // built-in LED toggle state

enum State { IDLE, FOCUS_STATE, RECORDING, SENDING, LOADING_STATE, SHOW };
State state = IDLE;
uint32_t stateStartTime = 0;

WiFiServer tcpServer(TCP_PORT, 1);
WiFiClient client;

constexpr uint16_t DISCOVERY_PORT = 8899; // UDP beacon so the PC can find us
WiFiUDP discoveryUdp;
uint32_t lastAnnounce = 0;

// Non-blocking WiFi manager state
bool wifi_connected = false;
bool wifi_connecting = false;
uint32_t wifi_connect_start = 0;
uint32_t wifi_retry_at = 0;
#define WIFI_CONNECT_TIMEOUT_MS 6000
#define WIFI_RETRY_DELAY_MS 4000

static uint8_t rx[2048];
static size_t rxlen = 0;

// Wake-word audio sliding window (matches the EI model: 1s @ 16kHz)
#define EI_WINDOW_SIZE EI_CLASSIFIER_RAW_SAMPLE_COUNT
static int16_t ei_buffer[EI_WINDOW_SIZE];      // last 1s of ambient audio
static size_t ei_buffer_ptr = 0;               // samples since last inference
#define EI_INFER_EVERY_SAMPLES 4000            // ~250ms of new audio per inference
#define WAKE_CLASS "wake"                      // label trained for the wake word
#define WAKE_THRESHOLD 0.80f                   // min confidence to wake up

static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&ei_buffer[offset], out_ptr, length);
  return 0;
}

// ============================================================
// Function declarations
// ============================================================
void changeState(State newState);
void loading();
void wrapText(const String &text);
bool show(const String &text);

// codec
size_t frame_build(uint8_t type, const uint8_t *p, size_t len, uint8_t *out,
                   size_t cap) {
  if (cap < len + 5)
    return 0;
  out[0] = type;
  out[1] = (len >> 24) & 0xFF;
  out[2] = (len >> 16) & 0xFF;
  out[3] = (len >> 8) & 0xFF;
  out[4] = len & 0xFF;
  if (len)
    memcpy(out + 5, p, len);
  return len + 5;
}
// returns bytes consumed if a full frame decoded, else 0
size_t frame_consume(const uint8_t *b, size_t n, uint8_t *type, uint8_t *out,
                     size_t cap) {
  if (n < 5)
    return 0;
  size_t len =
      ((size_t)b[1] << 24) | ((size_t)b[2] << 16) | ((size_t)b[3] << 8) | b[4];
  if (n < 5 + len)
    return 0;
  *type = b[0];
  if (cap >= len && len)
    memcpy(out, b + 5, len);
  return 5 + len;
}

// ============================================================
// WiFi helpers
// ============================================================
const char* wifi_status_str(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:      return "IDLE";
    case WL_NO_SSID_AVAIL:    return "NO_SSID_AVAIL (AP not found / wrong name, only 2.4GHz!)";
    case WL_CONNECTED:        return "CONNECTED";
    case WL_CONNECT_FAILED:   return "CONNECT_FAILED (wrong password?)";
    case WL_CONNECTION_LOST:  return "CONNECTION_LOST";
    case WL_DISCONNECTED:     return "DISCONNECTED";
    default:                  return "UNKNOWN";
  }
}

// Kick off a connection attempt without blocking
void start_wifi() {
  Serial.print("\n[+] Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifi_connecting = true;
  wifi_connected = false;
  wifi_connect_start = millis();
}

// Poll the connection state; never blocks, never spams dots
void update_wifi() {
  if (wifi_connected) {
    if (WiFi.status() == WL_CONNECTED) return;
    wifi_connected = false;
    wifi_connecting = false;
    wifi_retry_at = millis() + WIFI_RETRY_DELAY_MS;
    Serial.println("[!] WiFi connection lost. Will retry.");
    if (client) {
      client.stop();
      rxlen = 0;
    }
    return;
  }

  if (!wifi_connecting) {
    if (millis() >= wifi_retry_at) start_wifi();
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    wifi_connecting = false;
    Serial.printf("\n[+] WiFi Connected! ESP32 IP: %s  Port: %d\n",
                  WiFi.localIP().toString().c_str(), TCP_PORT);
    if (state == IDLE) {
      display.clearDisplay();
      display.setCursor(10, 25);
      display.print("Connected!");
      display.display();
    }
    tcpServer.begin();
  } else if (millis() - wifi_connect_start > WIFI_CONNECT_TIMEOUT_MS) {
    wifi_connecting = false;
    wifi_retry_at = millis() + WIFI_RETRY_DELAY_MS;
    Serial.printf("[!] WiFi connect TIMEOUT! status=%d (%s). Retrying in %d ms.\n",
                  WiFi.status(), wifi_status_str(WiFi.status()), WIFI_RETRY_DELAY_MS);
  }
}

// Broadcast "VOCEDGE:<ip>" over UDP so the PC can auto-discover us
void announce_ip() {
  String ip = WiFi.localIP().toString();
  String msg = "VOCEDGE:" + ip;
  discoveryUdp.beginPacket(WiFi.broadcastIP(), DISCOVERY_PORT);
  discoveryUdp.write((const uint8_t *)msg.c_str(), msg.length());
  discoveryUdp.endPacket();
  Serial.printf("[UDP] Announced %s\n", ip.c_str());
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  setup_mic();
  Wire.begin();          // init I2C
  Wire.setClock(400000); // fast mode for OLED
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 25);
  display.print("Connecting Wi-Fi...");
  display.display();

  start_wifi(); // non-blocking: the eyes/mic run immediately, WiFi connects in background

  delay(1000);
  pinMode(0, INPUT_PULLUP);       // Ensure BOOT button is set as input
  randomSeed(analogRead(A0));     // seed RNG from floating pin
  nextMovement = millis() + 1000; // first eye movement in 1s

  calibrate_mic();

  run_classifier_init(); // init Edge Impulse wake-word model
  memset(ei_buffer, 0, sizeof(ei_buffer));
  Serial.println("Wake-word engine ready.");

  Serial.println("System Ready. Waiting for button press...");
  delay(500);
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  static uint32_t last_btn_press = 0;

  update_wifi(); // poll connectivity - never blocks

  // periodically adapt the noise floor to the room while idle
  if (state == IDLE) {
    static uint32_t last_calibrate = 0;
    if (millis() - last_calibrate >= CALIBRATE_INTERVAL_MS) {
      last_calibrate = millis();
      calibrate_mic();
    }
  }

  // UDP beacon every 3s so the PC can discover us without a fixed IP
  if (wifi_connected && millis() - lastAnnounce > 3000) {
    lastAnnounce = millis();
    announce_ip();
  }

  if (digitalRead(0) == LOW && millis() - last_btn_press > 1000) {
    last_btn_press = millis();

    if (state == LOADING_STATE || state == SHOW || state == SENDING) {
      Serial.println("\n[!] Manual Override: Aborting to IDLE");
      changeState(IDLE);
      return; // Skip the rest of the loop this frame
    }
    // Otherwise, normal trigger
    else if (state == IDLE) {
      changeState(FOCUS_STATE);
    }
  }

  // accept a pending client
  if (!client || !client.connected()) {
    client = tcpServer.available();
  }

  // cleanly handle sudden Python disconnects
  if (client && !client.connected()) {
    client.stop();
    Serial.println("\n[!] Python client disconnected. Waiting for reconnect...");
  }

  // block-read the TCP stream (only when WiFi is actually up)
  if (wifi_connected && client && client.connected()) {
    while (client.available() > 0) {
      int to_read = client.available();
      if (rxlen + to_read > sizeof(rx)) {
        to_read = sizeof(rx) - rxlen;
      }

      if (to_read > 0) {
        rxlen += client.read(rx + rxlen, to_read);
      } else {
        client.read(); // buffer full: drop excess byte to prevent infinite loop
      }

      uint8_t type;
      uint8_t pay[1025]; // max text size
      size_t used = frame_consume(rx, rxlen, &type, pay, sizeof(pay) - 1);
      if (used) {
        size_t payload_len = used - 5;
        rxlen -= used;
        memmove(rx, rx + used, rxlen);
        if (type == MSG_TEXT) {
          // Safely null-terminate the string
          if (payload_len < sizeof(pay)) {
            pay[payload_len] = '\0';
          } else {
            pay[sizeof(pay) - 1] = '\0';
          }

          // --- JSON ACTION DISPATCH ---
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, pay);

          if (!error) {
            String action = doc["action"].as<String>();
            String value = doc["value"].as<String>();

            if (action == "lights") {
              // Toggle instead of trusting the STT/LLM value — prevents an
              // "already on/off" desync like "It's off." -> value "1"
              lightsOn = !lightsOn;

              // Built-in NeoPixel command for ESP32-S3 (Pin, R, G, B)
              // If it's a standard LED instead, change this to digitalWrite(48, lightsOn);
              neopixelWrite(48, lightsOn ? 255 : 0, lightsOn ? 255 : 0, lightsOn ? 255 : 0);

              reply = lightsOn ? "Lights ON!" : "Lights OFF";
            } else {
              // Standard conversation reply
              reply = value;
            }
          } else {
            reply = "JSON Error";
            Serial.println("Failed to parse JSON from Python!");
          }
          // ------------------------------

          changeState(SHOW);
          calibrate_mic();
        }
      }
    }
  }

  switch (state) {
  case IDLE:
    idle(); // keeps the eyes wandering

    // Wake-word: grab ambient audio and feed the Edge Impulse classifier
    {
      size_t bytes_read = record();
      int new_samples = bytes_read / 2;

      if (new_samples > 0 && (size_t)new_samples < EI_WINDOW_SIZE) {
        // Slide the old audio out, append the new audio in
        memmove(ei_buffer, ei_buffer + new_samples,
                (EI_WINDOW_SIZE - new_samples) * sizeof(int16_t));
        memcpy(ei_buffer + (EI_WINDOW_SIZE - new_samples), sample_buffer,
               bytes_read);
        ei_buffer_ptr += new_samples;

        // Run inference whenever we've accumulated ~250ms of new audio
        if (ei_buffer_ptr >= EI_INFER_EVERY_SAMPLES) {
          ei_buffer_ptr = 0; // reset counter for the next chunk

          signal_t signal;
          signal.total_length = EI_WINDOW_SIZE;
          signal.get_data = &raw_feature_get_data;

          ei_impulse_result_t result = {0};
          EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

          if (res == EI_IMPULSE_OK) {
            for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
              if (strcmp(result.classification[i].label, WAKE_CLASS) == 0) {
                // Print the confidence level for debugging
                Serial.printf("Wake Word Confidence: %.2f\n",
                              result.classification[i].value);

                // If confidence is over the threshold, WAKE UP!
                if (result.classification[i].value > WAKE_THRESHOLD) {
                  Serial.println("\n[!] WAKE WORD DETECTED!");
                  changeState(FOCUS_STATE);
                  memset(ei_buffer, 0, sizeof(ei_buffer)); // prevent double-triggers
                }
              }
            }
          }
        }
      }
    }
    break;
  case FOCUS_STATE:
    if (focus()) {
      Serial.println(
          "Focus finished! Moving to RECORDING..."); // <-- Breadcrumb!
      changeState(RECORDING);
      reset_silence();
    }
    break;

  case RECORDING: {
    size_t bytes_read = record();

    static uint8_t fb[2048 + 8];

    if (bytes_read > 0) {
      if (silence_check(bytes_read)) {
        Serial.println(
            "Silence detected! Sending END signal..."); // <-- Breadcrumb!
        if (client && client.connected()) {
          client.write(fb, frame_build(MSG_END, 0, 0, fb, sizeof(fb)));
          client.flush();
        }
        changeState(SENDING);
      } else {
        if (client && client.connected()) {
          client.write(fb, frame_build(MSG_AUDIO, (uint8_t *)sample_buffer,
                                       bytes_read, fb, sizeof(fb)));
        }
      }
    } else {
      // If you see this in the Serial Monitor, your Mic wiring is
      // loose/wrong!
      Serial.println(
          "Warning: 0 bytes read from Mic! Check SCK and WS pins.");
      delay(50); // Small delay to prevent a watchdog crash
    }
    break;
  }
  case SENDING:
    changeState(LOADING_STATE); // immediately show loading
    break;
  case LOADING_STATE:
    loading(); // animate dots
    break;
  case SHOW:
    if (show(reply))
      changeState(IDLE); // return to idle when done
    break;
  }

  // WiFi status dot: 2x2 px with a 1px gutter from the top-left corner
  if (wifi_connected) {
    display.fillRect(1, 1, 2, 2, SSD1306_WHITE);
  }
  display.display();
}
  // ============================================================
  // State transition helper
  // ============================================================
  void changeState(State newState) {
    state = newState;          // update current state
    stateStartTime = millis(); // reset timer for new state
  }

  // ============================================================
  // Loading mode - eyes close then dots animate
  // ============================================================
  void loading() {
    uint32_t now = millis();
    static float closeAmount = EYE_H; // shrinking eye height
    static bool closing = true;       // true during close phase
    if (mode != MODE_LOADING) {       // reset on mode entry
      mode = MODE_LOADING;
      closeAmount = eyeHeight;
      closing = true;
      loadingDots = 0;
    }
    if (closing) {
      closeAmount -= 2.0f; // shrink eyes each frame
      display.clearDisplay();
      int h = max(2, (int)closeAmount); // minimum height of 2px
      int y = EYE_Y + (EYE_H - h) / 2;  // vertically center the shrinking eye
      drawEye(LEFT_EYE_X, y, EYE_W, h);
      drawEye(RIGHT_EYE_X, y, EYE_W, h);
      display.display();
      if (closeAmount <= 2) { // eyes fully closed
        closeAmount = EYE_H;
        closing = false; // switch to dot animation
        display.clearDisplay();
        display.display();
        lastLoadingUpdate = now;
        loadingDots = 0;
      }
      return;
    }
    if (now - lastLoadingUpdate >= LOADING_SPEED) { // cycle dot count 0-3
      lastLoadingUpdate = now;
      loadingDots++;
      if (loadingDots > 3)
        loadingDots = 0;
    }
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    int width = loadingDots * 12; // total dot string width
    if (width == 0)
      width = 12;
    int x = (SCREEN_WIDTH - width) / 2; // center dots horizontally
    display.setCursor(x, 25);
    for (int i = 0; i < loadingDots; i++)
      display.print('.');
    display.display();
  }

  // ============================================================
  // Text wrapping into fixed-width lines
  // ============================================================
  void wrapText(const String &text) {
    lineCount = 0;
    String current = ""; // current line being built
    String word = "";    // current word being accumulated
    for (uint16_t i = 0; i <= text.length(); i++) {
      char c = (i < text.length()) ? text[i]
                                   : ' '; // sentinel space flushes last word
      if (c == ' ' || c == '\n') {
        if (word.length() == 0)
          continue;
        String test = (current.length() == 0) ? word : (current + " " + word);
        if (test.length() > CHARS_PER_LINE) { // word would overflow line
          if (lineCount < MAX_LINES)
            lines[lineCount++] = current;
          current = word; // move word to new line
        } else {
          current = test; // append word to current line
        }
        word = "";
        if (c == '\n') { // explicit newline
          if (current.length() > 0 && lineCount < MAX_LINES)
            lines[lineCount++] = current;
          current = "";
        }
      } else {
        word += c; // accumulate characters into word
      }
    }
    if (current.length() > 0 && lineCount < MAX_LINES)
      lines[lineCount++] = current;
  }

  // ============================================================
  // Show mode - scroll wrapped text on screen
  // ============================================================
  bool show(const String &text) {
    static String activeText = ""; // track current text to detect changes
    static bool initialized = false;
    static bool holdingAtEnd = false; // true when paused at last page
    static uint32_t endHoldStart = 0;
    if (text != activeText || !initialized) { // new text entered
      activeText = text;
      wrapText(activeText);
      textScrollLine = 0;
      lastTextScroll = millis();
      initialized = true;
      holdingAtEnd = false;
      mode = MODE_TEXT;
    }
    uint32_t now = millis();
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    constexpr int LINE_HEIGHT = 16;
    constexpr int VISIBLE_LINES = 4; // lines shown at once on screen
    for (int i = 0; i < VISIBLE_LINES; i++) {
      int index = textScrollLine + i;
      if (index >= lineCount)
        break;
      display.setCursor(0, i * LINE_HEIGHT);
      display.print(lines[index]);
    }
    display.display();
    int maxScroll =
        (int)lineCount - VISIBLE_LINES; // last valid scroll position
    if (maxScroll < 0)
      maxScroll = 0;
    if (!holdingAtEnd) {
      if (now - lastTextScroll >= TEXT_SCROLL_SPEED) {
        lastTextScroll = now;
        if (textScrollLine < maxScroll) {
          textScrollLine++; // scroll down one line
        } else {
          holdingAtEnd = true; // reached end, start hold timer
          endHoldStart = now;
        }
      }
    } else {
      if (now - endHoldStart >= END_HOLD_DELAY) { // hold complete
        initialized = false;
        holdingAtEnd = false;
        return true; // signal show is done
      }
    }
    return false;
  }
