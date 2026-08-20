#include "WString.h"
#include "config.h"
#include "esp32-hal-gpio.h"
#include "mic.h"
#include "roboeyes.h"
#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>

// for message
enum MsgType : uint8_t { MSG_AUDIO = 0x01, MSG_END = 0x02, MSG_TEXT = 0x03 };

constexpr int MAX_LINES = 64;      // max lines for wrapped text
constexpr int CHARS_PER_LINE = 10; // max chars before wrapping

constexpr uint16_t LOADING_SPEED = 300;      // dot animation speed
constexpr uint16_t TEXT_SCROLL_SPEED = 1200; // ms between scroll steps
constexpr uint32_t END_HOLD_DELAY = 2000;    // hold at end of text

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

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("...");
  }

  display.clearDisplay();
  display.setCursor(10, 25);
  display.print("Connected!");
  display.display();

  Serial.print("\n[+] WiFi Connected! ESP32 IP: ");
  Serial.print(WiFi.localIP());
  Serial.printf("  Port: %d\n", TCP_PORT);

  delay(1000);
  tcpServer.begin();

  pinMode(0, INPUT_PULLUP);       // Ensure BOOT button is set as input
  randomSeed(analogRead(A0));     // seed RNG from floating pin
  nextMovement = millis() + 1000; // first eye movement in 1s

  calibrate_mic();

  Serial.println("System Ready. Waiting for button press...");
  delay(500);
}

static uint8_t rx[2048];
static size_t rxlen = 0;

// ============================================================
// Main loop
// ============================================================
void loop() {
  static uint32_t last_btn_press = 0;
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

  // block-read the TCP stream
  if (client && client.connected()) {
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
    idle();
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
