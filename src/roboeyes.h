#ifndef ROBOEYES_H
#define ROBOEYES_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

extern Adafruit_SSD1306 display;

constexpr int LEFT_EYE_X  = 45;   // left eye horizontal origin
constexpr int RIGHT_EYE_X = 80;   // right eye horizontal origin
constexpr int EYE_Y       = 18;   // vertical origin for both eyes
constexpr int EYE_W       = 25;   // default eye width
constexpr int EYE_H       = 30;   // default eye height
constexpr int FOCUS_W     = 30;   // widened eye width for focus mode
constexpr int FOCUS_H     = 36;   // widened eye height for focus mode

constexpr uint16_t FRAME_TIME       = 25;   // ~40fps cap
constexpr uint32_t BLINK_INTERVAL   = 4000; // ms between blinks
constexpr uint16_t BLINK_DURATION   = 140;  // how long a blink lasts

constexpr float MOVE_SMOOTHNESS = 0.12f; // lerp factor for eye position
constexpr float SIZE_SMOOTHNESS = 0.10f; // lerp factor for eye size

enum DisplayMode { MODE_IDLE, MODE_FOCUS, MODE_LOADING, MODE_TEXT };
extern DisplayMode mode;

extern float eyeOffsetX;
extern float eyeOffsetY;
extern float targetX;
extern float targetY;
extern float eyeWidth;
extern float eyeHeight;
extern float targetWidth;
extern float targetHeight;

extern bool blinking;
extern uint32_t lastBlink;

extern uint32_t nextMovement;
extern uint32_t lastFrame;

void drawEye(int x, int y, int w, int h);
void renderEyes();
void renderBlink();
void updateEyeAnimation();
void chooseMovement();
void idle();
bool focus();

#endif
